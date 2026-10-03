"""Evidence-driven safety traceability for docs/SAFETY_TRACEABILITY.md.

The registry (docs/traceability.json) lists, for every requirement, the automated tests and the
saved result folders that verify it, and the documented gaps that keep it below VERIFIED. This
tool recomputes every status from the result files with the rules the project already documents,
writes the generated parts of the page, and checks the page against the evidence.

    python -B tools/traceability.py --check   # exit 1 on any mismatch (CI)
    python -B tools/traceability.py --write   # regenerate generated fields, then check

Standard library only, so it runs before the simulation dependencies are installed.
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from fnmatch import fnmatch
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
APP = 'outputs/visual-servoing-simulation'
DOC = 'docs/SAFETY_TRACEABILITY.md'
REGISTRY = 'docs/traceability.json'
RUNNER = 'tools/run_acceptance_test.py'

VERIFIED, PARTIAL, NOT = 'VERIFIED', 'PARTIALLY VERIFIED', 'NOT VERIFIED'
RANK = {NOT: 0, PARTIAL: 1, VERIFIED: 2}
REQ_ID = re.compile(r'^REQ-\d{2}$')
NOTE = '(do not edit; regenerate with python -B tools/traceability.py --write)'
MARKER = re.compile(r'<!-- BEGIN GENERATED: (?P<name>[\w-]+)[^>]*?-->(?P<body>.*?)<!-- END GENERATED: (?P=name) -->', re.S)
OPENING = re.compile(r'<!-- BEGIN GENERATED: ([\w-]+)')
BLOCKS = {'summary', 'status'}  # Generated as whole lines; the others sit inside a sentence.
RESULT_REF = re.compile(r'results/([A-Za-z0-9_.-]+)/([A-Za-z0-9_.*-]+)')
BARE_RUN = re.compile(r'`(\d{8}-[0-9A-Za-z_.-]+)`')
SHORTHAND = re.compile(r'`(-\d{6})`')
SAME_BOOT = timedelta(seconds=120)
MISSING = object()  # a path element that a record lacks


@dataclass
class Item:
    """One piece of evidence for one requirement, judged by an existing rule."""
    path: str
    outcome: str  # pass | partial | fail | invalid | measured | superseded | missing
    detail: str = ''
    scenario: str | None = None


@dataclass
class Run:
    """A full (non-smoke) acceptance-test run as its verdict.json and manifest.json record it."""
    name: str
    outcome: str
    status: str | None
    complete: bool | None
    uptime_s: float | None
    boot: datetime | None
    ac: bool | None
    mode: str | None
    failures: list = field(default_factory=list)
    judged_by_lower_bound: bool = False


@dataclass
class Result:
    status: str
    tests_found: list
    items: list
    open_gaps: list
    closed_gaps: list
    reasons: list


class Traceability:
    def __init__(self, root: Path = ROOT):
        self.root = Path(root)
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self._json: dict = {}
        self._defs: dict = {}
        self._runs: dict | None = None
        self.registry = self.load(REGISTRY, base=self.root)
        if self.registry is None:
            raise SystemExit(f'Cannot read the registry {REGISTRY}.')
        self.excluded = self.registry.get('excluded_results', {})
        self.git = self._git_available()

    # ------------------------------------------------------------------ input helpers

    def error(self, kind, message):
        text = f'[{kind}] {message}'
        if text not in self.errors:
            self.errors.append(text)

    def warn(self, kind, message):
        text = f'[{kind}] {message}'
        if text not in self.warnings:
            self.warnings.append(text)

    def app(self, relative):
        return self.root / APP / relative

    def load(self, relative, base=None):
        """Parsed JSON, or None when missing or unreadable. Cached."""
        path = (base or self.root / APP) / relative
        if path not in self._json:
            try:
                self._json[path] = json.loads(path.read_text(encoding='utf-8-sig'))
            except FileNotFoundError:
                self._json[path] = None
            except (ValueError, UnicodeError) as exc:
                self.error('invalid-run-data', f'{relative}: not valid JSON ({exc})')
                self._json[path] = None
        return self._json[path]

    def _git_available(self):
        try:
            out = subprocess.run(['git', 'rev-parse', '--show-toplevel'], cwd=self.root, capture_output=True,
                                 text=True, timeout=30)
        except (OSError, subprocess.SubprocessError):
            return False
        return out.returncode == 0 and Path(out.stdout.strip()).resolve() == self.root.resolve()

    def ignored(self, relatives):
        """Files git would not publish (ignored and untracked). Empty when git is unavailable."""
        if not self.git or not relatives:
            return set()
        paths = [f'{APP}/{r}' for r in relatives]
        out = subprocess.run(['git', 'check-ignore', '--stdin', '-z'], cwd=self.root, capture_output=True,
                             input='\0'.join(paths) + '\0', text=True, timeout=60)
        return {p[len(APP) + 1:] for p in out.stdout.split('\0') if p}

    def fresh_restart_s(self):
        if '_fresh' in self.__dict__:
            return self._fresh
        self._fresh = self._read_fresh_restart_s()
        return self._fresh

    def _read_fresh_restart_s(self):
        source = (self.root / RUNNER).read_text(encoding='utf-8-sig')
        for node in ast.parse(source).body:
            if (isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == 'FRESH_RESTART_S' for t in node.targets)
                    and isinstance(node.value, ast.Constant)):
                return float(node.value.value)
        self.error('rule', f'FRESH_RESTART_S not found in {RUNNER}; the fresh-restart rule cannot be applied')
        return None

    def test_exists(self, reference):
        """'tests/test_x.py::name', '::Class.method' or a bare file. tools/ paths are repository-relative."""
        file, _, name = reference.partition('::')
        path = self.root / file if file.startswith('tools/') else self.app(file)
        if path not in self._defs:
            try:
                tree = ast.parse(path.read_text(encoding='utf-8-sig'))
            except (OSError, SyntaxError, UnicodeError):
                self._defs[path] = None
            else:
                names = set()
                for node in ast.walk(tree):
                    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                        names.add(node.name)
                    if isinstance(node, ast.ClassDef):
                        names.update(f'{node.name}.{n.name}' for n in node.body if isinstance(n, ast.FunctionDef))
                self._defs[path] = names
        names = self._defs[path]
        return names is not None and (not name or name in names)

    # ------------------------------------------------------------------ result files

    def acceptance_runs(self):
        """Every acceptance-test run folder with a verdict, keyed by name; smoke runs are kept but flagged."""
        if self._runs is not None:
            return self._runs
        self._runs = {}
        base = self.app('results/acceptance-test')
        for folder in sorted(p for p in base.iterdir() if (p / 'verdict.json').exists()) if base.is_dir() else []:
            self._runs[folder.name] = self.read_run(folder.name)
        return self._runs

    def read_run(self, name):
        rel = f'results/acceptance-test/{name}'
        verdict, manifest = self.load(f'{rel}/verdict.json'), self.load(f'{rel}/manifest.json')
        if verdict is None or manifest is None:
            return Run(name, 'missing', None, None, None, None, None, None)
        status, complete, methods = verdict.get('status'), verdict.get('complete'), verdict.get('methods')
        problems = []
        if status not in ('PASS', 'FAIL', 'SMOKE_ONLY'):
            problems.append(f'unknown verdict status {status!r}')
        if not isinstance(complete, bool):
            problems.append('verdict has no complete flag')
        if not isinstance(methods, list) or not methods:
            problems.append('verdict has no methods')
            methods = []
        failures = []
        for method in methods:
            fails = method.get('failures')
            if not isinstance(fails, list) or method.get('verdict') not in ('PASS', 'FAIL'):
                problems.append(f"method {method.get('label')!r} lacks failures/verdict")
                continue
            if (method['verdict'] == 'FAIL') != bool(fails):
                self.error('contradictory-results', f"{rel}: method {method.get('label')} verdict {method['verdict']} "
                           f'disagrees with its failures list')
            failures += [f"{method.get('label')}: {f}" for f in fails]
        smoke = status == 'SMOKE_ONLY'
        if manifest.get('smoke') is True and not smoke:
            self.error('contradictory-results', f'{rel}: manifest says smoke run, verdict says {status}')
        if status == 'PASS' and (failures or complete is False):
            self.error('contradictory-results', f'{rel}: verdict PASS but a method failed or the run is incomplete')
        if status == 'FAIL' and complete and not failures and methods:
            self.error('contradictory-results', f'{rel}: verdict FAIL but no method lists a failure')
        uptime = manifest.get('uptime_at_start_s')
        uptime = float(uptime) if isinstance(uptime, (int, float)) else None
        boot = None
        try:
            boot = datetime.fromisoformat(manifest['created_utc']) - timedelta(seconds=uptime)
        except (KeyError, TypeError, ValueError):
            pass
        sessions = manifest.get('session_boundaries') or []
        ac = all((b.get('power') or {}).get('ACLineStatus') == 1 for b in sessions) if sessions else None
        modes = {(b.get('power_mode') or {}).get('mode') for b in sessions} - {None}
        mode = None if not modes else ', '.join(sorted(modes))
        if problems:
            outcome = 'invalid'
            failures = problems
        elif smoke:
            outcome = 'smoke'
        elif not complete:
            outcome = 'invalid'
        else:
            outcome = 'pass' if status == 'PASS' else 'fail'
        judged = bool(methods) and all(isinstance(m.get('success_ci'), dict) for m in methods)
        return Run(name, outcome, status, complete, uptime, boot, ac, mode, failures, judged)

    @staticmethod
    def status_of(runs):
        """run_fault_campaign.status_of, repeated here so the summary's stored status can be re-derived."""
        if not runs:
            return 'TESTED'
        if not all(r.get('passed') is True for r in runs):
            return NOT
        return VERIFIED if all(r.get('coverage') == 'full' for r in runs) else PARTIAL

    def expectations(self, path, expect):
        """Check declared field values; returns (outcome, detail) or None when nothing is declared."""
        if not expect:
            return None
        failed, missing = [], []
        for rule in expect:
            data = self.load(f"{path}/{rule['file']}")
            values = None if data is None else self.lookup(data, rule['field'])
            name = f"{rule['file']}:{rule['field']}"
            if rule.get('absent'):
                if data is None:
                    missing.append(rule['file'])
                elif values:
                    failed.append(f'{name} is present ({values!r}); it must be absent')
                continue
            if not values or any(v is MISSING for v in values):
                missing.append(name)  # every element under [*] must record the field
                continue
            for value in values:
                if not self.holds(value, rule, data):
                    failed.append(f'{name} = {value!r} ({self.describe(rule, data)})')
        if missing:
            return 'invalid', 'missing field ' + ', '.join(missing)
        if failed:
            return 'fail', '; '.join(failed)
        return 'pass', f'{len(expect)} declared value(s) hold'

    @staticmethod
    def lookup(data, dotted):
        """Values at a dotted path. [*] spans a list, [k=v] filters it; an element of [*] that lacks
        the rest of the path yields MISSING, so partly recorded data is not mistaken for complete."""
        values = [data]
        for part in re.findall(r'[^.\[\]]+|\[[^\]]*\]', dotted):
            nxt = []
            for v in values:
                if v is MISSING:
                    nxt.append(MISSING)
                elif '=' in part:  # [key=value]: list elements whose key equals value
                    key, _, wanted = part[1:-1].partition('=')
                    nxt += [x for x in v if isinstance(x, dict) and str(x.get(key)) == wanted] if isinstance(v, list) else []
                elif part == '[*]':
                    nxt += v if isinstance(v, list) else []
                elif part.startswith('['):
                    i = int(part[1:-1])
                    nxt.append(v[i] if isinstance(v, list) and i < len(v) else MISSING)
                else:
                    nxt.append(v[part] if isinstance(v, dict) and part in v else MISSING)
            values = nxt
        return [] if values and all(v is MISSING for v in values) else values

    def limit(self, value):
        """A literal, or a reference to an existing rule: '$FRESH_RESTART_S' or '$<repo file>#<field>'."""
        if not (isinstance(value, str) and value.startswith('$')):
            return value
        if value == '$FRESH_RESTART_S':
            return self.fresh_restart_s()
        file, _, dotted = value[1:].partition('#')
        data = self.load(file, base=self.root)
        found = None if data is None else self.lookup(data, dotted)
        if not found:
            self.error('rule', f'limit {value} not found')
            return None
        return found[0]

    def reference(self, value, data):
        """'@<field>' compares with another field of the same file (e.g. passed against planned)."""
        if isinstance(value, str) and value.startswith('@'):
            found = self.lookup(data, value[1:]) if data is not None else []
            return found[0] if found and found[0] is not MISSING else MISSING
        return self.limit(value)

    def holds(self, value, rule, data=None):
        if 'equals' in rule:
            return value == self.reference(rule['equals'], data)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return False
        most, least = self.reference(rule.get('at_most'), data), self.reference(rule.get('at_least'), data)
        if ('at_most' in rule and most in (None, MISSING)) or ('at_least' in rule and least in (None, MISSING)):
            return False
        return (most is None or value <= most) and (least is None or value >= least)

    def describe(self, rule, data=None):
        return ', '.join(f'{k} {self.reference(rule[k], data)!r}' for k in ('equals', 'at_most', 'at_least') if k in rule)

    # ------------------------------------------------------------------ evidence per requirement

    def evidence(self, req, entry):
        kind, path = entry.get('kind'), entry.get('path', '')
        if kind == 'acceptance_series':
            return [self.run_item(f'results/acceptance-test/{r.name}', r, entry)
                    for r in self.acceptance_runs().values()
                    if r.name >= entry['since'] and r.outcome != 'smoke']
        if not self.app(path).is_dir():
            return [Item(path, 'missing', 'folder not found')]
        if kind == 'acceptance':
            name = path.rsplit('/', 1)[-1]
            run = self.acceptance_runs().get(name) or self.read_run(name)
            return [self.run_item(path, run, entry)]
        if kind == 'fault_campaign':
            return self.fault_items(req, path, entry)
        if kind == 'margin':
            data = self.load(f'{path}/margin.json')
            if data is None:
                return [Item(path, 'missing', 'margin.json not found')]
            if data.get('status') != 'MEASURED':
                return [Item(path, 'invalid', f"margin status {data.get('status')!r} (only MEASURED runs count)")]
            checked = self.expectations(path, entry.get('expect'))
            return [Item(path, *(checked or ('measured', 'characterisation, no pass/fail')))]
        if kind == 'fields':
            checked = self.expectations(path, entry.get('expect'))
            if checked is None:
                self.error('registry', f'{req}: {path} is kind "fields" but declares no expected values')
                return []
            return [Item(path, *checked)]
        if kind == 'measurement':
            file = entry.get('file')
            if file and not self.app(f'{path}/{file}').exists():
                return [Item(path, 'missing', f'{file} not found')]
            return [Item(path, 'measured', entry.get('note', 'measurement; no pass/fail'))]
        self.error('registry', f'{req}: unknown evidence kind {kind!r} for {path}')
        return []

    def run_item(self, path, run, entry):
        if run.outcome == 'missing':
            return Item(path, 'missing', 'verdict.json or manifest.json not found')
        if run.outcome in ('invalid', 'smoke'):
            return Item(path, 'invalid', 'smoke run' if run.outcome == 'smoke' else
                        'incomplete or malformed run: ' + '; '.join(run.failures or ['complete is false']))
        if entry.get('judge') == 'fields':
            # Only the declared values matter for this requirement (e.g. a safety metric in a run
            # whose overall verdict failed on another requirement's gate).
            checked = self.expectations(path, entry.get('expect'))
            if checked is None:
                return Item(path, 'invalid', 'judge "fields" needs declared values')
            return Item(path, *checked)
        if run.outcome == 'fail':
            return Item(path, 'fail', 'acceptance FAIL: ' + '; '.join(run.failures))
        checked = self.expectations(path, entry.get('expect'))
        return Item(path, *(checked or ('pass', 'acceptance PASS')))

    def fault_items(self, req, path, entry):
        data = self.load(f'{path}/summary.json')
        if data is None:
            return [Item(path, 'missing', 'summary.json not found')]
        scenarios, runs = data.get('scenarios'), data.get('runs')
        if not isinstance(scenarios, list) or not isinstance(runs, dict):
            return [Item(path, 'invalid', 'summary.json lacks scenarios or runs')]
        wanted, superseded = entry.get('scenarios'), entry.get('superseded', {})
        items = []
        for sc in scenarios:
            sid = sc.get('id')
            if req not in (sc.get('requirements') or []) or (wanted and sid not in wanted):
                continue
            records = runs.get(str(sid).replace('-', '_'))
            if not isinstance(records, list) or any('passed' not in r or 'coverage' not in r for r in records):
                items.append(Item(path, 'invalid', 'no per-repeat records', sid))
                continue
            derived = self.status_of(records)
            passed = sum(r['passed'] is True for r in records)
            if (derived != sc.get('status') or passed != sc.get('passed') or len(records) != sc.get('repeats')):
                self.error('contradictory-results', f"{path}: scenario {sid} stores {sc.get('status')} "
                           f"{sc.get('passed')}/{sc.get('repeats')} but its repeats give {derived} {passed}/{len(records)}")
            detail = f'{passed}/{len(records)} {derived}'
            if sid in superseded:
                note = superseded[sid]
                if not (isinstance(note, dict) and note.get('reason') and note.get('replaced_by')):
                    self.error('registry', f'{req}: superseded {path} {sid} needs a reason and replaced_by')
                    note = {'reason': str(note), 'replaced_by': []}
                items.append(Item(path, 'superseded', f"{detail}; superseded: {note['reason']}", sid))
                self.replacements.append((req, path, sid, note.get('replaced_by', [])))
            elif derived == VERIFIED:
                items.append(Item(path, 'pass', detail, sid))
            elif derived == PARTIAL:
                items.append(Item(path, 'partial', detail + ' (partial coverage)', sid))
            elif derived == NOT:
                items.append(Item(path, 'fail', detail, sid))
            else:
                items.append(Item(path, 'invalid', detail, sid))
        for sid in (wanted or []):
            if not any(i.scenario == sid for i in items):
                self.error('registry', f'{req}: {path} has no scenario {sid} that lists {req}')
        for sid in superseded:
            if not any(i.scenario == sid for i in items):
                self.error('registry', f'{req}: superseded scenario {sid} is not in {path}')
        if not items:
            self.error('registry', f'{req}: cites {path}, but no scenario there lists {req}')
        return items

    # ------------------------------------------------------------------ rules

    def fresh_series(self):
        """The pre-declared consecutive fresh-restart rule, counted under every reading of its wording."""
        rule = self.registry['rules']['consecutive_fresh_acceptance']
        reading = rule.get('interpretation', {})
        limit = self.fresh_restart_s()
        literal, strict, ambiguities, boots = [], [], [], []
        breaks = False
        for run in self.acceptance_runs().values():
            if run.name <= rule['after'] or run.outcome == 'smoke':
                continue
            if run.outcome in ('invalid', 'missing'):
                if reading.get('incomplete_run_breaks_streak') is None:
                    ambiguities.append(f'{run.name} is incomplete; the rule does not say whether that breaks the streak')
                    breaks = True
                elif reading['incomplete_run_breaks_streak']:
                    literal, strict, boots = [], [], []
                continue
            fresh = limit is not None and run.uptime_s is not None and run.uptime_s <= limit
            if not fresh:
                if run.outcome == 'fail':
                    if reading.get('non_fresh_failure_breaks_streak') is None:
                        ambiguities.append(f'{run.name} failed without a fresh restart; the rule does not say '
                                           'whether that breaks the streak')
                        breaks = True
                    elif reading['non_fresh_failure_breaks_streak']:
                        literal, strict, boots = [], [], []
                continue
            if run.outcome == 'fail':
                literal, strict, boots = [], [], []
                continue
            literal.append(run.name)
            counts = True
            if not (run.ac is True and run.mode == 'Best performance'):
                if reading.get('requires_ac_and_best_performance') is None:
                    ambiguities.append(f'{run.name} counts by uptime, but AC power / Best performance (REQ-23) '
                                       'was not recorded as met')
                counts = reading.get('requires_ac_and_best_performance') is False
            if run.boot is not None and any(abs(run.boot - b) <= SAME_BOOT for b in boots):
                if reading.get('same_restart_counts') is None:
                    ambiguities.append(f'{run.name} ran after the same restart as an earlier counted run '
                                       f'(boot {run.boot:%Y-%m-%d %H:%M} UTC); it counts under "≤ 60 min uptime" '
                                       'but not under "one run per restart"')
                counts = counts and reading.get('same_restart_counts') is True
            if run.boot is not None:
                boots.append(run.boot)
            if counts:
                strict.append(run.name)
        n = rule['n']
        satisfied = len(literal) >= n and len(strict) >= n and not breaks
        if not satisfied and len(literal) >= n:
            self.error('ambiguous-rule', 'the fresh-restart rule is met under one reading but not another '
                       f'({len(literal)} vs {len(strict)} of {n}); record the intended reading in '
                       f'{REGISTRY} rules.consecutive_fresh_acceptance.interpretation')
        for text in ambiguities:
            self.warn('ambiguous-rule', text)
        return dict(n=n, literal=literal, strict=strict, satisfied=satisfied, ambiguities=ambiguities, limit=limit)

    def gap_closed(self, gap):
        closed = gap.get('closed_by')
        if not closed:
            return False
        if 'decision' in closed:
            return True
        if closed.get('rule') == 'consecutive_fresh_acceptance':
            return self.series['satisfied']
        self.error('registry', f"gap {gap.get('id')}: unknown closing rule {closed!r}")
        return False

    # ------------------------------------------------------------------ status

    def compute(self):
        self.series = self.fresh_series()
        self.results, self.evidence_files, self.replacements = {}, {}, []
        for req, spec in self.registry['requirements'].items():
            if not REQ_ID.match(req):
                self.error('requirement-id', f'malformed requirement ID {req!r} in the registry')
            self.results[req] = self.compute_one(req, spec)
        for f in sorted(self.ignored(sorted(self.evidence_files))):
            self.error('unpublished-evidence', f"{f} (evidence for {', '.join(sorted(self.evidence_files[f]))}) is "
                       'git-ignored, so CI and readers cannot see it')
        return self.results

    def compute_one(self, req, spec):
        tests = spec.get('tests', [])
        found = [t for t in tests if self.test_exists(t)]
        for t in tests:
            if t not in found:
                self.error('missing-test', f'{req}: test {t} not found')
        items = [i for e in spec.get('evidence', []) for i in self.evidence(req, e)]
        for owner, path, sid, newer in [r for r in self.replacements if r[0] == req]:
            for replacement in newer:
                if not any(i.path == replacement and i.scenario == sid and i.outcome == 'pass' for i in items):
                    self.error('registry', f'{req}: {path} {sid} is superseded by {replacement}, but that campaign '
                               f'is not cited for {req} or does not pass {sid}')
        for i in items:
            if i.outcome == 'missing':
                self.error('missing-evidence', f'{req}: {i.path}: {i.detail}')
            elif i.outcome == 'invalid':
                self.error('invalid-run-data', f"{req}: {i.path}{' ' + i.scenario if i.scenario else ''}: {i.detail}")
        published = {f for e in spec.get('evidence', []) for f in self.key_files(e)}
        published |= {f'{i.path}/{f}' for i in items if i.path.startswith('results/acceptance-test/')
                      for f in ('verdict.json', 'manifest.json')}
        for f in published:
            self.evidence_files.setdefault(f, set()).add(req)
        gaps = spec.get('gaps', [])
        closed = [g for g in gaps if self.gap_closed(g)]
        open_ = [g for g in gaps if g not in closed]
        covered = {}
        for g in gaps:
            for c in g.get('covers', []):
                covered[(c['path'], c.get('scenario'))] = g
        blocking = []
        for i in items:
            if i.outcome != 'fail':
                continue
            gap = covered.get((i.path, i.scenario))
            if gap is None:
                self.error('contradictory-results', f"{req}: {i.path}{' ' + i.scenario if i.scenario else ''} "
                           f'records a failure ({i.detail}) that no documented gap explains')
                blocking.append(i)
            elif gap not in closed:
                blocking.append(i)
        for (path, scenario), gap in covered.items():
            if not any(i.path == path and i.scenario == scenario and i.outcome == 'fail' for i in items):
                self.error('registry', f"{req}: gap {gap['id']} covers {path} {scenario or ''} but that evidence "
                           'records no failure (stale gap entry)')
        passes = [i for i in items if i.outcome == 'pass']
        partials = [i for i in items if i.outcome == 'partial']
        reasons = []
        if spec.get('implemented', True) is False:
            status, reasons = NOT, ['mechanism not implemented']
        elif not found and not passes and not partials:
            status, reasons = NOT, ['no automated test and no passing evidence']
        else:
            if not found:
                reasons.append('no automated test found')
            if not passes:
                reasons.append('no evidence item meets its rule')
            reasons += [f"gap: {g['id']}" for g in open_]
            reasons += [f'partial coverage: {i.path} {i.scenario}' for i in partials]
            reasons += [f"failure: {i.path}{' ' + i.scenario if i.scenario else ''}" for i in blocking]
            status = PARTIAL if reasons else VERIFIED
        return Result(status, found, items, open_, closed, reasons)

    @staticmethod
    def key_files(entry):
        kind, path = entry.get('kind'), entry.get('path', '')
        if kind == 'acceptance':
            return [f'{path}/verdict.json', f'{path}/manifest.json']
        if kind == 'fault_campaign':
            return [f'{path}/summary.json']
        if kind == 'margin':
            return [f'{path}/margin.json', f'{path}/manifest.json']
        if kind == 'measurement' and entry.get('file'):
            return [f"{path}/{entry['file']}"]
        return sorted({f"{path}/{e['file']}" for e in entry.get('expect', [])})

    # ------------------------------------------------------------------ generated text

    def generated(self):
        counts = {s: sorted(r for r, v in self.results.items() if v.status == s) for s in (VERIFIED, PARTIAL, NOT)}
        total = len(self.results)
        summary = (f"**Summary:** {total} requirements. {len(counts[VERIFIED])} are VERIFIED, "
                   f"{len(counts[PARTIAL])} are PARTIALLY VERIFIED and {len(counts[NOT])} "
                   f"{'is' if len(counts[NOT]) == 1 else 'are'} NOT VERIFIED"
                   + (f" ({', '.join(counts[NOT])})." if counts[NOT] else '.'))
        s = self.series
        streak = (f"{len(s['literal'])} of {s['n']} so far"
                  + (': ' + ', '.join(f'`{r}`' for r in s['literal']) if s['literal'] else '')
                  + (f"; {len(s['strict'])} of {s['n']} if each run must follow its own restart"
                     if len(s['strict']) != len(s['literal']) else ''))
        limit = s['limit']
        judged = [r for r in self.listed_runs() if r.judged_by_lower_bound and r.outcome in ('pass', 'fail')]
        fresh_pass = [r for r in judged if r.outcome == 'pass' and limit and r.uptime_s is not None and r.uptime_s <= limit]
        ruled = (f"{len(judged)} saved runs judged under the rule; {sum(r.outcome == 'pass' for r in judged)} passed, "
                 f"including {len(fresh_pass)} after a fresh restart")
        return {'summary': summary, 'fresh-streak': streak, 'rule-runs': ruled, 'status': self.status_section()}

    def listed_runs(self):
        """Acceptance runs used as evidence by any requirement (series or cited), in name order."""
        used = {i.path.rsplit('/', 1)[-1] for r in self.results.values() for i in r.items
                if i.path.startswith('results/acceptance-test/')}
        return [run for name, run in self.acceptance_runs().items() if name in used]

    def status_section(self):
        L = ['Computed by `tools/traceability.py` from the registry `docs/traceability.json` and the result files. '
             'Statuses in the tables above are copied from here.', '',
             '| ID | Status | Tests found | Evidence (rule outcome) | Why not VERIFIED |', '|---|---|---|---|---|']
        for req in sorted(self.results):
            r = self.results[req]
            tally = {}
            for i in r.items:
                tally[i.outcome] = tally.get(i.outcome, 0) + 1
            evidence = ', '.join(f'{n} {k}' for k, n in sorted(tally.items())) or 'none'
            why = '<br>'.join(r.reasons) if r.reasons else '—'
            L.append(f'| {req} | {r.status} | {len(r.tests_found)} | {evidence} | {why} |')
        since = min((e['since'] for spec in self.registry['requirements'].values() for e in spec.get('evidence', [])
                     if e.get('kind') == 'acceptance_series'), default=None)
        L += ['', '**Acceptance runs used as evidence** (`verdict.json` and `manifest.json`): the runs a requirement '
              'cites' + (f', and every full run since `{since}`' if since else '') + ':', '',
              '| Run | Uptime at start | Fresh restart (≤ 60 min) | AC power / power mode | Verdict | Judged on the lower bound |',
              '|---|---|---|---|---|---|']
        limit = self.series['limit']
        for run in self.listed_runs():
            up = '—' if run.uptime_s is None else f'{run.uptime_s / 60:.0f} min'
            fresh = '—' if run.uptime_s is None or limit is None else ('yes' if run.uptime_s <= limit else 'no')
            power = f"{'yes' if run.ac else 'no' if run.ac is False else '—'} / {run.mode or '—'}"
            verdict = {'pass': 'PASS', 'fail': 'FAIL'}.get(run.outcome, 'incomplete')
            L.append(f"| `{run.name}` | {up} | {fresh} | {power} | {verdict} | {'yes' if run.judged_by_lower_bound else 'no'} |")
        s = self.series
        rule = self.registry['rules']['consecutive_fresh_acceptance']
        L += ['', f"**Pre-declared fresh-restart rule** (REQ-03, REQ-19): {s['n']} consecutive fresh-restart passes "
              f"after `{rule['after']}`. Counted by uptime: {len(s['literal'])} "
              f"({', '.join(s['literal']) or 'none'}). One run per restart, AC power and Best performance: "
              f"{len(s['strict'])}. Met: {'yes' if s['satisfied'] else 'no'}."]
        if s['ambiguities']:
            L += ['', 'Open questions in the rule, reported and not resolved by the tool:']
            L += [f'- {a}' for a in s['ambiguities']]
        rows = {}
        for req, spec in self.registry['requirements'].items():
            for entry in spec.get('evidence', []):
                if entry.get('kind') != 'fault_campaign':
                    continue
                for i in self.results[req].items:
                    if i.path == entry['path'] and i.scenario:
                        rows.setdefault((i.path, i.scenario), [i, set()])[1].add(req)
        L += ['', '**Fault-injection scenarios cited as evidence** (`summary.json`, status from `status_of`):', '',
              '| Campaign | Scenario | Cited for | Repeats passed and status | Use |', '|---|---|---|---|---|']
        notes = []
        for (path, scenario), (item, reqs) in sorted(rows.items()):
            detail, _, note = item.detail.partition('; superseded: ')
            use = item.outcome
            if item.outcome == 'superseded':
                if note not in notes:
                    notes.append(note)
                use = f'superseded ({notes.index(note) + 1})'
            L.append(f"| `{path.rsplit('/', 1)[-1]}` | {scenario} | {', '.join(sorted(reqs))} | {detail} | {use} |")
        if notes:
            L += [''] + [f'({n}) {note}' + ('<br>' if n < len(notes) else '') for n, note in enumerate(notes, 1)]
        gaps = [(req, g, 'closed' if g in r.closed_gaps else 'open') for req, r in sorted(self.results.items())
                for g in r.open_gaps + r.closed_gaps]
        if gaps:
            L += ['', '**Documented gaps** (from the registry; a human decides when one is closed, except where a rule closes it):', '']
            for req, g, state in sorted(gaps, key=lambda x: (x[0], x[1]['id'])):
                how = g.get('closed_by', {})
                if 'decision' in how:
                    how = f" Closed by decision: {how['decision']}"
                elif how:
                    how = f" Closes when the rule `{how.get('rule')}` is met{' (met)' if state == 'closed' else ''}."
                else:
                    how = ''
                L.append(f"- **{req} `{g['id']}`** ({state}): {g['reason']}{how}")
        return '\n'.join(L)

    # ------------------------------------------------------------------ document

    def rows(self, text):
        """Requirement rows of the traceability tables and the hazard table, by section."""
        current, found = None, {'trace': [], 'hazard': []}
        for number, line in enumerate(text.split('\n'), 1):
            if line.startswith('## '):
                current = {'## Traceability table': 'trace', '## Hazard analysis': 'hazard'}.get(line.strip())
            if current and re.match(r'^\|\s*REQ-', line):
                found[current].append((number, line))
        return found

    def check_document(self, text, expected):
        registry_ids = set(self.registry['requirements'])
        rows = self.rows(text)
        for section, label in (('trace', 'traceability tables'), ('hazard', 'hazard table')):
            ids = [line.split('|')[1].strip() for _, line in rows[section]]
            for i in sorted({i for i in ids if ids.count(i) > 1}):
                self.error('requirement-id', f'{i} appears {ids.count(i)} times in the {label}')
            for i in sorted(set(ids) - registry_ids):
                self.error('requirement-id', f'{i} is in the {label} but not in the registry')
            for i in sorted(registry_ids - set(ids)):
                self.error('requirement-id', f'{i} is in the registry but missing from the {label}')
            for i in ids:
                if not REQ_ID.match(i):
                    self.error('requirement-id', f'malformed requirement ID {i!r} in the {label}')
        for number, line in rows['trace']:
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            req, status = cells[0], cells[-1]
            if req not in self.results:
                continue
            computed = self.results[req].status
            if status != computed:
                kind = 'claim-stronger-than-evidence' if RANK.get(status, -1) > RANK[computed] else 'wrong-status'
                self.error(kind, f'line {number}: {req} says {status!r}, the evidence gives {computed!r} '
                           f"({'; '.join(self.results[req].reasons) or 'all rules met'})")
            for test in self.registry['requirements'][req].get('tests', []):
                name = test.partition('::')[2].split('.')[-1] or test.rsplit('/', 1)[-1]
                if name not in line:
                    self.error('registry', f'{req}: registry test {test} is not named in the page row (line {number})')
        names = [m.group('name') for m in MARKER.finditer(text)]
        openings = OPENING.findall(text)
        for name in sorted(set(openings)):
            if openings.count(name) > 1:
                self.error('generated', f'generated field {name!r} appears {openings.count(name)} times')
            if name not in names:
                self.error('generated', f'generated field {name!r} has no matching END marker')
        for name in sorted(set(expected) - set(openings)):
            self.error('generated', f'generated field {name!r} is missing its markers in {DOC}')
        for m in MARKER.finditer(text):
            name = m.group('name')
            if name not in expected:
                self.error('generated', f'unknown generated field {name!r}')
            elif m.group('body') != self.body(name, expected[name]):
                self.error('stale-generated', f'generated field {name!r} differs from the evidence '
                           f'(hand-edited, or not regenerated after the results changed)')
        self.check_paths(text)
        self.check_claims(text)

    def body(self, name, content):
        return f'\n{content}\n' if name in BLOCKS else content

    def check_paths(self, text):
        cited, files = set(), []
        for category, name in sorted(set(RESULT_REF.findall(text))):
            pattern = f'results/{category}/{name}'
            base = self.app(f'results/{category}')
            matches = sorted(f'results/{category}/{p.name}' for p in base.iterdir()
                             if fnmatch(p.name, name)) if base.is_dir() else []
            if not matches:
                self.error('stale-path', f'{DOC} cites {pattern}, which does not exist')
            cited.update(matches)
        folders = self.result_folders(all_categories=True)
        for run in sorted(set(BARE_RUN.findall(text))):
            matches = [f for f in folders if f.rsplit('/', 1)[-1] == run]
            if not matches:
                self.error('stale-path', f'{DOC} cites run `{run}`, which matches no result folder')
            elif len(matches) > 1:
                self.warn('ambiguous-path', f"run `{run}` matches several folders: {', '.join(matches)}")
            cited.update(matches)
        for short in sorted(set(SHORTHAND.findall(text))):
            self.warn('ambiguous-path', f'`{short}` is shorthand the tool cannot resolve; write the full run name')
        for folder in sorted(cited):
            path = self.app(folder)
            if path.is_dir():
                files += [f'{folder}/{p.name}' for p in sorted(path.iterdir()) if p.is_file()]
        hidden = self.ignored(files)
        for folder in sorted(cited):
            inside = [f for f in files if f.startswith(folder + '/')]
            if inside and all(f in hidden for f in inside):
                self.error('unpublished-evidence', f'{DOC} cites {folder}, but all its files are git-ignored')
        self.cited = cited

    def check_claims(self, text):
        for claim in self.registry.get('claims', []):
            found = re.findall(claim['pattern'], text)
            if len(found) != 1:
                self.error('stale-claim', f"claim {claim['id']!r}: pattern found {len(found)} times in {DOC} "
                           '(text changed; update the claim in the registry)')
                continue
            data = self.load(claim['source'])
            values = None if data is None else [v for v in self.lookup(data, claim['field']) if v not in (None, MISSING)]
            if not values:
                self.error('missing-evidence', f"claim {claim['id']!r}: {claim['source']}:{claim['field']} not found")
                continue
            agg = {'max': max, 'min': min, 'sum': sum, 'count': len}.get(claim.get('agg'), lambda v: v[0])
            value = agg(values) * claim.get('scale', 1)
            expected = claim.get('format', '{}').format(value)
            if found[0] != expected:
                self.error('outdated-value', f"claim {claim['id']!r}: the page says {found[0]}, "
                           f"{claim['source']} gives {expected}")

    def result_folders(self, all_categories=False):
        base = self.app('results')
        categories = (sorted(p.name for p in base.iterdir() if p.is_dir()) if all_categories and base.is_dir()
                      else self.registry.get('scan', []))
        out = []
        for category in categories:
            folder = base / category
            if folder.is_dir():
                out += [f'results/{category}/{p.name}' for p in sorted(folder.iterdir()) if p.is_dir()]
        return out

    def check_unreferenced(self):
        referenced = set(self.cited) | set(self.excluded)
        for spec in self.registry['requirements'].values():
            referenced.update(e['path'] for e in spec.get('evidence', []) if 'path' in e)
        for r in self.results.values():
            referenced.update(i.path for i in r.items)
        candidates = self.result_folders()
        files = {f: [str(p.relative_to(self.app(''))).replace('\\', '/') for p in sorted(self.app(f).rglob('*'))
                     if p.is_file() and 'traces' not in p.parts] for f in candidates}
        hidden = self.ignored(sorted(x for v in files.values() for x in v))
        for folder in candidates:
            if folder in referenced or (files[folder] and all(f in hidden for f in files[folder])):
                continue
            self.error('unreferenced-result', f'{folder} is not cited as evidence and not listed in '
                       f'excluded_results with a reason')
        sinces = [e['since'] for spec in self.registry['requirements'].values() for e in spec.get('evidence', [])
                  if e.get('kind') == 'acceptance_series']
        after = self.registry['rules']['consecutive_fresh_acceptance']['after']
        for path, reason in sorted(self.excluded.items()):
            if not str(reason).strip():
                self.error('registry', f'excluded result {path} has no reason')
            name = path.rsplit('/', 1)[-1]
            run = self.acceptance_runs().get(name) if path.startswith('results/acceptance-test/') else None
            if run and run.outcome != 'smoke' and (name > after or any(name >= since for since in sinces)):
                self.error('registry', f'{path} cannot be excluded: every full run in the acceptance series is '
                           'evidence (a failure needs a documented gap instead)')

    # ------------------------------------------------------------------ entry point

    def run(self, write=False):
        self.compute()
        expected = self.generated()
        doc_path = self.root / DOC
        with open(doc_path, encoding='utf-8', newline='') as handle:
            raw = handle.read()
        newline = '\r\n' if '\r\n' in raw else '\n'  # keep the checkout's line endings
        text = raw.replace('\r\n', '\n')
        if write:
            updated = self.render(text, expected)
            if updated != text:
                with open(doc_path, 'w', encoding='utf-8', newline='') as handle:
                    handle.write(updated.replace('\n', newline))
                text = updated
        self.check_document(text, expected)
        self.check_unreferenced()
        return not self.errors

    def render(self, text, expected):
        def marker(m):
            name = m.group('name')
            if name not in expected:
                return m.group(0)
            return (f'<!-- BEGIN GENERATED: {name} {NOTE} -->{self.body(name, expected[name])}'
                    f'<!-- END GENERATED: {name} -->')
        text = MARKER.sub(marker, text)
        lines = text.split('\n')
        for number, line in self.rows(text)['trace']:
            cells = line.rstrip().split('|')
            req = cells[1].strip()
            if req in self.results and len(cells) >= 3:
                cells[-2] = f' {self.results[req].status} '
                lines[number - 1] = '|'.join(cells)
        return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='validate only (default); exit 1 on any mismatch')
    mode.add_argument('--write', action='store_true', help='regenerate the generated fields, then validate')
    parser.add_argument('--root', type=Path, default=ROOT, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')  # a Windows console may not encode '≤' or '—'
    tool = Traceability(args.root)
    ok = tool.run(write=args.write)
    for line in tool.warnings:
        print('warning:', line)
    for line in tool.errors:
        print('error:', line)
    counts = {}
    for r in tool.results.values():
        counts[r.status] = counts.get(r.status, 0) + 1
    print(f"{len(tool.results)} requirements: " + ', '.join(f'{counts.get(s, 0)} {s}' for s in (VERIFIED, PARTIAL, NOT))
          + ('' if tool.git else ' (git not available: publication not checked)'))
    print('Traceability OK.' if ok else f'Traceability check FAILED: {len(tool.errors)} problem(s).')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
