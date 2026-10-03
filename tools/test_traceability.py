"""Tests for tools/traceability.py on small synthetic projects (no simulation dependencies)."""
from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import traceability as tr  # noqa: E402

APP = tr.APP
NOTE = tr.NOTE
FAIL_RUN = '20261003-100000'


def write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=1), encoding='utf-8')


def method(label, failures=()):
    return {'label': label, 'verdict': 'FAIL' if failures else 'PASS', 'failures': list(failures),
            'control_misses': 1 if failures else 0,
            'success_ci': {'ci_low': 0.97, 'trials': 149, 'successes': 149}}


def doc_text(ids=('REQ-01', 'REQ-02', 'REQ-03'), statuses=None, hazard_ids=None):
    statuses = statuses or {}
    rows = '\n'.join(f'| {i} | Requirement {i}. | `tests/test_demo.py::test_a` `test_b` | evidence | '
                     f"{statuses.get(i, 'NOT VERIFIED')} |" for i in ids)
    hazards = '\n'.join(f'| {i} | consequence | High | residual |' for i in (hazard_ids or ids))
    return f"""# Traceability

<!-- BEGIN GENERATED: summary {NOTE} -->
x
<!-- END GENERATED: summary -->

Streak (<!-- BEGIN GENERATED: fresh-streak {NOTE} -->x<!-- END GENERATED: fresh-streak -->).
The miss in `{FAIL_RUN}` is open. Its uptime was 5 min.

## Traceability table

| ID | Requirement | Verification | Evidence | Status |
|---|---|---|---|---|
{rows}

## Hazard analysis

| ID | Consequence | Severity | Residual |
|---|---|---|---|
{hazards}

Runs: <!-- BEGIN GENERATED: rule-runs {NOTE} -->x<!-- END GENERATED: rule-runs -->.

## Machine-checked status

<!-- BEGIN GENERATED: status {NOTE} -->
x
<!-- END GENERATED: status -->
"""


class Project:
    """A minimal repository: runner constant, one test file, result folders, registry and page."""

    def __init__(self, root: Path):
        self.root = root
        self.app = root / APP
        (root / 'tools').mkdir(parents=True)
        (root / 'tools/run_acceptance_test.py').write_text('FRESH_RESTART_S = 3600\n', encoding='utf-8')
        (self.app / 'tests').mkdir(parents=True)
        (self.app / 'tests/test_demo.py').write_text('def test_a():\n    pass\n\n\ndef test_b():\n    pass\n',
                                                     encoding='utf-8')
        self.clock = 0
        self.registry = {
            'rules': {'consecutive_fresh_acceptance': {'n': 5, 'after': FAIL_RUN, 'interpretation': {}}},
            'scan': ['acceptance-test', 'fault-injection'],
            'excluded_results': {},
            'requirements': {
                'REQ-01': {'tests': ['tests/test_demo.py::test_a'],
                           'evidence': [{'kind': 'fault_campaign', 'path': 'results/fault-injection/C1'}]},
                'REQ-02': {'tests': ['tests/test_demo.py::test_b'],
                           'evidence': [{'kind': 'acceptance_series', 'since': FAIL_RUN}],
                           'gaps': [{'id': 'miss', 'reason': 'one unexplained miss',
                                     'covers': [{'path': f'results/acceptance-test/{FAIL_RUN}'}],
                                     'closed_by': {'rule': 'consecutive_fresh_acceptance'}}]},
                'REQ-03': {'tests': ['tests/test_demo.py::test_a'], 'evidence': []},
            },
            'claims': [{'id': 'uptime', 'pattern': r'Its uptime was (\d+) min',
                        'source': f'results/acceptance-test/{FAIL_RUN}/manifest.json',
                        'field': 'uptime_at_start_s', 'scale': 1 / 60, 'format': '{:.0f}'}],
        }
        self.campaign('C1', passed=[True] * 5)
        self.run(FAIL_RUN, failures=['control_misses 1'], uptime=300)
        self.doc = doc_text()

    # -- builders
    def run(self, name, failures=(), uptime=600.0, complete=True, smoke=False, same_boot_as_previous=False,
            ac=1, mode='Best performance'):
        if not same_boot_as_previous:
            self.clock += 3  # hours: every run after its own restart unless asked otherwise
        created = f'2026-10-03T{self.clock:02d}:00:00+00:00' if self.clock < 24 else f'2026-10-04T{self.clock - 24:02d}:00:00+00:00'
        if same_boot_as_previous:
            created = created.replace(':00:00+', ':30:00+')
            uptime = uptime + 1800
        status = 'SMOKE_ONLY' if smoke else ('FAIL' if failures or not complete else 'PASS')
        folder = self.app / 'results/acceptance-test' / name
        write_json(folder / 'verdict.json', {'status': status, 'complete': complete,
                                             'methods': [method('SIFT', failures), method('Learned GPU')]})
        write_json(folder / 'manifest.json', {'created_utc': created, 'uptime_at_start_s': uptime, 'smoke': smoke,
                                              'session_boundaries': [{'power': {'ACLineStatus': ac},
                                                                      'power_mode': {'mode': mode}}]})
        return folder

    def campaign(self, name, passed, requirement='REQ-01', status=None, coverage='full'):
        runs = [{'passed': p, 'coverage': coverage} for p in passed]
        write_json(self.app / 'results/fault-injection' / name / 'summary.json', {
            'scenarios': [{'id': 'watchdog-stop', 'requirements': [requirement], 'passed': sum(passed),
                           'repeats': len(passed), 'coverage': coverage,
                           'status': status or tr.Traceability.status_of(runs)}],
            'runs': {'watchdog_stop': runs}})

    def fresh_passes(self, count, start=1):
        for i in range(start, start + count):
            self.run(f'20261003-1{i:02d}000', uptime=600)

    def save(self):
        write_json(self.root / tr.REGISTRY, self.registry)
        (self.root / tr.DOC).write_text(self.doc, encoding='utf-8')

    def tool(self, write=False):
        self.save()
        tool = tr.Traceability(self.root)
        ok = tool.run(write=write)
        self.doc = (self.root / tr.DOC).read_text(encoding='utf-8')
        return tool, ok


class TraceabilityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.p = Project(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def kinds(self, tool):
        return {e.split(']')[0][1:] for e in tool.errors}

    def generate(self):
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)
        return tool

    # -- complete and consistent evidence
    def test_all_evidence_present_passes_and_output_is_deterministic(self):
        self.generate()
        first = (self.tmp / tr.DOC).read_bytes()
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)
        self.assertEqual(first, (self.tmp / tr.DOC).read_bytes())
        tool, ok = self.p.tool()
        self.assertTrue(ok, tool.errors)
        self.assertEqual(tool.results['REQ-01'].status, tr.VERIFIED)
        self.assertIn(f'BEGIN GENERATED: status {NOTE}', self.p.doc)

    def test_windows_line_endings_are_kept(self):
        self.generate()
        crlf = self.p.doc.replace('\n', '\r\n')
        (self.tmp / tr.DOC).write_bytes(crlf.encode('utf-8'))
        tool = tr.Traceability(self.tmp)
        self.assertTrue(tool.run(write=True), tool.errors)
        self.assertEqual((self.tmp / tr.DOC).read_bytes(), crlf.encode('utf-8'))

    def test_check_mode_writes_nothing_and_cli_exits_non_zero_on_mismatch(self):
        self.p.save()
        before = (self.tmp / tr.DOC).read_bytes()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(tr.main(['--check', '--root', str(self.tmp)]), 1)
            self.assertEqual(before, (self.tmp / tr.DOC).read_bytes())
            self.assertEqual(tr.main(['--write', '--root', str(self.tmp)]), 0)
            self.assertEqual(tr.main(['--check', '--root', str(self.tmp)]), 0)
        self.assertIn('[wrong-status]', out.getvalue())

    def test_requirement_with_code_and_tests_but_no_evidence_is_not_verified(self):
        tool = self.generate()
        self.assertEqual(tool.results['REQ-03'].status, tr.PARTIAL)
        self.assertIn('no evidence item meets its rule', tool.results['REQ-03'].reasons)
        self.p.registry['requirements']['REQ-03'] = {'tests': [], 'evidence': []}
        tool = self.generate()
        self.assertEqual(tool.results['REQ-03'].status, tr.NOT)

    # -- missing or stale evidence
    def test_missing_evidence_file(self):
        self.generate()
        (self.p.app / 'results/fault-injection/C1/summary.json').unlink()
        tool, ok = self.p.tool()
        self.assertFalse(ok)
        self.assertIn('missing-evidence', self.kinds(tool))
        self.assertNotEqual(tool.results['REQ-01'].status, tr.VERIFIED)

    def test_stale_evidence_path_in_registry_and_page(self):
        self.p.registry['requirements']['REQ-01']['evidence'].append(
            {'kind': 'fault_campaign', 'path': 'results/fault-injection/GONE'})
        self.p.doc = self.p.doc.replace('is open.', 'is open; see `results/fault-injection/20260101-000000`.')
        tool, ok = self.p.tool(write=True)
        self.assertFalse(ok)
        self.assertIn('missing-evidence', self.kinds(tool))
        self.assertTrue(any('stale-path' in e and '20260101-000000' in e for e in tool.errors), tool.errors)

    def test_missing_named_test(self):
        self.p.registry['requirements']['REQ-01']['tests'].append('tests/test_demo.py::test_removed')
        tool, ok = self.p.tool(write=True)
        self.assertIn('missing-test', self.kinds(tool))

    def test_unreferenced_result_folder(self):
        self.p.campaign('C2', passed=[True])
        tool, ok = self.p.tool(write=True)
        self.assertFalse(ok)
        self.assertTrue(any('unreferenced-result' in e and 'C2' in e for e in tool.errors))
        self.p.registry['excluded_results']['results/fault-injection/C2'] = 'Exploratory run, not evidence.'
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)

    # -- the page disagrees with the evidence
    def test_wrong_manual_count_in_generated_field(self):
        self.generate()
        self.p.doc = self.p.doc.replace('1 are VERIFIED', '2 are VERIFIED')
        tool, ok = self.p.tool()
        self.assertFalse(ok)
        self.assertIn('stale-generated', self.kinds(tool))

    def test_outdated_number_in_hand_written_text(self):
        self.generate()
        self.p.doc = self.p.doc.replace('Its uptime was 5 min', 'Its uptime was 9 min')
        tool, ok = self.p.tool()
        self.assertIn('outdated-value', self.kinds(tool))
        self.p.doc = self.p.doc.replace('Its uptime was 9 min', 'It ran soon after a restart')
        tool, ok = self.p.tool()
        self.assertIn('stale-claim', self.kinds(tool))

    def test_incorrect_status(self):
        self.generate()
        self.p.doc = self.p.doc.replace('| REQ-01 | Requirement REQ-01. | `tests/test_demo.py::test_a` `test_b` | evidence | VERIFIED |',
                                        '| REQ-01 | Requirement REQ-01. | `tests/test_demo.py::test_a` `test_b` | evidence | PARTIALLY VERIFIED |')
        tool, ok = self.p.tool()
        self.assertIn('wrong-status', self.kinds(tool))

    def test_status_stronger_than_evidence(self):
        self.generate()
        row = '| REQ-02 | Requirement REQ-02. | `tests/test_demo.py::test_a` `test_b` | evidence | PARTIALLY VERIFIED |'
        self.assertIn(row, self.p.doc)
        self.p.doc = self.p.doc.replace(row, row.replace('PARTIALLY VERIFIED', 'VERIFIED'))
        tool, ok = self.p.tool()
        self.assertIn('claim-stronger-than-evidence', self.kinds(tool))
        tool, ok = self.p.tool(write=True)  # --write restores the computed status
        self.assertTrue(ok, tool.errors)
        self.assertIn(row, self.p.doc)

    def test_missing_and_duplicated_requirement_ids(self):
        self.p.doc = doc_text(ids=('REQ-01', 'REQ-01', 'REQ-03'), hazard_ids=('REQ-01', 'REQ-02', 'REQ-03', 'REQ-04'))
        tool, ok = self.p.tool(write=True)
        errors = '\n'.join(tool.errors)
        self.assertIn('REQ-01 appears 2 times in the traceability tables', errors)
        self.assertIn('REQ-02 is in the registry but missing from the traceability tables', errors)
        self.assertIn('REQ-04 is in the hazard table but not in the registry', errors)

    def test_generated_markers_missing_or_hand_edited(self):
        self.generate()
        self.p.doc = self.p.doc.replace(f'<!-- BEGIN GENERATED: rule-runs {NOTE} -->', '')
        tool, ok = self.p.tool()
        self.assertIn('generated', self.kinds(tool))

    # -- consecutive fresh-restart rule
    def test_fewer_than_required_consecutive_passes(self):
        self.p.fresh_passes(4)
        tool = self.generate()
        self.assertFalse(tool.series['satisfied'])
        self.assertEqual(len(tool.series['literal']), 4)
        self.assertEqual(tool.results['REQ-02'].status, tr.PARTIAL)
        self.assertIn('4 of 5 so far', self.p.doc)

    def test_exactly_enough_consecutive_passes(self):
        self.p.fresh_passes(5)
        tool = self.generate()
        self.assertTrue(tool.series['satisfied'])
        self.assertEqual(tool.results['REQ-02'].status, tr.VERIFIED)
        self.assertIn('5 of 5 so far', self.p.doc)

    def test_fresh_failure_resets_and_non_fresh_runs_do_not_count(self):
        self.p.fresh_passes(3)
        self.p.run('20261003-150000', uptime=7200)          # not fresh: neither counts nor resets
        self.p.run('20261003-160000', failures=['control_misses 1'], uptime=300)  # fresh failure resets
        self.p.registry['requirements']['REQ-02']['gaps'][0]['covers'].append(
            {'path': 'results/acceptance-test/20261003-160000'})
        self.p.fresh_passes(2, start=70)
        tool = self.generate()
        self.assertEqual(tool.series['literal'], ['20261003-170000', '20261003-171000'])

    def test_ambiguous_reading_is_reported_and_does_not_upgrade(self):
        self.p.fresh_passes(4)
        self.p.run('20261003-150000', uptime=600, same_boot_as_previous=True)
        tool, ok = self.p.tool(write=True)
        self.assertEqual(len(tool.series['literal']), 5)
        self.assertEqual(len(tool.series['strict']), 4)
        self.assertFalse(tool.series['satisfied'])
        self.assertEqual(tool.results['REQ-02'].status, tr.PARTIAL)
        self.assertIn('ambiguous-rule', self.kinds(tool))
        self.assertTrue(any('same restart' in w for w in tool.warnings))
        self.p.registry['rules']['consecutive_fresh_acceptance']['interpretation'] = {'same_restart_counts': True}
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)
        self.assertEqual(tool.results['REQ-02'].status, tr.VERIFIED)

    def test_recorded_one_run_per_restart_reading(self):
        self.p.registry['rules']['consecutive_fresh_acceptance']['interpretation'] = {
            'same_restart_counts': False, 'decided': '2026-10-03', 'reason': 'procedure: restart before every run'}
        self.p.fresh_passes(4)
        self.p.run('20261003-150000', uptime=600, same_boot_as_previous=True)
        tool = self.generate()  # decided, so no error and no open question
        self.assertEqual(len(tool.series['literal']), 5)
        self.assertEqual(len(tool.series['strict']), 4)
        self.assertFalse(tool.series['satisfied'])
        self.assertEqual(tool.results['REQ-02'].status, tr.PARTIAL)
        self.assertFalse(any('ambiguous-rule' in w for w in tool.warnings))
        self.assertIn('4 of 5 so far', self.p.doc)
        self.assertIn('5 qualifying runs in total', self.p.doc)
        self.assertIn('at most one run per restart counts (procedure: restart before every run)', self.p.doc)
        self.p.run('20261003-160000', uptime=600)  # its own restart
        tool = self.generate()
        self.assertTrue(tool.series['satisfied'])
        self.assertEqual(tool.results['REQ-02'].status, tr.VERIFIED)

    def test_interpretation_values_are_validated(self):
        self.p.registry['rules']['consecutive_fresh_acceptance']['interpretation'] = {
            'same_restart_counts': 'no', 'one_per_day': True}
        tool, ok = self.p.tool(write=True)
        errors = '\n'.join(tool.errors)
        self.assertIn('same_restart_counts must be true or false', errors)
        self.assertIn("unknown interpretation key 'one_per_day'", errors)

    def test_non_fresh_failure_inside_the_series_blocks_until_interpreted(self):
        self.p.fresh_passes(2)
        self.p.run('20261003-130000', failures=['control_misses 1'], uptime=7200)
        self.p.registry['requirements']['REQ-02']['gaps'][0]['covers'].append(
            {'path': 'results/acceptance-test/20261003-130000'})
        self.p.fresh_passes(3, start=40)
        tool, ok = self.p.tool(write=True)
        self.assertFalse(tool.series['satisfied'])
        self.assertIn('ambiguous-rule', self.kinds(tool))

    # -- invalid or contradictory data
    def test_incomplete_run_is_invalid_data(self):
        self.p.run('20261003-110000', complete=False)
        tool, ok = self.p.tool(write=True)
        self.assertIn('invalid-run-data', self.kinds(tool))
        self.assertTrue(any('incomplete' in w for w in tool.warnings))

    def test_malformed_result_file(self):
        (self.p.app / 'results/fault-injection/C1/summary.json').write_text('{"scenarios": [', encoding='utf-8')
        tool, ok = self.p.tool(write=True)
        self.assertFalse(ok)
        self.assertIn('invalid-run-data', self.kinds(tool))

    def test_contradictory_acceptance_files(self):
        folder = self.p.run('20261003-110000')
        verdict = json.loads((folder / 'verdict.json').read_text())
        verdict['methods'][0]['failures'] = ['processing p95 160.0 ms > 150']  # but status stays PASS
        write_json(folder / 'verdict.json', verdict)
        tool, ok = self.p.tool(write=True)
        self.assertIn('contradictory-results', self.kinds(tool))

    def test_contradictory_fault_summary(self):
        self.p.campaign('C1', passed=[True, True, False], status=tr.VERIFIED)
        tool, ok = self.p.tool(write=True)
        self.assertIn('contradictory-results', self.kinds(tool))
        self.assertNotEqual(tool.results['REQ-01'].status, tr.VERIFIED)

    def test_failure_needs_a_documented_gap(self):
        self.p.campaign('C1', passed=[True, False])
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('no documented gap explains' in e for e in tool.errors))
        self.p.registry['requirements']['REQ-01']['gaps'] = [{
            'id': 'pre-fault', 'reason': 'stopped before the fault', 'covers': [
                {'path': 'results/fault-injection/C1', 'scenario': 'watchdog-stop'}]}]
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)
        self.assertEqual(tool.results['REQ-01'].status, tr.PARTIAL)

    def test_superseded_scenario_must_name_a_passing_replacement(self):
        self.p.campaign('C0', passed=[False] * 3)
        entry = {'kind': 'fault_campaign', 'path': 'results/fault-injection/C0',
                 'superseded': {'watchdog-stop': {'reason': 'old gate', 'replaced_by': ['results/fault-injection/C1']}}}
        self.p.registry['requirements']['REQ-01']['evidence'].insert(0, entry)
        tool, ok = self.p.tool(write=True)
        self.assertTrue(ok, tool.errors)
        self.assertEqual(tool.results['REQ-01'].status, tr.VERIFIED)
        self.p.campaign('C1', passed=[True, False])  # the replacement no longer passes
        self.p.registry['requirements']['REQ-01']['gaps'] = [{'id': 'g', 'reason': 'r', 'covers': [
            {'path': 'results/fault-injection/C1', 'scenario': 'watchdog-stop'}]}]
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('superseded by results/fault-injection/C1' in e for e in tool.errors), tool.errors)
        entry['superseded']['watchdog-stop'] = 'old gate'  # a bare reason is not enough
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('needs a reason and replaced_by' in e for e in tool.errors), tool.errors)

    def test_runs_in_the_series_cannot_be_excluded(self):
        self.p.registry['excluded_results'][f'results/acceptance-test/{FAIL_RUN}'] = 'inconvenient'
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('cannot be excluded' in e for e in tool.errors), tool.errors)
        self.assertIn(f'results/acceptance-test/{FAIL_RUN}', [i.path for i in tool.results['REQ-02'].items])

    def test_field_missing_in_one_list_element_is_invalid(self):
        write_json(self.p.app / 'results/latency/L1/verification.json',
                   {'methods': [{'misses': 0}, {'label': 'no record'}]})
        self.p.registry['requirements']['REQ-03']['evidence'] = [
            {'kind': 'fields', 'path': 'results/latency/L1',
             'expect': [{'file': 'verification.json', 'field': 'methods[*].misses', 'equals': 0}]}]
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('invalid-run-data' in e and 'methods[*].misses' in e for e in tool.errors), tool.errors)

    def test_field_references_and_absent_fields(self):
        write_json(self.p.app / 'results/study/S1/summary.json',
                   {'planned': 9, 'passed': 9, 'negative': {'target_not_found': 2}})
        self.p.registry['requirements']['REQ-03']['evidence'] = [
            {'kind': 'fields', 'path': 'results/study/S1', 'expect': [
                {'file': 'summary.json', 'field': 'passed', 'equals': '@planned'},
                {'file': 'summary.json', 'field': 'negative.converged', 'absent': True}]}]
        tool = self.generate()
        self.assertEqual(tool.results['REQ-03'].status, tr.VERIFIED)
        write_json(self.p.app / 'results/study/S1/summary.json',
                   {'planned': 9, 'passed': 8, 'negative': {'converged': 1}})
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('passed = 8' in e and 'must be absent' in e for e in tool.errors), tool.errors)

    def test_partial_coverage_caps_status(self):
        self.p.campaign('C1', passed=[True] * 3, coverage='partial')
        tool = self.generate()
        self.assertEqual(tool.results['REQ-01'].status, tr.PARTIAL)

    def test_declared_field_values_use_existing_limits(self):
        write_json(self.p.app / 'results/latency/L1/verification.json', {'lateness_ms': 51})
        self.p.registry['requirements']['REQ-03']['evidence'] = [
            {'kind': 'fields', 'path': 'results/latency/L1',
             'expect': [{'file': 'verification.json', 'field': 'lateness_ms', 'at_most': 50}]}]
        tool, ok = self.p.tool(write=True)
        self.assertTrue(any('lateness_ms = 51' in e for e in tool.errors), tool.errors)
        self.p.registry['requirements']['REQ-03']['evidence'][0]['expect'][0]['at_most'] = '$FRESH_RESTART_S'
        tool, ok = self.p.tool(write=True)
        self.assertEqual(tool.results['REQ-03'].status, tr.VERIFIED)

    @unittest.skipUnless(shutil.which('git'), 'git not installed')
    def test_git_ignored_evidence_is_reported(self):
        self.p.run('20261002-000000', smoke=True)
        self.p.registry['excluded_results']['results/acceptance-test/20261002-000000'] = 'smoke run'
        self.generate()
        # The generated text does not depend on git: an unpublished, excluded run changes nothing.
        (self.tmp / '.gitignore').write_text(f'{APP}/results/acceptance-test/20261002-000000/\n', encoding='utf-8')
        subprocess.run(['git', 'init', '-q'], cwd=self.tmp, check=True)
        tool, ok = self.p.tool()
        self.assertTrue(ok, tool.errors)
        (self.tmp / '.gitignore').write_text(f'{APP}/results/fault-injection/\n', encoding='utf-8')
        subprocess.run(['git', 'init', '-q'], cwd=self.tmp, check=True)
        tool, ok = self.p.tool()
        self.assertTrue(any('unpublished-evidence' in e and 'C1' in e for e in tool.errors), tool.errors)
        # A run in the acceptance series cannot be hidden by not publishing it.
        (self.tmp / '.gitignore').write_text(f'{APP}/results/acceptance-test/{FAIL_RUN}/\n', encoding='utf-8')
        tool, ok = self.p.tool()
        self.assertTrue(any('unpublished-evidence' in e and FAIL_RUN in e for e in tool.errors), tool.errors)


class RepositoryTests(unittest.TestCase):
    def test_registry_is_well_formed(self):
        registry = json.loads((tr.ROOT / tr.REGISTRY).read_text(encoding='utf-8'))
        for req, spec in registry['requirements'].items():
            self.assertRegex(req, tr.REQ_ID.pattern)
            for gap in spec.get('gaps', []):
                self.assertTrue(gap.get('reason'), f'{req} gap without a reason')
        for path, reason in registry['excluded_results'].items():
            self.assertTrue(reason.strip(), path)


if __name__ == '__main__':
    unittest.main()
