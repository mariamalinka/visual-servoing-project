"""Regenerate the figures in docs/figures/ for docs/PROJECT_REPORT.md.

Study counts are copied from the linked study pages; the acceptance figure reads
results/acceptance-test/<run>/verdict.json and manifest.json for every October run.
Needs matplotlib.  Usage: python -B tools/report_figures.py
"""
import json, math, os, sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = Path(__file__).resolve().parents[1]
OUT = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / 'docs' / 'figures')
RUNS = sys.argv[2] if len(sys.argv) > 2 else str(ROOT / 'outputs' / 'visual-servoing-simulation' / 'results' / 'acceptance-test')
os.makedirs(OUT, exist_ok=True)
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
BLUE, ORANGE, GOOD, BAD = '#2a78d6', '#eb6834', '#0ca30c', '#d03b3b'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8.5, 'axes.edgecolor': '#b9b8b2',
                     'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                     'axes.spines.top': False, 'axes.spines.right': False, 'svg.fonttype': 'none', 'svg.hashsalt': 'project-report'})


def _tail(k, n, p):
    """P(X >= k) for X ~ Binomial(n, p)."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))


def _solve(f, target):
    lo, hi = 0.0, 1.0
    for _ in range(80):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) < target else (lo, mid)
    return (lo + hi) / 2


def cp(k, n, alpha=0.05):
    """Clopper-Pearson interval in percent."""
    lo = 0.0 if k == 0 else _solve(lambda p: _tail(k, n, p), alpha / 2)
    hi = 1.0 if k == n else _solve(lambda p: _tail(k + 1, n, p), 1 - alpha / 2)
    return lo * 100, hi * 100


def save(fig, name):
    fig.savefig(f'{OUT}/{name}.svg', bbox_inches='tight', metadata={'Date': None})
    plt.close(fig)


# F1: runtime architecture -----------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 3.3))
ax.set_xlim(0, 100); ax.set_ylim(0, 46); ax.axis('off')

def box(x, y, w, h, title, lines, fc='#f3f7f9', ec='#9fb3bd'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.3,rounding_size=1.2', fc=fc, ec=ec, lw=0.9))
    ax.text(x + w / 2, y + h - 2.0, title, ha='center', va='top', fontsize=7.6, weight='bold', color=INK)
    ax.text(x + w / 2, y + h - 5.6, '\n'.join(lines), ha='center', va='top', fontsize=5.7, color=INK2, linespacing=1.45)

def arrow(x1, y1, x2, y2, label='', tx=None, ty=None, ha='center'):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1), arrowprops=dict(arrowstyle='-|>', color='#5f6b73', lw=0.9, mutation_scale=9))
    if label:
        ax.text(tx if tx is not None else (x1 + x2) / 2, ty if ty is not None else (y1 + y2) / 2 + 1.0, label,
                ha=ha, va='bottom', fontsize=5.9, color=INK2, linespacing=1.25)

W, H = 25, 18
X = (1, 37.5, 74)
box(X[0], 26, W, H, 'Control thread', ['2 ms loop; owns live physics', 'MuJoCo 500 Hz + actuator model', 'IBVS + precision stop gates', 'command lease (generation no.)'], fc='#eaf4f2', ec='#5fa89c')
box(X[1], 26, W, H, 'Sensor process', ['spawned; own OpenGL + model', 'renders copied state (30 Hz)', 'ArUco / SIFT / Learned GPU', 'work > 50 ms old is dropped'])
box(X[2], 26, W, H, 'Receiver thread', ['sole reader of the result pipe', 'forwards whole messages only', 'may block; control never does', 'bounded 0.5 s join at close'])
box(X[0], 1, W, H, 'Command path', ['zero on any trip (latched/held)', 'caps 0.35 / 0.6 rad/s', 'backstop 0.03 rad before limit', 'accel / decel / jerk limits'])
box(X[1], 1, W, H, 'Supervision, every tick', ['image age ≤ 400 ms (250 default)', 'loop gap / compute ≤ 50 ms', 'worker alive, clock monotonic', 'clearance + joint-limit guard'], fc='#fbf1ee', ec='#d88f78')
box(X[2], 1, W, H, 'Mailbox + transport', ['one item, latest wins', 'never blocks the producer', 'transport queue ≤ 8', 'optional added delay'])
arrow(26.6, 35, 37.1, 35, 'state copy\n(1 pending)', ty=36.0)
arrow(63.1, 35, 73.6, 35, 'result pipe', ty=36.0)
arrow(86.5, 25.6, 86.5, 19.4, 'whole\nmessage', tx=88.0, ty=20.6, ha='left')
arrow(73.6, 10, 63.1, 10, 'fresh\nobservation', ty=11.0)
arrow(37.1, 10, 26.6, 10, 'velocity\ncommand', ty=11.0)
arrow(13.5, 19.4, 13.5, 25.6, 'servo\nsetpoints', tx=15.0, ty=20.6, ha='left')
save(fig, 'fig1_architecture')

# F2: capability progression ---------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.35), gridspec_kw={'wspace': 0.95})
panels = [
    ('(a) Benchmark starts, 200 poses', [('Baseline IBVS', 34, 200), ('Detector fix', 86, 200), ('+ search & recovery', 200, 200)]),
    ('(b) Startup study, 200 scenes', [('Direct IBVS', 78, 200), ('Startup search', 196, 200), ('+ joint-limit retry', 198, 200), ('+ refined coverage', 200, 200)]),
]
for ax, (title, rows) in zip(axes, panels):
    ys = list(range(len(rows)))[::-1]
    for y, (name, k, n) in zip(ys, rows):
        r = 100 * k / n; lo, hi = cp(k, n)
        ax.barh(y, r, height=0.52, color=BLUE, alpha=0.9)
        ax.plot([lo, hi], [y, y], color=INK, lw=1.0)
        for v in (lo, hi):
            ax.plot([v, v], [y - 0.12, y + 0.12], color=INK, lw=1.0)
        ax.text(min(hi, 100) + 2, y, f'{k}/{n}', va='center', fontsize=7.5, color=INK)
    ax.set_yticks(ys, [r[0] for r in rows], fontsize=7.6)
    ax.set_xlim(0, 118); ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel('aligned, % (95% Clopper–Pearson)', fontsize=7.5)
    ax.set_title(title, fontsize=8.2, loc='left', color=INK)
    ax.grid(axis='x', color=GRID, lw=0.6); ax.set_axisbelow(True)
save(fig, 'fig2_progression')

# F3: acceptance run record ------------------------------------------------------
runs = sorted(d for d in os.listdir(RUNS) if d.startswith('2026100'))
up, status, p99 = [], [], {'SIFT': [], 'Learned GPU': []}
for d in runs:
    v = json.load(open(f'{RUNS}/{d}/verdict.json')); m = json.load(open(f'{RUNS}/{d}/manifest.json'))
    up.append(m['uptime_at_start_s'] / 60); status.append(v['status'])
    for x in v['methods']:
        p99[x['label']].append(x['capture_ms']['p99'])
labels = [d[4:8] + '\n' + d[9:13] for d in runs]
xs = range(len(runs))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 3.5), sharex=True, gridspec_kw={'hspace': 0.28, 'height_ratios': [1.15, 1]})
cols = ['#5fa89c' if u <= 60 else '#c9c8c2' for u in up]
a1.bar(xs, up, color=cols, width=0.6)
a1.set_yscale('log'); a1.set_ylim(1, 30000)
a1.axhline(60, color=INK2, lw=0.8, ls=(0, (3, 2)))
a1.text(len(runs) - 0.6, 66, 'fresh-restart limit, 60 min', fontsize=6.8, color=INK2, ha='right', va='bottom')
for x, u, s in zip(xs, up, status):
    a1.text(x, u * 1.35, ('PASS' if s == 'PASS' else 'FAIL'), ha='center', va='bottom', fontsize=6.6,
            color=GOOD if s == 'PASS' else BAD, weight='bold')
a1.set_ylabel('uptime at start, min', fontsize=7.5)
a1.set_title('(a) Acceptance runs on the fixed runtime: uptime at start and verdict', fontsize=8.2, loc='left', color=INK)
a1.grid(axis='y', color=GRID, lw=0.6); a1.set_axisbelow(True)
for name, c, mk in (('SIFT', BLUE, 'o'), ('Learned GPU', ORANGE, 's')):
    a2.plot(xs, p99[name], color=c, marker=mk, ms=4.5, lw=1.4, label=name)
a2.axhline(250, color=BAD, lw=0.9, ls=(0, (3, 2)))
a2.text(len(runs) - 0.6, 243, 'gate: p99 ≤ 250 ms', fontsize=6.8, color=INK2, ha='right', va='top')
a2.set_ylim(0, 270); a2.set_ylabel('capture→command p99, ms', fontsize=7.5)
a2.set_title('(b) Capture-to-command latency, 99th percentile per method', fontsize=8.2, loc='left', color=INK)
a2.legend(frameon=False, fontsize=7, loc='lower left', ncol=2)
a2.grid(axis='y', color=GRID, lw=0.6); a2.set_axisbelow(True)
a2.set_xticks(list(xs), labels, fontsize=6.6)
save(fig, 'fig3_acceptance')

# F4: stopping budget -------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 1.9))
rows = [('measured trips (default)', None), ('ideal actuator, worst case', 88), ('default model, worst case', 152)]
for y, (name, brake) in enumerate(rows):
    if brake is None:
        ax.barh(y, 474 - 442, left=442, color=INK2, height=0.32)
        ax.text(480, y, 'standstill at about 442–474 ms image age', va='center', fontsize=7, color=INK2)
        continue
    ax.barh(y, 400, color='#c9c8c2', height=0.55)
    ax.barh(y, 50, left=402, color='#86b6ef', height=0.55)
    ax.barh(y, brake, left=454, color=ORANGE, height=0.55)
    ax.text(200, y, 'trip at image-age limit: 400', ha='center', va='center', fontsize=6.8, color=INK)
    ax.text(427, y, '50', ha='center', va='center', fontsize=6.8, color=INK)
    ax.text(454 + brake / 2, y, f'brake {brake}', ha='center', va='center', fontsize=6.8, color='white')
    ax.text(454 + brake + 6, y, f'= {400 + 50 + brake} ms', va='center', fontsize=7.5, color=INK, weight='bold')
ax.set_yticks(range(len(rows)), [r[0] for r in rows], fontsize=7.4)
ax.set_ylim(-0.6, 2.5); ax.set_xlim(0, 760)
ax.set_xlabel('image age when the arm reaches standstill, ms (motion at 0.35 rad/s)', fontsize=7.5)
ax.grid(axis='x', color=GRID, lw=0.6); ax.set_axisbelow(True)
save(fig, 'fig4_stopping')
print(f'Figures written to {OUT}')
