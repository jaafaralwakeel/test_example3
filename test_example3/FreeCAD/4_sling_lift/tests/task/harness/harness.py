import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _repo_root(start):
    d = start
    for _ in range(8):
        if (d / "common" / "__init__.py").is_file():
            return d
        if d.parent == d:
            break
        d = d.parent
    return start


REPO = _repo_root(HERE)
sys.path[:0] = [str(REPO), "/opt"]

from common.harness_base import Harness, score_error, score_ratio

ALL_CRITERIA = {
    "executes and builds geometry": 0,
    "geometry matches ground truth": 1,
    "pantograph joints mated": 1,
    "mechanism articulates": 1,
    "carries dimensional constraints": 1,
    "dimensions drive the geometry": 1,
    "signature dimensions present": 1,
}

REF_SOLIDS = [
    (167.6, 396.1), (167.6, 396.1), (167.6, 396.1), (229.5, 542.2),
    (247.4, 604.8), (247.4, 604.8), (247.4, 604.8), (388.8, 918.9),
    (451.0, 633.6), (451.0, 633.6), (451.0, 633.6), (1132.7, 1108.3),
    (1433.0, 1381.1), (1433.0, 1381.1), (1433.0, 1381.1), (1433.0, 1381.1),
    (1875.5, 1241.9), (1875.5, 1241.9), (1875.5, 1241.9), (2633.1, 2214.4),
    (2633.1, 2214.4), (2715.4, 1625.0), (2715.4, 1625.0), (4599.0, 3486.6),
    (4599.0, 3486.6), (4955.4, 2716.9), (4955.4, 2716.9), (4964.4, 2400.5),
    (5347.1, 2299.2), (8419.5, 2398.9), (8419.5, 2398.9), (8419.5, 2398.9),
    (8419.5, 2398.9), (9522.1, 4392.7), (9522.1, 4392.7), (9522.1, 4392.7),
    (16369.8, 4194.5), (16369.8, 4194.5), (18293.9, 5123.3), (18293.9, 5123.3),
    (20844.6, 5995.1), (22100.5, 7571.7), (22100.5, 7571.7), (22100.5, 7571.7),
    (22100.5, 7571.7), (24551.5, 9478.0), (24551.5, 9478.0), (32834.7, 11144.6),
    (34699.4, 10366.7), (34699.4, 10366.7), (39792.9, 12020.8), (39792.9, 12020.8),
    (69259.6, 17314.9), (87862.8, 26297.4), (92614.3, 25816.4), (92614.3, 25816.4),
    (96659.9, 28968.6),
]
REF_MOVING = 9
REF_DRIVING = 132
REF_LENGTHS = [
    10.0, 10.0, 10.0, 11.0, 11.0, 11.0, 12.0, 12.0,
    12.2, 12.2, 12.5, 12.5, 12.5, 13.0, 14.21, 15.96,
    16.0, 16.0, 16.0, 16.0, 16.0, 16.0, 16.0, 16.2,
    16.8, 16.85, 17.0, 17.0, 17.0, 17.5, 18.0, 20.0,
    20.0, 20.0, 20.2, 20.5, 21.0, 22.0, 22.0, 22.0,
    22.0, 22.5, 24.0, 24.0, 24.2, 25.0, 25.0, 28.0,
    28.5, 30.0, 30.0, 31.5, 33.7, 34.0, 34.0, 34.0,
    35.0, 40.0, 40.5, 41.0, 42.0, 43.0, 43.0, 45.0,
    45.0, 45.0, 55.0, 57.3, 60.0, 60.0, 71.75, 81.0,
    90.0, 95.0, 97.5, 103.5, 108.5, 110.0, 119.7, 120.0,
    120.0, 190.0, 195.0, 306.0,
]
REF_ANGLES = [
    20.0, 20.0, 30.0, 30.0, 45.0, 45.0, 45.0, 45.0,
    45.0, 45.0, 45.0, 45.0, 60.0, 60.0, 60.0, 130.0,
]

PRESENT_TOL = 0.25
FIDELITY_PERFECT = 0.01
FIDELITY_ZERO = 0.25
COMPLETE_ZERO = 0.75
EXTRA_PERFECT = 0.005
EXTRA_ZERO = 0.10
PUSH_FULL = 0.5
PUSH_ZERO = 0.05
LIMIT_SLACK_MM = 1.0
DRIVING_FULL = 0.9
DRIVING_ZERO = 0.1
PROBE_MIN_TESTED = 4
SIGNATURE_FULL = 0.97
SIGNATURE_ZERO = 0.25


def _num(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def pair_solids(ref, cand, tol=PRESENT_TOL):
    edges = []
    for i, (rv, ra) in enumerate(ref):
        for j, (cv, ca) in enumerate(cand):
            e = max(abs(cv - rv) / rv, abs(ca - ra) / ra)
            if e <= tol:
                edges.append((e, i, j))
    edges.sort()
    used_i, used_j, pairs = set(), set(), {}
    for e, i, j in edges:
        if i in used_i or j in used_j:
            continue
        used_i.add(i)
        used_j.add(j)
        pairs[i] = (j, e)
    extras = [j for j in range(len(cand)) if j not in used_j]
    return pairs, extras


def geometry_score(solids):
    cand = [(_num(s[0]), _num(s[1])) for s in solids if len(s) >= 2 and _num(s[0]) > 0]
    pairs, extras = pair_solids(REF_SOLIDS, cand)
    n = len(REF_SOLIDS)
    total_v = sum(v for v, _ in REF_SOLIDS)
    fid = {i: score_error(e, FIDELITY_PERFECT, FIDELITY_ZERO) for i, (j, e) in pairs.items()}
    f_count = sum(fid.get(i, 0.0) for i in range(n)) / n
    f_vol = sum(fid.get(i, 0.0) * REF_SOLIDS[i][0] for i in range(n)) / total_v
    fidelity = 0.5 * f_count + 0.5 * f_vol
    completeness = score_ratio(len(pairs) / n, 1.0, COMPLETE_ZERO)
    extra_vol = sum(cand[j][0] for j in extras) / total_v
    extras_f = score_error(extra_vol, EXTRA_PERFECT, EXTRA_ZERO)
    score = completeness * fidelity * extras_f
    off = sorted(e for _, e in pairs.values() if e > FIDELITY_PERFECT)
    text = (f"{len(pairs)}/{n} reference solids found; fidelity {fidelity:.3f} "
            f"({len(off)} off by more than 1%, worst {off[-1]*100:.1f}%)" if off else
            f"{len(pairs)}/{n} reference solids found; fidelity {fidelity:.3f}")
    text += f"; {len(extras)} extra solids ({extra_vol*100:.2f}% of reference volume)"
    return score, text


def _asms(m):
    return [a for a in (m.get("assemblies") or []) if isinstance(a, dict) and "n_comps" in a]


def joints_score(m):
    asms = _asms(m)
    if not asms:
        return 0.0, "no assembly with joints found"
    moving = sum(int(a.get("moving_between_groups", 0)) for a in asms)
    structure = (min(moving, REF_MOVING) / max(moving, REF_MOVING)) if moving else 0.0
    ok = [a for a in asms if a.get("solve_rc") == 0]
    solves = len(ok) / len(asms)
    moved = sum(int(a.get("solve_moved", 0)) for a in ok)
    comps = sum(int(a.get("n_comps", 0)) for a in ok) or 1
    posed = 1.0 - min(1.0, moved / comps)
    score = structure * solves * posed
    text = (f"{moving} moving joints link different rigid groups (mechanism needs {REF_MOVING}); "
            f"{len(ok)}/{len(asms)} assemblies solve; solver shifts {moved} parts at the delivered pose")
    return score, text


def push_fraction(m):
    best, seen = 0.0, False
    for a in _asms(m):
        push = a.get("push")
        if not isinstance(push, dict):
            continue
        seen = True
        n_other = max(1, int(a.get("n_comps", 0)) - 1)
        for t in push.get("tests") or []:
            if len(t) == 5 and t[2] == 0 and _num(t[3]) < 0.5:
                best = max(best, _num(t[4]) / n_other)
    return best, seen


def slider_limit_score(m):
    worst, notes = 1.0, []
    for a in _asms(m):
        for s in a.get("sliders") or []:
            if "disp_z" not in s:
                continue
            pos = -_num(s["disp_z"])
            lo = _num(s.get("LengthMin")) if s.get("EnableLengthMin") else None
            hi = _num(s.get("LengthMax")) if s.get("EnableLengthMax") else None
            if lo is None and hi is None:
                continue
            out = 0.0
            if lo is not None and pos < lo:
                out = lo - pos
            if hi is not None and pos > hi:
                out = max(out, pos - hi)
            span = (hi - lo) if (lo is not None and hi is not None) else 50.0
            sc = score_error(out, LIMIT_SLACK_MM, max(span, 10.0))
            notes.append(f"slider at {pos:.1f} mm, limits {lo}..{hi}, outside by {out:.1f}")
            worst = min(worst, sc)
    return worst, notes


def articulate_score(m):
    frac, seen = push_fraction(m)
    resp = score_ratio(frac, PUSH_FULL, PUSH_ZERO) if seen else 0.0
    lim, notes = slider_limit_score(m)
    text = (f"a push on the slider makes {frac*100:.0f}% of the other parts follow"
            + ("; " + "; ".join(notes) if notes else "; slider limits disabled"))
    return resp * lim, text


def _take(pool, want, tol, twins):
    cands = [want] + ([want * 2, want / 2] if twins else [])
    for c in cands:
        for k, v in enumerate(pool):
            if abs(v - c) <= max(0.05, tol * c):
                return pool.pop(k)
    return None


def signature_score(dims):
    lens = sorted(_num(v) for v in (dims.get("length_values") or []))
    angs = sorted(_num(v) for v in (dims.get("angle_values") or []))
    hit = 0
    for v in sorted(REF_LENGTHS, reverse=True):
        if _take(lens, v, 0.005, True) is not None:
            hit += 1
    for v in REF_ANGLES:
        got = _take(angs, v, 0.005, False)
        if got is None:
            got = _take(angs, 180.0 - v, 0.005, False)
        if got is not None:
            hit += 1
    total = len(REF_LENGTHS) + len(REF_ANGLES)
    frac = hit / total
    return score_ratio(frac, SIGNATURE_FULL, SIGNATURE_ZERO), \
        f"{hit}/{total} of the reference's key dimensions (lengths of 10 mm+ and angles) are present"


class SlingLiftHarness(Harness):
    MUST_PASS = ("executes and builds geometry",)
    WEIGHTS = ALL_CRITERIA
    BUILD_TIMEOUT_S = 600

    def build_state(self, candidate_path):
        try:
            measurement, error = self.run_freecad_stage(candidate_path)
        except SystemExit as exc:
            measurement, error = None, str(exc)
        except Exception as exc:
            measurement, error = None, f"{type(exc).__name__}: {exc}"
        return {"candidate": candidate_path, "measurement": measurement or {}, "error": error}

    def checks(self, state):
        err, m = state["error"], state["measurement"]
        out = {}
        name = "executes and builds geometry"
        built = err is None and bool(m.get("solids"))
        out[name] = (name, 1.0 if built else 0.0,
                     f"opened, rebuilt, {m.get('n_solids')} solids" if built
                     else f"the measurement stage failed: {err or 'no solids'}")

        def run(name, fn, *args):
            if not built:
                out[name] = (name, 0.0, "nothing measured (the gate failed)")
                return
            try:
                score, text = fn(*args)
            except Exception as exc:
                score, text = 0.0, f"could not be scored: {type(exc).__name__}: {exc}"
            out[name] = (name, max(0.0, min(1.0, score)), text)

        dims = m.get("dims") if isinstance(m.get("dims"), dict) else {}
        probe = m.get("dim_probe") if isinstance(m.get("dim_probe"), dict) else {}

        run("geometry matches ground truth", lambda: geometry_score(m.get("solids") or []))
        run("pantograph joints mated", lambda: joints_score(m))
        run("mechanism articulates", lambda: articulate_score(m))

        def carries():
            n = int(dims.get("n_driving", 0))
            return (score_ratio(n / REF_DRIVING, DRIVING_FULL, DRIVING_ZERO),
                    f"{n} driving dimensions (reference has {REF_DRIVING}); full marks from "
                    f"{int(DRIVING_FULL * REF_DRIVING)}")

        def drives():
            tested, live = int(probe.get("tested", 0)), int(probe.get("live", 0))
            if not tested:
                return 0.0, "no driving dimension could be tested (dead geometry?)"
            return ((live / tested) * min(1.0, tested / PROBE_MIN_TESTED),
                    f"{live}/{tested} dimensions changed the geometry when raised 10%")

        run("carries dimensional constraints", carries)
        run("dimensions drive the geometry", drives)
        run("signature dimensions present", lambda: signature_score(dims))
        return out


main = SlingLiftHarness.as_main()

if __name__ == "__main__":
    SlingLiftHarness.cli()
