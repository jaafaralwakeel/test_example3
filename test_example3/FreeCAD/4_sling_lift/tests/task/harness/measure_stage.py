import math
import os
import sys
import time

sys.path.insert(0, os.environ["FC_STAGE_COMMON"])

from common.freecad_stage import Stage
import FreeCAD

LEN_TYPES = ("Distance", "DistanceX", "DistanceY", "Radius", "Diameter")
TIME_BUDGET_S = 150

def vol(o):
    try:
        return float(o.Shape.Volume)
    except Exception:
        return 0.0

def rname(v):
    try:
        return v[0].Name
    except Exception:
        return "?"

def comps_of(a):
    return [c for c in a.Group if c.TypeId in ("App::Link", "Assembly::AssemblyLink")]

def joints_of(a):
    js = []
    for c in a.Group:
        if c.TypeId == "Assembly::JointGroup":
            js += [j for j in c.Group if "JointType" in j.PropertiesList]
    return js

def snap(cs):
    return {c.Name: FreeCAD.Placement(c.Placement) for c in cs}

def restore(cs, s):
    for c in cs:
        c.Placement = FreeCAD.Placement(s[c.Name])

def shift(a, b):
    return (a.Base - b.Base).Length

def guard(fn, *args):
    try:
        return fn(*args)
    except Exception as e:
        return {"error": repr(e)[:200]}

def groups(js):
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    redundant = 0
    for j in js:
        if j.JointType == "Fixed":
            a = find(rname(j.Reference1))
            b = find(rname(j.Reference2))
            if a == b:
                redundant += 1
            else:
                parent[a] = b
    moving = [j for j in js if j.JointType != "Fixed"]
    between = 0
    roots = set()
    for j in moving:
        a = find(rname(j.Reference1))
        b = find(rname(j.Reference2))
        roots.add(a)
        roots.add(b)
        if a != b:
            between += 1
    for x in list(parent):
        roots.add(find(x))
    return {"rigid_groups": len(roots), "moving_joints": len(moving),
            "moving_between_groups": between, "fixed_inside_group": redundant}

def solid_sigs(shape):
    out = [[round(float(s.Volume), 1), round(float(s.Area), 1)] for s in shape.Solids]
    out.sort()
    return out

def push_test(doc, a, js, cs, s0):
    moving = [j for j in js if j.JointType != "Fixed"]
    if not moving:
        return None
    sliders = [j for j in moving if j.JointType == "Slider"]
    j = (sliders or moving)[0]
    ends = []
    for n in (rname(j.Reference1), rname(j.Reference2)):
        if n in s0 and n not in ends:
            ends.append(n)
    axes = (FreeCAD.Vector(1, 0, 0), FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(0, 0, 1))
    tests = []
    best = 0
    for n in ends:
        o = doc.getObject(n)
        for k, axis in enumerate(axes):
            restore(cs, s0)
            pl = FreeCAD.Placement(s0[n])
            pl.Base = pl.Base + axis * 10
            o.Placement = pl
            try:
                rc = a.solve()
            except Exception as e:
                rc = "ERR " + repr(e)[:60]
            kept = (o.Placement.Base - s0[n].Base).dot(axis) / 10.0
            others = sum(1 for c in cs if c.Name != n and shift(c.Placement, s0[c.Name]) > 0.05)
            tests.append([n, "xyz"[k], rc, round(kept, 2), others])
            if rc == 0:
                best = max(best, others)
    restore(cs, s0)
    return {"best_others_moved": best, "tests": tests}

def slider_info(js):
    out = []
    for j in js:
        if j.JointType != "Slider":
            continue
        d = {}
        for k in ("EnableLengthMin", "LengthMin", "EnableLengthMax", "LengthMax"):
            if k in j.PropertiesList:
                v = getattr(j, k)
                d[k] = v if isinstance(v, bool) else round(float(v.Value), 2)
        try:
            o1 = j.Reference1[0]
            o2 = j.Reference2[0]
            g1 = FreeCAD.Placement(o1.Placement) * FreeCAD.Placement(j.Placement1)
            g2 = FreeCAD.Placement(o2.Placement) * FreeCAD.Placement(j.Placement2)
            rel = g1.inverse() * g2
            d["disp_z"] = round(rel.Base.z, 2)
            d["off_axis"] = round((rel.Base.x ** 2 + rel.Base.y ** 2) ** 0.5, 2)
        except Exception as e:
            d["disp_error"] = repr(e)[:80]
        out.append(d)
    return out

def assembly_report(doc, a):
    js = joints_of(a)
    cs = comps_of(a)
    kinds = {}
    for j in js:
        kinds[j.JointType] = kinds.get(j.JointType, 0) + 1
    rep = {"name": a.Name, "label": a.Label, "volume": round(vol(a), 1),
           "n_comps": len(cs), "n_joints": len(js), "joint_types": kinds}
    rep.update(groups(js))
    s0 = snap(cs)
    try:
        rc = a.solve()
    except Exception as e:
        rc = "ERR " + repr(e)[:80]
    moved = 0
    for c in cs:
        if shift(c.Placement, s0[c.Name]) > 0.01:
            moved += 1
    restore(cs, s0)
    rep["solve_rc"] = rc
    rep["solve_moved"] = moved
    rep["push"] = guard(push_test, doc, a, js, cs, s0)
    rep["sliders"] = guard(slider_info, js)
    return rep

def dims_report(doc):
    lens = []
    angs = []
    n_sk = 0
    n_dim = 0
    for o in doc.Objects:
        if o.TypeId != "Sketcher::SketchObject":
            continue
        n_sk += 1
        for c in o.Constraints:
            if c.Type in LEN_TYPES or c.Type == "Angle":
                n_dim += 1
                if c.Driving:
                    if c.Type == "Angle":
                        angs.append(round(abs(math.degrees(float(c.Value))), 2))
                    else:
                        lens.append(round(abs(float(c.Value)), 2))
    lens.sort()
    angs.sort()
    return {"n_sketches": n_sk, "n_dimensional": n_dim, "n_driving": len(lens) + len(angs),
            "length_values": lens, "angle_values": angs}

def part_fingerprint(doc):
    v = a = cx = cy = cz = 0.0
    for o in doc.Objects:
        if o.TypeId == "App::Part":
            try:
                s = o.Shape
                v += s.Volume
                a += s.Area
                c = s.CenterOfMass
                cx += c.x
                cy += c.y
                cz += c.z
            except Exception:
                pass
    return (v, a, cx, cy, cz)

def dim_probe(doc, budget_s):
    t0 = time.time()
    cand = []
    for sk in doc.Objects:
        if sk.TypeId != "Sketcher::SketchObject":
            continue
        for i, c in enumerate(sk.Constraints):
            if c.Type in LEN_TYPES and c.Driving:
                cand.append((sk, i))
                break
    step = max(1, len(cand) // 8)
    picks = cand[::step][:8]
    base = part_fingerprint(doc)
    tested = 0
    live = 0
    for sk, i in picks:
        if time.time() - t0 > budget_s:
            break
        old = float(sk.Constraints[i].Value)
        try:
            sk.setDatum(i, FreeCAD.Units.Quantity(old * 1.1, "mm"))
            doc.recompute()
            fp = part_fingerprint(doc)
            tested += 1
            if (abs(fp[0] - base[0]) > 0.05 or abs(fp[1] - base[1]) > 0.5
                    or max(abs(fp[k] - base[k]) for k in (2, 3, 4)) > 0.01):
                live += 1
        except Exception:
            pass
        finally:
            try:
                sk.setDatum(i, FreeCAD.Units.Quantity(old, "mm"))
                doc.recompute()
            except Exception:
                pass
    return {"tested": tested, "live": live, "available": len(cand)}

class SlingLiftStage(Stage):
    def measure(self, doc):
        asms = sorted([o for o in doc.Objects if o.TypeId == "Assembly::AssemblyObject"],
                      key=vol, reverse=True)
        out = {}
        if asms:
            solids = solid_sigs(asms[0].Shape)
            out["geometry_source"] = "assembly"
        else:
            solids = []
            for o in doc.Objects:
                if o.TypeId == "App::Part":
                    solids += solid_sigs(o.Shape)
            solids.sort()
            out["geometry_source"] = "parts (no assembly object)"
        if not solids:
            raise RuntimeError("the document produced no solids")
        out["n_solids"] = len(solids)
        out["total_volume"] = round(sum(s[0] for s in solids), 1)
        out["solids"] = solids
        out["assemblies"] = [guard(assembly_report, doc, a) for a in asms]
        out["dims"] = guard(dims_report, doc)
        out["dim_probe"] = guard(dim_probe, doc, TIME_BUDGET_S)
        return out

SlingLiftStage.run()
