"""Check the planner against a photo where every strip's first and last channel is lit and each
place two strips meet is marked with a blue X in a yellow box.

  1. find the marks (blue X strokes inside yellow outlines)
  2. build the spiral in 3D: angle from the LED map, radius from the metal, depth from the funnel
  3. fit the phone's pose (solvePnP) and pair marks with predicted strip ends (Hungarian)
  4. write photo_check.json (read by build_planner_data.py) and photo_check.jpg (overlay)

Run after build_planner_data.py, then run the build once more so the page picks up the result:
  PY=/Users/stephanschulz/Documents/cursor_ai/pixel-mapper/.venv/bin/python
  $PY build_planner_data.py && $PY photo_check.py && $PY build_planner_data.py
"""
import json
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.optimize import linear_sum_assignment

HERE = os.path.dirname(os.path.abspath(__file__))
PHOTO = os.path.join(HERE, "..", "pixel mapping scans", "first and last channel on each LED strips.png")
DATA = os.path.join(HERE, "planner_data.json")

Z_IN, Z_OUT = 100.0, 370.0     # mm off the wall at r = 100 and r = 700 (fin tips)
F_35MM = 26.0                  # phone main camera, 35 mm-equivalent focal length
MATCH_PX = 60                  # a predicted end counts as marked within this many pixels


def find_marks(rgb):
    R, G, B = (rgb[..., i].astype(int) for i in range(3))
    yellow = ndimage.binary_dilation((R > 200) & (G > 190) & (B < 90), iterations=25)
    blue = (B > 150) & (R < 90) & (G < 120) & yellow
    lab, n = ndimage.label(ndimage.binary_dilation(blue, iterations=2))
    size = ndimage.sum(blue, lab, range(1, n + 1))
    cm = ndimage.center_of_mass(blue, lab, range(1, n + 1))
    frag = np.array([(c[1], c[0]) for c, s in zip(cm, size) if s > 80])
    cl = fcluster(linkage(frag, "single"), t=40, criterion="distance")   # X strokes -> one mark
    return np.array([frag[cl == k].mean(0) for k in np.unique(cl)])


def main():
    d = json.load(open(DATA))
    rgb = np.asarray(Image.open(PHOTO).convert("RGB"))
    H, W = rgb.shape[:2]
    marks = find_marks(rgb)

    th = np.array(d["led_th"])
    r = d["helixA"] + d["helixB"] * ((th - d["ribAngle"][0]) / 360 + d["Hc"])
    z = Z_IN + (np.clip(r, 100, 700) - 100) * (Z_OUT - Z_IN) / 600
    P = np.c_[r * np.cos(np.radians(th)), r * np.sin(np.radians(th)), z]

    # physical strip-end locations: chain start, every hand-off, the tip, and the off-chain ends
    S = d["strips"]
    locs = [dict(label=f"{S[0]['name']} in", g=[S[0]["g0"]], kind="start",
                 detail=f"{S[0]['ip'][-4:]} p{S[0]['port']} data in")]
    for i in range(1, len(S)):
        locs.append(dict(label=f"{S[i-1]['name']}→{S[i]['name']}", g=[S[i - 1]["g1"], S[i]["g0"]], kind="handoff",
                         detail=f"{S[i-1]['ip'][-4:]} p{S[i-1]['port']} → {S[i]['ip'][-4:]} p{S[i]['port']}"))
    locs.append(dict(label=f"{S[-1]['name']} tip", g=[S[-1]["g1"]], kind="tip", detail="end of the tube"))
    extra = []
    for o in d["outer"]:
        for k, w in ((0, "first"), (-1, "last detected")):
            t, rr = o["th"][k], o["r"][k]
            zz = Z_IN + (min(max(rr, 100), 700) - 100) * (Z_OUT - Z_IN) / 600
            extra.append(dict(label=f"{o['name']} {w}", kind="outer",
                              detail=f"{o['ip'][-4:]} p{o['port']}",
                              xyz=[rr * np.cos(np.radians(t)), rr * np.sin(np.radians(t)), zz]))
    X = np.array([P[l["g"]].mean(0) for l in locs] + [e["xyz"] for e in extra])
    locs += extra
    # strip ends that share one spot (e.g. two ports fed from the same point) are one location
    keep = []
    for i in range(len(locs)):
        j = next((k for k in keep if np.linalg.norm(X[k] - X[i]) < 30), None)
        if j is None:
            keep.append(i)
        else:
            locs[j] = dict(locs[j], label=locs[j]["label"] + " + " + locs[i]["label"], kind="start")
    X, locs = X[keep], [locs[k] for k in keep]

    f = F_35MM / 36.0 * max(W, H)
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    # start: plan view scaled onto the frame, centred where the marks are
    c0 = marks.mean(0)
    s0 = (marks[:, 0].max() - marks[:, 0].min()) / (X[:, 0].max() - X[:, 0].min())
    proj = np.c_[c0[0] + s0 * (X[:, 0] - X[:, 0].mean()), c0[1] - s0 * (X[:, 1] - X[:, 1].mean())]
    for _ in range(8):
        D = np.hypot(marks[:, None, 0] - proj[None, :, 0], marks[:, None, 1] - proj[None, :, 1])
        mi, li = linear_sum_assignment(D)
        _, rv, tv = cv2.solvePnP(X[li], marks[mi], K, None, flags=cv2.SOLVEPNP_ITERATIVE)
        proj = cv2.projectPoints(X, rv, tv, K, None)[0][:, 0]
    D = np.hypot(marks[:, None, 0] - proj[None, :, 0], marks[:, None, 1] - proj[None, :, 1])
    mi, li = linear_sum_assignment(D)
    pair = {int(l): int(m) for m, l in zip(mi, li) if D[m, l] < MATCH_PX}
    px_per_mm = np.median(np.hypot(*np.diff(cv2.projectPoints(np.ascontiguousarray(P[::50]), rv, tv, K, None)[0][:, 0], axis=0).T)
                          / np.hypot(*np.diff(P[::50, :2], axis=0).T))
    sc = 1200 / W                                   # the page's copy of the photo is 1200 px wide
    rows = []
    for i, l in enumerate(locs):
        m = pair.get(i)
        rows.append(dict(label=l["label"], kind=l["kind"], detail=l.get("detail", ""),
                         marked=m is not None, x=round(float(proj[i][0] * sc), 1),
                         y=round(float(proj[i][1] * sc), 1), mark=m,
                         err_mm=round(float(D[m, i] / px_per_mm), 1) if m is not None else None))
    used = set(pair.values())
    errs = [D[m, i] / px_per_mm for i, m in pair.items()]
    out = dict(photo=os.path.basename(PHOTO), marks=len(marks),
               matched=len(pair), unexplained=len(marks) - len(used),
               expected=sum(1 for l in locs if l["kind"] != "outer"),
               median_mm=round(float(np.median(errs)), 1), max_mm=round(float(np.max(errs)), 1),
               camera_mm=round(float(np.linalg.norm(tv))), rows=rows,
               view=[int(W * sc), int(H * sc)],
               mark_xy=[[round(float(a * sc), 1), round(float(b * sc), 1)] for a, b in marks])
    # every strip projected into the photo (every 3rd LED), and the tail: seen part + drawn hang
    led2d = cv2.projectPoints(P, rv, tv, K, None)[0][:, 0] * sc
    out["strips"] = [dict(name=s["name"], ip=s["ip"],
                          pts=[int(round(v)) for q in led2d[s["g0"]:s["g1"] + 1:3] for v in q]
                          + [int(round(v)) for v in led2d[s["g1"]]]) for s in S]
    for o in d["outer"]:
        xy = np.array(o["xy"] + o["hang"], dtype=float)
        rr = np.hypot(xy[:, 0], xy[:, 1])
        zz = np.where(np.arange(len(xy)) < len(o["xy"]), Z_IN + (np.clip(rr, 100, 700) - 100) * (Z_OUT - Z_IN) / 600, Z_OUT)
        q = cv2.projectPoints(np.ascontiguousarray(np.c_[xy, zz]), rv, tv, K, None)[0][:, 0] * sc
        out["strips"].append(dict(name=o["name"], ip=o["ip"], seen=len(o["xy"]) // 3 + 1,
                                  pts=[int(round(v)) for p_ in q[::3] for v in p_]))
    Image.fromarray(rgb).resize(tuple(out["view"])).save(os.path.join(HERE, "photo_check_base.jpg"), quality=80)
    json.dump(out, open(os.path.join(HERE, "photo_check.json"), "w"), indent=1)

    # overlay
    sc = 1600 / W
    im = Image.fromarray(rgb).resize((int(W * sc), int(H * sc)))
    g = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 15)
    except OSError:
        font = ImageFont.load_default()
    led = cv2.projectPoints(P, rv, tv, K, None)[0][:, 0] * sc
    col = {"101": (95, 160, 224), "102": (232, 150, 74), "103": (79, 191, 148), "104": (208, 124, 194)}
    for s in S:
        pts = [tuple(p) for p in led[s["g0"]:s["g1"] + 1:3]]
        g.line(pts, fill=col[s["ip"][-3:]], width=2)
    for m in marks * sc:
        g.rectangle([m[0] - 14, m[1] - 18, m[0] + 14, m[1] + 18], outline=(255, 230, 0), width=2)
    for i, (p, l) in enumerate(zip(proj * sc, locs)):
        ok = i in pair
        c = (60, 220, 120) if ok else (255, 80, 90)
        g.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], outline=c, width=3)
        g.text((p[0] + 10, p[1] - 22), l["label"], fill=c, font=font, stroke_width=3, stroke_fill=(0, 0, 0))
    im.save(os.path.join(HERE, "photo_check.jpg"), quality=88)

    print(f"marks {len(marks)}, matched {len(pair)}, unexplained {out['unexplained']}; "
          f"median {out['median_mm']} mm, max {out['max_mm']} mm; camera {out['camera_mm']} mm away")
    for r_ in rows:
        if not r_["marked"]:
            print("  not marked:", r_["label"])


if __name__ == "__main__":
    main()
