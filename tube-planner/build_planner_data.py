"""Put the pixel-mapper LED map onto the metal ribs, and write the result into the planner page.

Input
  scan/<name>_evenly_spaced.csv   index,dmx,universe,node,ip,strand_id,u,v  (pixel-mapper export)
  metal_ribs.json                 15 rib angles + numbers + measured tooth radii (from the B-rep)

Output
  planner_data.json               everything the page draws
  tube_junction_planner.html      the block between `/*DATA:BEGIN*/` and `/*DATA:END*/` is replaced
  strips.csv, junctions.csv       the same, as tables

What is measured and what is assumed
  * DIRECTION is measured: dmx 1 of every strip is where the camera saw that strip's first LED
    light up. Every strip starts at its outer end, so data runs outer -> inner.
  * CHAIN ORDER is measured: strip k's last LED lies next to strip k+1's first LED (checked
    below, and the check is written into the output).
  * ANGLE is measured, per LED, about a centre fitted one turn at a time (the centre drifts
    ~0.006 of the frame from rim to middle, because the funnel's middle is farther away).
  * Which RIB is which comes from two things you saw on the wall (ANCHORS). The camera frame
    turns out to be the metal's frame, unmirrored, to within ~1 degree.
  * RADIUS is taken from the metal (tooth helix), not from the camera: the camera's radial scale
    is warped by perspective. The LEDs-per-turn radius is kept as a cross-check.

Run with the pixel-mapper venv (numpy):
  /Users/stephanschulz/Documents/cursor_ai/pixel-mapper/.venv/bin/python build_planner_data.py
"""
import csv
import json
import math
import os
import re
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys
SCANS = os.path.join(HERE, "..", "pixel mapping scans")
SCAN_NAME = "led_map_2d_20260918-081257"
# how the LEDs the camera missed are filled in: "evenly_spaced" or "interpolated". Both pass
# exactly through every detected LED; on this scan they differ by at most 1/3 of an LED.
FILL = sys.argv[1] if len(sys.argv) > 1 else "evenly_spaced"
SCAN = os.path.join(SCANS, f"{SCAN_NAME}_{FILL}.csv")
META = os.path.join(HERE, "scan", f"{SCAN_NAME}.meta.json")
PHOTO_CHECK = os.path.join(HERE, "photo_check.json")
METAL = os.path.join(HERE, "metal_ribs.json")
PAGE = os.path.join(HERE, "tube_junction_planner.html")

LED_PER_M = 120.0              # 6x12 neon, WS2815, 120 px/m
PITCH = 1000.0 / LED_PER_M     # mm of tube per LED
ROLL = 300                     # LEDs in one 2.5 m roll
PORT = 600                     # LEDs on one node port = two rolls
JOINT = 24.0                   # mm of tube a joint + its flat cable occupies
RIB_T = 3.5                    # mm, rib tooth thickness
SIGMA_DEG = 1.0                # angle we trust the map to; within r*SIGMA of the line = marginal

# Things seen on the wall. Each pins one LED to one rib and tooth. Written against the node
# IP and port, because pixel-mapper's strand_id numbering changes between exports.
ANCHORS = [
    dict(ip="192.168.0.103", port=8, led=0, rib=7, tooth=29,
         note="cable 18 (tube 9-10 joint) sits on rib 07, tooth 29"),
    dict(ip="192.168.0.101", port=7, led=300, rib=4, tooth=34,
         note="cable 23 (inside tube 12) sits on rib 04, tooth 34"),
]
# The other four sightings from the v1 planner. Not used for the fit - only to check it.
CHECKS = [
    dict(tooth=3, between=[9, 10], label="a joint at tooth 3 between ribs 09/10"),
    dict(tooth=6, on=5, label="a joint on rib 05, tooth 6 (clashes)"),
    dict(tooth=12, between=[8, 9], label="a joint at tooth 12 between ribs 08/09"),
    dict(tooth=14, between=[4, 5], label="a joint at tooth 14 between ribs 04/05"),
]
# Strips off the chain whose real length is known (the camera saw only part). The outer tail:
# one 2.5 m roll, fed with strip 1 between ribs 02 and 03. It runs along the rim for the LEDs the
# camera saw, then hangs free. The hang is drawn, not measured.
OFF_CHAIN_LEDS = {("192.168.0.104", 5): ROLL}
HANG_BEND_MM = 120.0           # radius it bends through, from the rim tangent to straight down

# Four controllers, each at the tips of a pair of ribs; ID 1 at ribs 13+14, then clockwise.
CONTROLLERS = [[13, 14], [10, 11], [5, 6], [2, 3]]


# The label on each strip's cable, as printed on the sticker of each controller's lid. The lid lists
# the strips in the order of Art-Net ports 4, 1, 6, 2, 7, 8, 5, 3 (this order puts both "empty" slots on
# the two ports that never lit up, and matches the scan on 30 of 32 slots).
LID_LABELS = {
    "192.168.0.101": "23 7 2 13 12 21 empty 24",
    "192.168.0.102": "20 6 26 14 16 17 22 27",
    "192.168.0.103": "8 1 4 11 15 9 A 25",
    "192.168.0.104": "19 Y 18 3 5 10 Z empty",
}
LID_PORT_ORDER = [4, 1, 6, 2, 7, 8, 5, 3]


def cable_label(ip, port):
    lid = LID_LABELS.get(ip, "").split()
    for slot, p in enumerate(LID_PORT_ORDER):
        if p == port and slot < len(lid):
            return lid[slot], slot + 1
    return None, None


def strip_name(order, n_chain):
    """Strips are named from the centre: the innermost is 1. The short outermost piece of the
    chain is Y and the hanging outer tail is Z."""
    return "Y" if order == 0 else str(n_chain - order)


def load_scan(path):
    rows = list(csv.DictReader(open(path)))
    strands = defaultdict(list)
    for r in rows:
        strands[int(r["strand_id"])].append(r)
    for s in strands.values():
        s.sort(key=lambda r: int(r["index"]))
    return strands


def fit_circle(x, y):
    A = np.vstack([x, y, np.ones(len(x))]).T
    c = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    return cx, cy, math.sqrt(c[2] + cx * cx + cy * cy)


def chain_order(strands, max_gap=0.02):
    """Follow end -> start links. The chain starts at the strand whose first LED no other strand
    ends next to, and the longest such chain wins. Returns (chain, links, leftovers)."""
    ends = {k: (np.array([float(v[-1]["u"]), float(v[-1]["v"])]),
                np.array([float(v[0]["u"]), float(v[0]["v"])])) for k, v in strands.items()}
    heads = [k for k in strands
             if min((np.hypot(*(ends[j][0] - ends[k][1])) for j in strands if j != k),
                    default=1) > max_gap]
    best = None
    for h in heads:
        c = walk(h, ends, max_gap)
        if best is None or len(c[0]) > len(best[0]):
            best = c
    chain, links = best
    rest = [k for k in sorted(strands) if k not in chain]
    return chain, links, rest


def walk(head, ends, max_gap):
    chain, links = [head], []
    while True:
        k = chain[-1]
        best, bd = None, 1e9
        for j, (_, start) in ends.items():
            if j in chain:
                continue
            d = float(np.hypot(*(ends[k][0] - start)))
            if d < bd:
                best, bd = j, d
        if best is None or bd > max_gap:
            break
        links.append(dict(frm=k, to=best, gap_uv=round(bd, 5)))
        chain.append(best)
    return chain, links


def load_photo():
    """photo_check.json plus the plain photo it was fitted on, inlined so the page stays one file."""
    if not os.path.exists(PHOTO_CHECK):
        return None
    ph = json.load(open(PHOTO_CHECK))
    img = os.path.join(HERE, "photo_check_base.jpg")
    if os.path.exists(img):
        import base64
        ph["img"] = "data:image/jpeg;base64," + base64.b64encode(open(img, "rb").read()).decode()
    return ph


def main():
    metal = json.load(open(METAL))
    rib_ang = np.array(metal["ribAngle"])
    rib_no = metal["ribNo"]
    A, B = metal["helixA"], metal["helixB"]
    strands = load_scan(SCAN)
    chain, links, rest = chain_order(strands)

    rows = [r for k in chain for r in strands[k]]
    U = np.array([float(r["u"]) for r in rows])
    V = np.array([float(r["v"]) for r in rows])
    n = len(U)
    sid = np.array([int(r["strand_id"]) for r in rows])
    led = np.concatenate([np.arange(len(strands[k])) for k in chain])

    # --- centre, one turn at a time -------------------------------------------------------
    c0 = np.array([U.mean(), V.mean()])
    wins, i = [], 0
    while i < n - 40:
        t = np.unwrap(np.arctan2(-(V[i:] - c0[1]), U[i:] - c0[0]))
        j = int(np.argmax(np.abs(t - t[0]) >= 2 * np.pi))
        if j == 0:
            break
        cx, cy, _ = fit_circle(U[i:i + j], V[i:i + j])
        wins.append((i + j / 2, cx, cy))
        i += max(j // 2, 1)
    wins = np.array(wins)
    CX = np.interp(np.arange(n), wins[:, 0], wins[:, 1])
    CY = np.interp(np.arange(n), wins[:, 0], wins[:, 2])
    th_cam = np.degrees(np.unwrap(np.arctan2(-(V - CY), U - CX)))   # y up = seen from the front

    # --- which rib is which: rotation from the anchors ------------------------------------
    start_of = {}
    acc = 0
    for k in chain:
        start_of[k] = acc
        acc += len(strands[k])
    by_port = {(strands[k][0]["ip"], int(strands[k][0]["universe"]) // 2 + 1): k for k in chain}
    for a in ANCHORS:
        a["strand"] = by_port[(a["ip"], a["port"])]
    diffs = []
    for a in ANCHORS:
        g = start_of[a["strand"]] + a["led"]
        want = rib_ang[rib_no.index(a["rib"])]
        diffs.append(((want - th_cam[g] + 180) % 360) - 180)
    delta = math.degrees(math.atan2(np.mean(np.sin(np.radians(diffs))),
                                    np.mean(np.cos(np.radians(diffs)))))
    th = th_cam + delta                                               # metal frame, unwrapped

    # --- helix coordinate: H = n + p/15 at rib p, tooth n (0-based) -----------------------
    anchors_H = []
    for a in ANCHORS:
        g = start_of[a["strand"]] + a["led"]
        p = rib_no.index(a["rib"])
        anchors_H.append((a["tooth"] - 1 + p / 15) - (th[g] - rib_ang[0]) / 360)
    Hc = float(np.mean(anchors_H))
    H = (th - rib_ang[0]) / 360 + Hc
    r_helix = A + B * H

    # radius from the tube itself: LEDs in the surrounding turn x pitch / 2pi
    r_leds = np.full(n, np.nan)
    neg = -th
    for g in range(n):
        lo = np.searchsorted(neg, -(th[g] + 180))
        hi = np.searchsorted(neg, -(th[g] - 180))
        if lo > 0 and hi < n:
            r_leds[g] = (hi - lo) * PITCH / (2 * math.pi)

    # --- rib crossings, in LED coordinates ------------------------------------------------
    cross = []
    kk = (th - rib_ang[0]) / 24.0
    for g in range(n - 1):
        a, b = kk[g], kk[g + 1]
        for m in range(math.ceil(min(a, b)), math.floor(max(a, b)) + 1):
            f = (m - a) / (b - a) if b != a else 0
            p = m % 15
            h = Hc + m / 15.0
            tooth0 = round(h - p / 15)
            cross.append(dict(g=g + f, rib=rib_no[p], p=p, tooth=tooth0 + 1,
                              r=A + B * h, th=float(rib_ang[p])))
    cross.sort(key=lambda c: c["g"])
    cg = np.array([c["g"] for c in cross])

    def near_cross(gpos):
        j = int(np.searchsorted(cg, gpos))
        cand = [c for c in (cross[j - 1] if j > 0 else None, cross[j] if j < len(cross) else None) if c]
        return min(cand, key=lambda c: abs(c["g"] - gpos))

    def between(gpos):
        j = int(np.searchsorted(cg, gpos))
        a = cross[j - 1] if j > 0 else None
        b = cross[j] if j < len(cross) else None
        return a, b

    half = (JOINT + RIB_T) / 2

    def verdict(clear, r):
        unc = r * math.radians(SIGMA_DEG)
        return "clash" if clear < -unc else "marginal" if clear < unc else "clear"

    def junction(gpos, kind, **kw):
        c = near_cross(gpos)
        a, b = between(gpos)
        clear = abs(c["g"] - gpos) * PITCH - half
        f = gpos - math.floor(gpos)
        g0 = int(min(max(math.floor(gpos), 0), n - 1))
        g1 = min(g0 + 1, n - 1)
        thj = th[g0] + f * (th[g1] - th[g0])
        return dict(kind=kind, g=round(gpos, 2), th=round(float(thj % 360), 2),
                    r=round(float(A + B * ((thj - rib_ang[0]) / 360 + Hc)), 1),
                    rib=c["rib"], tooth=c["tooth"], clear=round(clear, 1),
                    verdict=verdict(clear, float(A + B * ((thj - rib_ang[0]) / 360 + Hc))),
                    ribs=[a["rib"] if a else None, b["rib"] if b else None],
                    from_tip_m=round((n - gpos - 0.5) * PITCH / 1000, 3), **kw)

    # --- strips ---------------------------------------------------------------------------
    strips, juncs = [], []
    for order, k in enumerate(chain):
        s0 = start_of[k]
        L = len(strands[k])
        r0 = strands[k][0]
        uni = int(r0["universe"])
        ip = r0["ip"]
        cs = [c for c in cross if s0 <= c["g"] < s0 + L]
        info = dict(
            order=order, pos=strip_name(order, len(chain)), strand=k, node=int(r0["node"]), ip=ip,
            port=uni // 2 + 1, universes=[uni, uni + 1] if L > 512 else [uni],
            dmx_first=int(r0["dmx"]), dmx_last=int(strands[k][-1]["dmx"]),
            index_first=int(r0["index"]), index_last=int(strands[k][-1]["index"]),
            leds=L, g0=s0, g1=s0 + L - 1,
            th_start=round(float(th[s0] % 360), 1), th_end=round(float(th[s0 + L - 1] % 360), 1),
            r_start=round(float(r_helix[s0]), 1), r_end=round(float(r_helix[s0 + L - 1]), 1),
            turns=round(float((th[s0] - th[s0 + L - 1]) / 360), 3),
            crossings=[[c["rib"], c["tooth"], round(c["g"] - s0, 1)] for c in cs],
        )
        strips.append(info)
        # port feed at this strip's first LED (the joint to the previous strip's last LED)
        juncs.append(junction(s0 - 0.5 if order else s0 - 0.5, "feed", strand=k, order=order,
                              ip=ip, port=uni // 2 + 1))
        if L > ROLL:
            juncs.append(junction(s0 + ROLL - 0.5, "splice", strand=k, order=order,
                                  ip=ip, port=uni // 2 + 1))
    # the tip, where the last roll ends
    juncs.append(junction(n - 0.5, "tip", strand=chain[-1], order=len(chain) - 1,
                          ip=strips[-1]["ip"], port=strips[-1]["port"]))
    for i, j in enumerate(juncs):
        j["id"] = i + 1

    # --- check the four sightings that were not used for the fit ---------------------------
    checks = []
    for c in CHECKS:
        hits = []
        for j in juncs:
            if j["kind"] == "tip":
                continue
            if "on" in c:
                if j["rib"] == c["on"] and j["tooth"] == c["tooth"] and j["clear"] < 0:
                    hits.append(j)
            else:
                pair = sorted(x for x in j["ribs"] if x is not None)
                if pair == sorted(c["between"]) and j["tooth"] == c["tooth"] and j["clear"] >= 0:
                    hits.append(j)
        checks.append(dict(label=c["label"], ok=bool(hits),
                           junction=hits[0]["id"] if hits else None))
    anchor_rows = []
    for a in ANCHORS:
        g = start_of[a["strand"]] + a["led"]
        c = near_cross(g - 0.5 if a["led"] in (0, ROLL) else g)
        anchor_rows.append(dict(label=a["note"], rib=c["rib"], tooth=c["tooth"],
                                ok=(c["rib"] == a["rib"] and c["tooth"] == a["tooth"]),
                                off_mm=round(abs(c["g"] - g + 0.5) * PITCH, 1)))

    # --- strands the camera saw but that are not in the chain ------------------------------
    outer = []
    rho_chain = np.hypot(U - CX, V - CY)
    ok = ~np.isnan(r_leds)
    k_scale = float(np.median(r_helix[ok] / rho_chain[ok][:]) if ok.any() else 2600)
    # rho -> mm, fitted on the outer turns only (these strands are outside them)
    outer_sel = np.arange(n) < 1500
    coef = np.polyfit(rho_chain[outer_sel], r_helix[outer_sel], 1)
    for k in rest:
        pts = strands[k]
        u = np.array([float(r["u"]) for r in pts]); v = np.array([float(r["v"]) for r in pts])
        t = np.degrees(np.arctan2(-(v - CY[0]), u - CX[0])) + delta
        rr = np.polyval(coef, np.hypot(u - CX[0], v - CY[0]))
        ip, port = pts[0]["ip"], int(pts[0]["universe"]) // 2 + 1
        xy = np.c_[rr * np.cos(np.radians(t)), rr * np.sin(np.radians(t))]
        total = OFF_CHAIN_LEDS.get((ip, port), len(pts))
        hang = []
        if total > len(pts):
            # carry on from the last seen LED along its tangent, bend down, then hang straight
            d = xy[-1] - xy[-4]
            d /= np.linalg.norm(d)
            p = xy[-1].copy()
            turn = PITCH / HANG_BEND_MM
            for _ in range(total - len(pts)):
                a = math.atan2(d[1], d[0])
                err = ((-math.pi / 2 - a + math.pi) % (2 * math.pi)) - math.pi
                a += max(-turn, min(turn, err))
                d = np.array([math.cos(a), math.sin(a)])
                p = p + d * PITCH
                hang.append([round(float(p[0]), 1), round(float(p[1]), 1)])
        # ribs it passes along the rim (it sits past the teeth, on the rib tips)
        passed = []
        for a0, a1 in zip(t[:-1], t[1:]):
            for i_, ra in enumerate(rib_ang):
                if (ra - a0) % 360 < (a1 - a0) % 360 < 180 or (a0 - ra) % 360 < (a0 - a1) % 360 < 180:
                    passed.append(rib_no[i_])
        f0 = t[0] % 360
        near = sorted(range(15), key=lambda i_: abs((rib_ang[i_] - f0 + 180) % 360 - 180))[:2]
        outer.append(dict(pos="Z", strand=k, ip=ip, node=int(pts[0]["node"]), port=port,
                          universes=[int(pts[0]["universe"])],
                          leds=len(pts), total=total, dmx_last=int(pts[-1]["dmx"]),
                          th=[round(float(x % 360), 2) for x in t],
                          r=[round(float(x), 1) for x in rr],
                          xy=[[round(float(a), 1), round(float(b), 1)] for a, b in xy], hang=hang,
                          fed_between=sorted(rib_no[i_] for i_ in near), ribs_passed=passed,
                          turns=round(float(np.ptp(np.degrees(np.unwrap(np.radians(t)))) / 360), 3)))

    # ports that exist on the nodes but never lit up for the camera
    meta = json.load(open(META))
    seen = {(s["ip"], s["port"]) for s in strips} | {(o["ip"], o["port"]) for o in outer}
    dark = [dict(ip=ip, port=p, universes=[2 * p - 2, 2 * p - 1])
            for ip in meta["artnet_ips"] for p in range(1, 9) if (ip, p) not in seen]

    handoff = [dict(frm=l["frm"], to=l["to"], gap_uv=l["gap_uv"],
                    th_end=round(float(th[start_of[l["to"]] - 1] % 360), 1),
                    th_start=round(float(th[start_of[l["to"]]] % 360), 1)) for l in links]

    res = r_leds[ok] - r_helix[ok]
    out = dict(
        source=os.path.basename(SCAN), fill=FILL,
        photo=load_photo(),
        ledPerM=LED_PER_M, pitch=PITCH, roll=ROLL, port=PORT, joint=JOINT, ribT=RIB_T,
        delta=round(delta, 2), anchorSpread=round(float(np.ptp(diffs)), 2), Hc=Hc,
        ribAngle=metal["ribAngle"], ribNo=rib_no, teeth=metal["teeth"],
        helixA=A, helixB=B,
        turns=round(float((th[0] - th[-1]) / 360), 2),
        toothOuter=round(float(H[0] + 1), 2), toothInner=round(float(H[-1] + 1), 2),
        totalLeds=n, totalM=round(n * PITCH / 1000, 2),
        led_th=[round(float(x), 2) for x in th],
        strips=strips, junctions=juncs, crossings=len(cross),
        anchors=anchor_rows, checks=checks, handoff=handoff,
        outer=outer, dark=dark, controllers=CONTROLLERS,
        rCheck=dict(median=round(float(np.median(res)), 1),
                    p10=round(float(np.percentile(res, 10)), 1),
                    p90=round(float(np.percentile(res, 90)), 1)),
    )
    # helix tooth range each rib actually carries, so a position can also be counted from the rim
    # what a technician reads: the cable label; fall back to the position if a port has no label
    for x in strips + outer:
        lab, slot = cable_label(x["ip"], x["port"])
        x["name"] = lab if lab and lab != "empty" else x["pos"]
        x["lid_slot"] = slot
        x["label_matches"] = x["name"] == x["pos"]
    out["lidPortOrder"] = LID_PORT_ORDER
    out["ribTeethRange"] = [[round((t[0] - A) / B - p / 15) + 1, round((t[-1] - A) / B - p / 15) + 1]
                            for p, t in enumerate(metal["teeth"])]
    # which node's strips start near which controller (circular mean of start angles)
    ctr = []
    for c in CONTROLLERS:
        a, b = (rib_ang[rib_no.index(x)] for x in c)
        ctr.append(math.degrees(math.atan2(math.sin(math.radians(a)) + math.sin(math.radians(b)),
                                           math.cos(math.radians(a)) + math.cos(math.radians(b)))) % 360)
    node_near = []
    for ip in meta["artnet_ips"]:
        a = [s["th_start"] for s in strips if s["ip"] == ip] + [o["th"][0] for o in outer if o["ip"] == ip]
        if not a:
            continue
        z = complex(np.mean(np.cos(np.radians(a))), np.mean(np.sin(np.radians(a))))
        mean = math.degrees(math.atan2(z.imag, z.real)) % 360
        dist = [abs((mean - c + 180) % 360 - 180) for c in ctr]
        order = sorted(range(len(ctr)), key=lambda i: dist[i])
        node_near.append(dict(ip=ip, mean=round(mean), spread=round(abs(z), 2),
                              nearest=order[0] + 1, off=round(dist[order[0]]),
                              second=order[1] + 1, off2=round(dist[order[1]]),
                              clear=dist[order[1]] - dist[order[0]] > 30))
    out["nodeNear"] = node_near
    json.dump(out, open(os.path.join(HERE, "planner_data.json"), "w"))

    with open(os.path.join(HERE, "strips.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "position_from_centre", "lid_slot", "order_outer_to_inner", "strand_id", "ip", "port", "universes", "dmx",
                    "leds", "start_rib", "start_tooth", "start_deg", "end_rib", "end_tooth",
                    "end_deg", "turns", "rib_crossings", "ribs_passed"])
        for s in strips:
            c = s["crossings"]
            w.writerow([s["name"], s["pos"], s["lid_slot"], s["order"], s["strand"], s["ip"], s["port"],
                        "-".join(map(str, s["universes"])), f'{s["dmx_first"]}-{s["dmx_last"]}',
                        s["leds"], c[0][0] if c else "", c[0][1] if c else "", s["th_start"],
                        c[-1][0] if c else "", c[-1][1] if c else "", s["th_end"], s["turns"],
                        len(c), " ".join(f"{a:02d}#{b}" for a, b, _ in c)])
    with open(os.path.join(HERE, "junctions.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "kind", "strip", "strand_id", "ip", "port", "from_centre_tip_m", "deg", "r_mm",
                    "nearest_rib", "tooth_helix", "tooth_from_rim", "between_ribs", "clearance_mm", "verdict"])
        for j in juncs:
            w.writerow([j["id"], j["kind"], strips[j["order"]]["name"], j["strand"], j["ip"], j["port"], j["from_tip_m"],
                        j["th"], j["r"], j["rib"], j["tooth"],
                        out["ribTeethRange"][rib_no.index(j["rib"])][1] - j["tooth"] + 1,
                        "/".join(f"{x:02d}" for x in j["ribs"] if x is not None),
                        j["clear"], j["verdict"]])

    # write into the page
    if os.path.exists(PAGE):
        s = open(PAGE).read()
        blob = "/*DATA:BEGIN*/" + json.dumps(out, separators=(",", ":")) + "/*DATA:END*/"
        s2 = re.sub(r"/\*DATA:BEGIN\*/.*?/\*DATA:END\*/", lambda m: blob, s, flags=re.S)
        open(PAGE, "w").write(s2)

    # report
    print(f"chain: {len(chain)} strips, {n} LEDs = {n * PITCH / 1000:.2f} m, "
          f"{out['turns']} turns; off-chain strands {rest}; dark ports {len(dark)}")
    print(f"handoff gaps (uv): max {max(l['gap_uv'] for l in links):.4f}")
    print(f"rotation delta {delta:.2f} deg (anchor spread {np.ptp(diffs):.2f}); "
          f"tooth outer {H[0] + 1:.2f}, inner {H[-1] + 1:.2f}")
    print("anchors:", [(a["rib"], a["tooth"], a["ok"], a["off_mm"]) for a in anchor_rows])
    print("checks:", [(c["label"], c["ok"], c["junction"]) for c in checks])
    print(f"r from LEDs/turn minus helix r: median {out['rCheck']['median']} mm "
          f"(p10 {out['rCheck']['p10']}, p90 {out['rCheck']['p90']})")
    for v in ("clash", "marginal"):
        bad = [j for j in juncs if j["verdict"] == v]
        print(f"{v}: {len(bad)}", [(j['id'], j['kind'], j['rib'], j['tooth'], float(j['clear'])) for j in bad])


if __name__ == "__main__":
    main()
