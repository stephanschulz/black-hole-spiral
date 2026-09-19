# Where the 2.5 m tubes join, and whether a junction can miss every rib

Answer up front: **with fixed 2.5 m tubes and a 25 mm splice, no arrangement is clean.**
All 175 possible phases were tried. The best one puts **1 junction of 31 on a rib** (missing
by 2 mm) and **3 inside a clip**. Shifting things in or out does not fix it — it only moves
which junctions are the bad ones.

Two things do fix it, and both are cheap. See *What to do* below.

## The measurement this rests on

Worked in **LED index**, not millimetres of arc. The strip is 70/m, so a 2.5 m tube is exactly
**175 LEDs** and every junction lands on an integer index. Rib crossings come from the scan's
**angles**, which a radial warp cannot distort — so no pitch law is integrated over 77 m and no
error accumulates. (An earlier attempt integrating the fitted pitch law gave 36.4 turns and
86 m against the scan's own 28.73 turns and 77 m; that route is abandoned.)

| | |
|---|---|
| measured strip | **5400 LEDs = 77.14 m**, r = 700 -> 130 mm |
| turns | **28.73** |
| rib crossings | **431** (15 ribs at theta = 6 + 24k, one crossing per rib per turn) |
| junctions at 2.5 m | **31** |

The 15 metal ribs and the camera holder's own fins are at the **same** angles (checked in one
file, one frame: the rib reads 6.00 deg and the holder's fin at 6.0 deg is coplanar with it).
An earlier note in `spiral_analysis.md` claiming fin slots at 18 + 24k is wrong.

## Why it cannot be phased clean

The arc between two rib crossings is just `0.4189 * r`:

| r (mm) | 700 | 500 | 400 | 300 | 200 | 160 | 140 |
|---|---|---|---|---|---|---|---|
| between crossings | 293 | 209 | 168 | 126 | 84 | 67 | 59 mm |

A junction needs the splice plus whatever sits on the crossing:

- against the **3.2 mm rib**: 3.2 + 25 = **28.2 mm** blocked
- against a **30 mm `neon-open-channel` clip**: 30 + 25 = **55 mm** blocked

So roughly 16 % of the tube is blocked (28.2 mm out of a 179 mm average spacing). With 31
junctions, the chance all of them miss is about 0.84^31 = **0.5 %** — and across 175 phases you
would expect about one clean phase. There is none. That is bad luck, not a missing trick: the
crossing spacing sweeps continuously from 293 mm to 59 mm, so there is no resonance to align to.

## What to do

**Either** shorten the splice. This is the whole game:

| splice | phases clean of the rib | best case |
|---|---|---|
| 25 mm | **0** of 175 | 1 junction on a rib |
| 20 mm | 1 | clean |
| 15 mm | 4 | clean |
| 10 mm | 17 | clean |
| 5 mm | 36 | clean |

**Or** cut the tubes. LED neon cuts at 3-LED marks (42.86 mm). Taking the longest length that
clears, each time: **32 tubes, 0.67 m of offcut in total, every junction at least 5.6 mm clear
of a rib** — and if you want them clear of the full 30 mm clip too, 1.14 m of offcut and 6.9 mm
of margin. This is the robust answer and it costs less than half a tube.

**And in any case** a junction that lands in a clip is only a problem for the *clip*. Leaving
3 of 431 crossings unclipped, or fitting a 15 mm clip there, changes nothing structurally.

## The plan, if you keep full 2.5 m tubes

First tube at the rim **0.914 m (64 LEDs)**, then 30 full 2.5 m tubes. Per-junction positions
are in `tube_junctions.csv` (LED index, metres from the rim, turns, radius, angle, and the
clearance against both the rib and the clip). Three junctions want their clip skipped:
**10.91 m, 60.91 m and 75.91 m** from the rim.

## What is NOT settled

- **The inner tail.** The scan's last ~40 LEDs dive inward at 87 mm/turn instead of ~15 — that
  is the old holder's bad winding, and the scan's radial scale disagrees with the real part
  inside r ~ 250 anyway. So everything above is anchored at the **rim**, where the data is
  solid. Your "70 cm past the last support at the 10 o'clock rib" reconciles with the geometry
  (the holder's 1.33 turns from r = 101 to r = 81.3 is 0.76 m), but it does not pin the phase.
- **Re-winding inside r ~ 200 costs tube.** The installed strip dives to the centre over its
  last 0.57 m. A proper spiral at the design pitch over that range is several metres longer, so
  budget tube for it before ordering.
- Assumed 70 LEDs/m and a 25 mm splice. Both are parameters at the top of `tube_cut_plan.py`.
