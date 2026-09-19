"""Static figure of the planner map for the manual (manual/images/tube-planner-map.png).
Run after build_planner_data.py:  $PY make_manual_figure.py"""
import json, collections
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
from matplotlib import patheffects as pe

d = json.load(open('planner_data.json'))
th = np.array(d['led_th']); r = d['helixA'] + d['helixB'] * ((th - d['ribAngle'][0]) / 360 + d['Hc'])
X = r * np.cos(np.radians(th)); Y = r * np.sin(np.radians(th))
col = {'101': '#2F6FB0', '102': '#B8651B', '103': '#2E8468', '104': '#9A3E8C'}
ver = {'clash': '#C43D50', 'marginal': '#D09A10', 'clear': '#1F9D55'}
fig, ax = plt.subplots(figsize=(12, 13.2)); fig.patch.set_facecolor('white')
for a, n in zip(d['ribAngle'], d['ribNo']):
    t = np.radians(a)
    ax.plot([100*np.cos(t), 696*np.cos(t)], [100*np.sin(t), 696*np.sin(t)], color='#B6C2CA', lw=2.2, zorder=1)
    ax.text(718*np.cos(t), 718*np.sin(t), f'{n:02d}', ha='center', va='center', fontsize=11,
            color='#5C6B76', family='monospace', weight='bold')
halo = [pe.withStroke(linewidth=3.5, foreground='white')]
for s in d['strips']:
    sl = slice(s['g0'], s['g1'] + 1)
    ax.plot(X[sl], Y[sl], color=col[s['ip'][-3:]], lw=1.9, zorder=2, solid_capstyle='butt')
    m = s["g0"] + (s["g1"] - s["g0"]) // 4          # a quarter along: the midpoint is the roll splice
    ax.text(X[m], Y[m], s['name'], ha='center', va='center', fontsize=9.5, weight='bold', family='monospace',
            color='#17202A' if s['label_matches'] else '#B9791B', path_effects=halo, zorder=6)
for o in d['outer']:
    xy = np.array(o['xy']); hg = np.array(o['hang']); hg = hg[hg[:, 1] > -900]
    c = col[o['ip'][-3:]]
    ax.plot(xy[:, 0], xy[:, 1], color=c, lw=1.9, zorder=2)
    ax.plot(hg[:, 0], hg[:, 1], '--', color=c, lw=1.9, zorder=2)
    ax.text(xy[len(xy)//2, 0] - 18, xy[len(xy)//2, 1], o['name'], ha='right', va='center', fontsize=9.5,
            weight='bold', family='monospace', path_effects=halo, zorder=6)
    ax.annotate('hangs free', (hg[-1, 0], hg[-1, 1]), xytext=(8, 0), textcoords='offset points',
                fontsize=9, color=c, va='center')
drawn = collections.Counter()
for j in d['junctions']:
    t = np.radians(j['th']); x, y = j['r']*np.cos(t), j['r']*np.sin(t)
    mk = {'feed': 's', 'splice': 'o', 'tip': 'D'}[j['kind']]
    big = j['verdict'] == 'clash'
    ax.scatter([x], [y], marker=mk, s=110 if big else 60, c=ver[j['verdict']], edgecolors='white',
               linewidths=1.2, zorder=7)
    if big:
        ax.scatter([x], [y], marker='o', s=420, facecolors='none', edgecolors=ver['clash'], linewidths=1.3, zorder=7)
    drawn[mk] += 1
rib = dict(zip(d['ribNo'], d['ribAngle']))
for i, (a, b) in enumerate(d['controllers']):
    A, B = rib[a], rib[b]
    if abs(B - A) > 180:
        if B < A: B += 360
        else: A += 360
    mid = np.radians((A + B) / 2); cx, cy = 752*np.cos(mid), 752*np.sin(mid)
    rect = Rectangle((-60, -20), 120, 40, color='#46566B', zorder=3)
    rect.set_transform(matplotlib.transforms.Affine2D().rotate(mid - np.pi/2).translate(cx, cy) + ax.transData)
    ax.add_patch(rect)
    ax.text(cx, cy, f'C{i+1}', color='white', ha='center', va='center', fontsize=9, weight='bold', family='monospace', zorder=4)
    ax.text(800*np.cos(mid), 800*np.sin(mid), f'.10{i+1}', color='#46566B', ha='center', va='center', fontsize=9, family='monospace')
h = [Line2D([], [], color=col[k], lw=3, label=f'controller #{int(k)-100} (192.168.0.{k})') for k in col] + [
    Line2D([], [], ls='', marker='s', color=ver['clear'], label='data in (port feed)'),
    Line2D([], [], ls='', marker='o', color=ver['clear'], label='2.5 m roll splice'),
    Line2D([], [], ls='', marker='o', color=ver['clash'], label='joint on a rib (ringed)'),
    Line2D([], [], ls='', marker='o', color=ver['marginal'], label='marginal'),
    Line2D([], [], ls='', marker='$A$', color='#B9791B', label='cable label out of order')]
ax.legend(handles=h, loc='lower right', fontsize=9, frameon=True)
ax.set_aspect('equal'); ax.axis('off'); ax.set_xlim(-830, 830); ax.set_ylim(-900, 830)
ax.set_title('Tube planner – LED strips on the ribs, seen from the front\n'
             'first 6×12 mm unit, LED scan of 18 Sep 2026 – labels are the cable labels', fontsize=12)
plt.savefig('../manual/images/tube-planner-map.png', dpi=170, bbox_inches='tight')
print(dict(drawn))
