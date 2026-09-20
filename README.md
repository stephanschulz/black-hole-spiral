# Tube Junction Planner

Interactive map of the Black Hole Spiral LED tube: every strip, joint, and controller port, built from the camera scan rather than from a model of the winding.

**Open the planner:** [https://stephanschulz.ca/black-hole-spiral/](https://stephanschulz.ca/black-hole-spiral/)

The page is a single file. Data is built in, so it needs no server.

![Plan view of the LED spiral on the 15 ribs, with strip controls](docs/screenshots/planner-map.png)

This map is for the **first 6×12 mm unit**, scanned on 18 Sep 2026. A later unit may route strips to controllers and lay the tube on the spiral differently. Scan it and build its own planner.

## What it shows

- Where each strip starts and ends, as a rib and tooth
- Cable labels from the controller lid stickers, next to each strip's position from the centre
- Data direction: every strip is fed at its outer end and runs clockwise (seen from the front) toward the centre
- Whether each data-input joint and each 2.5 m roll splice lands on a rib, near one, or clear
- Which node IP and Art-Net port drive which part of the spiral

![Photo overlay: predicted hand-offs on the studio photo](docs/screenshots/planner-photo.png)

The photo panel draws the fitted spiral over the studio shot of lit strip ends. Green rings are where the LED map puts each hand-off; yellow boxes are the marks on the photo. All 30 marks pair with a predicted strip end (median miss 9 mm).

## How to use it

1. Open [https://stephanschulz.ca/black-hole-spiral/](https://stephanschulz.ca/black-hole-spiral/)
2. Drag the **Strip** slider, or click a strip on the map or a row in the tables
3. Colour the tube by node, by strip, or by 2.5 m roll
4. Click the photo to zoom in on a joint

Labels shown in amber are out of order: **A** is the 22nd strip, labels 22–27 sit at positions 23–28, and labels 17 and 18 are swapped.

## Rebuild

Source and rebuild notes live in [`tube-planner/`](tube-planner/README.md). After you rebuild `tube_junction_planner.html`, copy it so GitHub Pages stays current:

```
cp tube-planner/tube_junction_planner.html docs/index.html
```
