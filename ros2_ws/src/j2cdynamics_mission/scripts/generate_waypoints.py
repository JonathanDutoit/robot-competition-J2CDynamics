#!/usr/bin/env python3
"""
Generate a coverage grid of waypoints over the free space of a saved map.

This is the "equidistant nodes" generator for autonomous coverage of a KNOWN
map (not frontier exploration — you already have the map). It:

  1. loads the occupancy map (free = white) and the keepout mask,
  2. erodes the free space by the robot radius so no node hugs a wall,
  3. keeps only the robot's assigned half of the arena,
  4. lays a grid with spacing = the camera's reliable detection range R,
  5. orders the survivors as a boustrophedon (lawnmower) sweep,
  6. writes a waypoints.yaml the mission runner loads.

Run OFFLINE on your laptop, commit the result, then load it in the mission.

Example:
    python3 generate_waypoints.py \
        --map ../../j2cdynamics_bringup/maps/arena/map.yaml \
        --keepout ../../j2cdynamics_bringup/maps/arena/map_keepout.yaml \
        --spacing 1.0 --robot-radius 0.25 --half right \
        --out ../../j2cdynamics_bringup/maps/arena/waypoints_right.yaml
"""
import argparse
import os

import numpy as np
import yaml

try:
    from scipy.ndimage import binary_erosion, distance_transform_edt
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False


# ── map IO ────────────────────────────────────────────────────────────────────
def load_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        assert magic == b'P5', f'{path}: not a binary PGM ({magic!r})'
        line = f.readline()
        while line.startswith(b'#'):          # skip GIMP comment line
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline())                     # maxval
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return data


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    grid = load_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))
    return grid, meta


# ── free-space erosion (keep cells whose whole robot-radius disk is free) ──────
def erode_free(free, radius_px):
    if radius_px <= 0:
        return free
    if _HAVE_SCIPY:
        # distance from each free cell to the nearest obstacle; keep cells that
        # are at least radius_px away → robot footprint never overlaps a wall.
        dist = distance_transform_edt(free)
        return dist >= radius_px
    # pure-numpy fallback: erode with a disk structuring element
    r = radius_px
    ys, xs = np.ogrid[-r:r + 1, -r:r + 1]
    disk = (xs * xs + ys * ys) <= r * r
    H, W = free.shape
    out = free.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if not disk[dy + r, dx + r]:
                continue
            shifted = np.zeros_like(free)
            y0s, y1s = max(0, dy), min(H, H + dy)
            x0s, x1s = max(0, dx), min(W, W + dx)
            shifted[max(0, -dy):max(0, -dy) + (y1s - y0s),
                    max(0, -dx):max(0, -dx) + (x1s - x0s)] = free[y0s:y1s, x0s:x1s]
            out &= shifted
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map.yaml')
    ap.add_argument('--keepout', default=None, help='map_keepout.yaml (optional)')
    ap.add_argument('--spacing', type=float, default=1.0,
                    help='grid spacing in metres ≈ camera reliable detection range R')
    ap.add_argument('--robot-radius', type=float, default=0.25,
                    help='keep nodes at least this far from walls (m)')
    ap.add_argument('--half', choices=['right', 'left', 'all'], default='right',
                    help="which half of the arena this robot covers")
    ap.add_argument('--out', required=True, help='output waypoints.yaml')
    args = ap.parse_args()

    grid, meta = load_map(args.map)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = grid.shape

    free = grid >= 250                         # white = free (trinary: 254 free / 0 occ / 205 unknown)

    if args.keepout:
        kgrid, _ = load_map(args.keepout)
        if kgrid.shape == free.shape:
            free &= (kgrid >= 250)             # drop anything painted into the keepout mask
        else:
            print(f"WARNING: keepout {kgrid.shape} != map {free.shape}; ignoring keepout")

    free_eroded = erode_free(free, int(round(args.robot_radius / res)))

    def cell_to_world(col, row):
        return ox + (col + 0.5) * res, oy + (H - 1 - row + 0.5) * res   # row 0 = top = high y

    x_mid = ox + 0.5 * W * res

    def in_half(wx):
        if args.half == 'right':
            return wx >= x_mid
        if args.half == 'left':
            return wx < x_mid
        return True

    # lay the grid + boustrophedon ordering (reverse every other populated row)
    step = max(1, int(round(args.spacing / res)))
    nodes, flip = [], False
    for row in range(step // 2, H, step):
        row_nodes = []
        for col in range(step // 2, W, step):
            if not free_eroded[row, col]:
                continue
            wx, wy = cell_to_world(col, row)
            if in_half(wx):
                row_nodes.append((wx, wy))
        if not row_nodes:
            continue
        if flip:
            row_nodes.reverse()
        flip = not flip
        nodes.extend(row_nodes)

    wps = [[round(float(x), 3), round(float(y), 3), 0.0] for x, y in nodes]
    with open(args.out, 'w') as f:
        yaml.safe_dump(
            {'_comment': f'coverage grid: half={args.half} spacing={args.spacing}m '
                         f'robot_radius={args.robot_radius}m  ({len(wps)} nodes)',
             'waypoints': wps},
            f, default_flow_style=False, sort_keys=False)

    print(f"\n{len(wps)} waypoints  (half={args.half}, spacing={args.spacing} m)  → {args.out}")
    _ascii_preview(free_eroded, nodes, ox, oy, res, H, W, args.half, x_mid)


def _ascii_preview(free_eroded, nodes, ox, oy, res, H, W, half, x_mid):
    """Tiny terminal preview: '#' wall/blocked, '.' navigable, '*' a waypoint."""
    SC = max(1, W // 70)                       # downsample columns to ~70 wide
    node_cells = {(int((wy - oy) / res), int((wx - ox) / res)) for wx, wy in nodes}
    print(f"   preview (x_mid={x_mid:.2f}, robot covers '{half}'):")
    for row in range(0, H, SC):
        line = []
        for col in range(0, W, SC):
            disp_row = H - 1 - row
            if any((disp_row, c) in node_cells for c in range(col, min(col + SC, W))):
                line.append('*')
            elif free_eroded[disp_row, col]:
                line.append('.')
            else:
                line.append(' ')
        print('   ' + ''.join(line))


if __name__ == '__main__':
    main()
