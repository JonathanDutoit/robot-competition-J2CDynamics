import argparse
import os

import numpy as np
import yaml

try:
    from scipy.ndimage import distance_transform_edt
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False

"""
command:
python3 generate_waypoints.py \
    --map ../../j2cdynamics_bringup/maps/arena/map.yaml \
    --keepout ../../j2cdynamics_bringup/maps/arena/map_keepout.yaml \
    --spacing 0.5 --robot-radius 0.25 --half right \
    --start-x 1.25 --start-y 0.4 \
    --out ../../j2cdynamics_bringup/maps/arena/waypoints_right.yaml \
    --debug-png waypoint_debug.png

Coverage strategy
-----------------
The arena is a rotated rectangle in the image.  We:
  1. Fit a minimum-area rectangle to the free space (robust for squares,
     unlike PCA whose axes are degenerate when the two extents are equal).
  2. Pick the rectangle corner nearest the robot start  -> P0  ("bottom-left").
  3. The diagonally-opposite corner is P2 ("top-right").  The diagonal P0->P2
     splits the arena; --half right keeps the lower-right triangle (the one the
     robot reaches by driving right from the start), --half left keeps the
     complementary triangle, --half all keeps the whole rectangle.
  4. A boustrophedon ("lawnmower") snake is built directly in the arena frame:
     row 0 (bottom) goes right, the next goes left to the diagonal limit, then
     right, etc.  No post-hoc list rotation, so the path stays continuous and
     starts at the corner under the robot.
"""

# ---------------------------------------------------------------------
# MAP IO
# ---------------------------------------------------------------------

def load_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P5"

        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()

        w, h = map(int, line.split())
        int(f.readline())  # maxval

        data = np.frombuffer(
            f.read(w * h),
            dtype=np.uint8,
        ).reshape(h, w)

    return data


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    img = load_pgm(
        os.path.join(
            os.path.dirname(yaml_path),
            meta["image"],
        )
    )

    return img, meta


# ---------------------------------------------------------------------
# EROSION
# ---------------------------------------------------------------------

def erode_free(free, radius_px):
    if radius_px <= 0:
        return free

    if not _HAVE_SCIPY:
        raise RuntimeError("Install scipy for distance transform erosion.")

    dist = distance_transform_edt(free)

    return dist >= radius_px


# ---------------------------------------------------------------------
# IMAGE <-> WORLD
# ---------------------------------------------------------------------

def pixel_to_world(col, row, meta, H):
    res = float(meta["resolution"])
    ox = float(meta["origin"][0])
    oy = float(meta["origin"][1])

    wx = ox + (col + 0.5) * res
    wy = oy + (H - 1 - row + 0.5) * res

    return wx, wy


def world_to_pixel(wx, wy, meta, H):
    res = float(meta["resolution"])
    ox = float(meta["origin"][0])
    oy = float(meta["origin"][1])

    col = (wx - ox) / res - 0.5
    row = H - 1 - ((wy - oy) / res - 0.5)

    return col, row


# ---------------------------------------------------------------------
# ARENA FRAME  (minimum-area rectangle, robust for squares)
# ---------------------------------------------------------------------

def fit_arena_rect(free, meta, H):
    """Return the 4 arena corners in (world, pixel) coords, in cyclic order."""
    ys, xs = np.where(free)
    pts = np.column_stack((xs, ys)).astype(float)  # (col, row)

    if len(pts) == 0:
        raise RuntimeError("No free space found after erosion.")

    # subsample for the angle sweep; extremes are preserved well enough
    if len(pts) > 4000:
        step = len(pts) // 4000 + 1
        sample = pts[::step]
    else:
        sample = pts

    best = None
    for deg in np.arange(0.0, 90.0, 0.25):
        th = np.radians(deg)
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s], [s, c]])
        rot = sample @ R.T
        mn = rot.min(axis=0)
        mx = rot.max(axis=0)
        area = (mx[0] - mn[0]) * (mx[1] - mn[1])
        if best is None or area < best[0]:
            best = (area, th, mn.copy(), mx.copy())

    _, th, mn, mx = best
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])

    corners_rot = np.array([
        [mn[0], mn[1]],
        [mx[0], mn[1]],
        [mx[0], mx[1]],
        [mn[0], mx[1]],
    ])
    corners_pix = corners_rot @ R  # rotate back into pixel frame

    corners_world = np.array([
        pixel_to_world(cp[0], cp[1], meta, H) for cp in corners_pix
    ])

    return corners_world, corners_pix


# ---------------------------------------------------------------------
# SNAKE
# ---------------------------------------------------------------------

def build_snake(P0, u_dir, v_dir, Lu, Lv, spacing, half, inset):
    """Boustrophedon (u, v) coverage of a triangle/rectangle in arena frame.

    Rows are stacked along v (bottom->top). Row 0 sweeps +u ("right").
    For half=='right' a row is clipped to u/Lu >= v/Lv (lower-right triangle).

    `inset` keeps the grid off the eroded boundary (rows exactly on the wall
    are numerically fragile after erosion). The diagonal ratio still uses the
    full Lu/Lv so it stays anchored to the true corners P0 / P2.
    """
    us = np.arange(inset, Lu - inset + 1e-9, spacing)
    vs = np.arange(inset, Lv - inset + 1e-9, spacing)

    ordered = []
    for j, vv in enumerate(vs):
        ratio_v = vv / Lv if Lv > 0 else 0.0
        row = []
        for uu in us:
            ratio_u = uu / Lu if Lu > 0 else 0.0
            if half == "right" and ratio_u < ratio_v - 1e-9:
                continue
            if half == "left" and ratio_u > ratio_v + 1e-9:
                continue
            row.append((uu, vv))

        if j % 2 == 1:          # boustrophedon: odd rows reverse
            row.reverse()

        ordered.extend(row)

    return [P0 + uu * u_dir + vv * v_dir for uu, vv in ordered]


# ---------------------------------------------------------------------
# DEBUG VIZ
# ---------------------------------------------------------------------

def save_debug(grid, meta, H, corners_pix, P0, P2, waypoints, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping debug PNG")
        return

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(grid, cmap="gray", origin="upper")

    cp = np.vstack([corners_pix, corners_pix[0]])
    ax.plot(cp[:, 0], cp[:, 1], "c-", lw=1, label="arena rect")

    p0c = world_to_pixel(P0[0], P0[1], meta, H)
    p2c = world_to_pixel(P2[0], P2[1], meta, H)
    ax.plot([p0c[0], p2c[0]], [p0c[1], p2c[1]], "g--", lw=2, label="diagonal")

    px, py = [], []
    for wx, wy, _ in waypoints:
        cc, rr = world_to_pixel(wx, wy, meta, H)
        px.append(cc)
        py.append(rr)

    ax.plot(px, py, "-", color="orange", lw=0.8)
    ax.scatter(px, py, c="red", s=16, zorder=3)
    for i, (cc, rr) in enumerate(zip(px, py)):
        ax.text(cc + 1, rr, str(i), fontsize=6, color="blue")
    if px:
        ax.scatter([px[0]], [py[0]], c="lime", s=70, zorder=4,
                   edgecolors="k", label="start")

    ax.legend(loc="upper right", fontsize=7)
    ax.set_title("coverage waypoints")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Wrote debug image {path}")


# ---------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--keepout")
    ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--robot-radius", type=float, default=0.25)
    ap.add_argument("--half", choices=["left", "right", "all"], default="right")
    ap.add_argument("--start-x", type=float, required=True)
    ap.add_argument("--start-y", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--inset", type=float, default=None,
                    help="margin (m) to keep rows off the eroded wall; "
                         "defaults to min(spacing/2, robot_radius)")
    ap.add_argument("--debug-png", default=None)
    args = ap.parse_args()

    if args.inset is None:
        args.inset = min(args.spacing * 0.5, args.robot_radius)

    grid, meta = load_map(args.map)
    H, W = grid.shape
    res = float(meta["resolution"])

    free = grid >= 250
    if args.keepout:
        kgrid, _ = load_map(args.keepout)
        if kgrid.shape == free.shape:
            free &= (kgrid >= 250)

    radius_px = int(round(args.robot_radius / res))
    free = erode_free(free, radius_px)

    # --- arena rectangle + corners -----------------------------------
    corners_world, corners_pix = fit_arena_rect(free, meta, H)

    start = np.array([args.start_x, args.start_y])
    i0 = int(np.argmin(np.linalg.norm(corners_world - start, axis=1)))

    P0 = corners_world[i0]                  # corner under the robot ("BL")
    Pa = corners_world[(i0 + 1) % 4]
    Pb = corners_world[(i0 - 1) % 4]
    P2 = corners_world[(i0 + 2) % 4]        # opposite corner ("TR")

    ea = Pa - P0
    eb = Pb - P0

    # u-axis = edge most aligned with world +x ("right"); v-axis = "up"
    if ea[0] >= eb[0]:
        u_edge, v_edge = ea, eb
    else:
        u_edge, v_edge = eb, ea

    Lu = float(np.linalg.norm(u_edge))
    Lv = float(np.linalg.norm(v_edge))
    u_dir = u_edge / Lu
    v_dir = v_edge / Lv

    # --- snake in arena frame ----------------------------------------
    raw_nodes = build_snake(P0, u_dir, v_dir, Lu, Lv,
                            args.spacing, args.half, args.inset)

    # --- keep only nodes that land in free space ---------------------
    world_nodes = []
    for p in raw_nodes:
        col, row = world_to_pixel(p[0], p[1], meta, H)
        ci, ri = int(round(col)), int(round(row))
        if 0 <= ri < H and 0 <= ci < W and free[ri, ci]:
            world_nodes.append((float(p[0]), float(p[1])))

    # --- yaw points toward the next waypoint -------------------------
    waypoints = []
    n = len(world_nodes)
    for k, (wx, wy) in enumerate(world_nodes):
        if k < n - 1:
            nx, ny = world_nodes[k + 1]
            yaw = float(np.arctan2(ny - wy, nx - wx))
        else:
            yaw = waypoints[-1][2] if waypoints else 0.0
        waypoints.append([round(wx, 3), round(wy, 3), round(yaw, 3)])

    with open(args.out, "w") as f:
        yaml.safe_dump(
            {
                "_comment":
                    f"coverage grid half={args.half} "
                    f"spacing={args.spacing} "
                    f"arena={Lu:.2f}x{Lv:.2f}m "
                    f"start_corner=({P0[0]:.2f},{P0[1]:.2f})",
                "waypoints": waypoints,
            },
            f,
            sort_keys=False,
        )

    print(f"Generated {len(waypoints)} waypoints "
          f"(arena {Lu:.2f}x{Lv:.2f} m, half={args.half})")

    if args.debug_png:
        save_debug(grid, meta, H, corners_pix, P0, P2, waypoints, args.debug_png)


if __name__ == "__main__":
    main()