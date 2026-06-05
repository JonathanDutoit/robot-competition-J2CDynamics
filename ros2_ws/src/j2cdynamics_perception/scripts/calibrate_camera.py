#!/usr/bin/env python3
"""
Camera intrinsics calibration for ground-plane projection.

Capture ~20 photos of a printed checkerboard at varied angles/distances with the
SAME camera + resolution the detector uses (640x480), drop them in a folder, then:

    python3 calibrate_camera.py --images ./calib_imgs --rows 6 --cols 9 --square 0.025

It prints `camera_matrix` and `dist_coeffs` ready to paste into
config/camera_calibration.yaml. `--rows`/`--cols` are the count of INNER corners
(a board with 7x10 squares has 6x9 inner corners). `--square` is the side length
of one square in metres.

EXTRINSICS (height / pitch / offsets) are measured separately — see the notes at
the bottom of this file. They matter more than intrinsics for near-field accuracy.
"""
import argparse
import glob
import os

import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("calibrate_camera needs OpenCV (pip install opencv-python)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', required=True, help='folder of checkerboard photos')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column')
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row')
    ap.add_argument('--square', type=float, default=0.025, help='square size (m)')
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    objpoints, imgpoints = [], []
    files = sorted(sum([glob.glob(os.path.join(args.images, f'*.{e}'))
                        for e in ('jpg', 'jpeg', 'png', 'JPG', 'PNG')], []))
    if not files:
        raise SystemExit(f'no images found in {args.images}')

    shape = None
    found = 0
    for fn in files:
        img = cv2.imread(fn)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not ok:
            print(f'  no corners: {os.path.basename(fn)}')
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        objpoints.append(objp)
        imgpoints.append(corners)
        found += 1
        print(f'  OK: {os.path.basename(fn)}')

    if found < 5:
        raise SystemExit(f'only {found} usable images — need ~15-20 for a good fit')

    rms, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    print(f'\nused {found} images   |   RMS reprojection error: {rms:.3f} px '
          f'(aim < 0.5; > 1.0 means re-shoot)\n')
    print(f'# calibrated at {shape[0]}x{shape[1]}  — must match the detector stream')
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print('camera_matrix: [{:.2f}, 0.0, {:.2f},'.format(fx, cx))
    print('                 0.0, {:.2f}, {:.2f},'.format(fy, cy))
    print('                 0.0, 0.0, 1.0]')
    d = dist.reshape(-1)
    print('dist_coeffs:   [' + ', '.join(f'{v:.5f}' for v in d) + ']')


if __name__ == '__main__':
    main()

# ── EXTRINSICS (camera mount on base_link) ────────────────────────────────────
#
# The ground_projection node needs height / pitch / x_offset / y_offset. Two ways:
#
# 1) MEASURE (fast, good enough to start):
#    height   – tape from the floor to the camera lens centre.
#    x_offset – horizontal distance from base_link origin (wheel axle centre)
#               forward to a point directly under the lens.
#    pitch    – phone level / protractor on the camera body; angle below horizontal.
#    y_offset – lateral offset (0 if the camera is centred).
#
# 2) solvePnP (more accurate — what the prior team did):
#    Lay the checkerboard FLAT on the ground at a known pose relative to base_link.
#    Detect its corners, build the matching ground coordinates in base_link, then:
#       ok, rvec, tvec = cv2.solvePnP(obj_base, img_pts, K, dist)
#    That yields the full camera↔base transform. Decompose to height/pitch, or
#    extend GroundProjector to take (rvec, tvec) directly. Verify by projecting a
#    duplo at a tape-measured spot and checking /duplo_points lands within a few cm.
