#!/usr/bin/env python3
"""
Capture calibration stills with the EXACT same camera configuration the detector
uses, so the intrinsics you compute are valid for the bboxes it reports.

Run this INSIDE the camera container (it has picamera2). Press ENTER to grab a
frame, 'q'+ENTER to quit. Aim for ~20 shots of the checkerboard at varied angles,
distances, and positions (especially push the board into the frame CORNERS — that
is where lens distortion is strongest and where calibration needs data).

    python3 capture_calib.py --out ./calib_imgs

The config below mirrors camera.py (main 640x480 RGB888 + lores 320x320). Keep it
identical: a different sensor mode = a different field of view = wrong intrinsics.
"""
import argparse
import os
import time

from picamera2 import Picamera2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='./calib_imgs', help='output folder')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    picam2 = Picamera2()
    # ── MUST match camera.py exactly ──────────────────────────────────────────
    config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        lores={"size": (320, 320), "format": "RGB888"},
        buffer_count=2,
        queue=False,
        encode="main",
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)            # let auto-exposure / white balance settle

    print(f"saving to {args.out}/ — ENTER to capture, 'q'+ENTER to quit")
    i = 0
    while True:
        if input().strip().lower() == 'q':
            break
        fn = os.path.join(args.out, f'calib_{i:02d}.jpg')
        picam2.capture_file(fn)         # captures the 640x480 main stream
        print(f'  saved {fn}')
        i += 1
    picam2.stop()
    print(f'done — {i} images')


if __name__ == '__main__':
    main()
