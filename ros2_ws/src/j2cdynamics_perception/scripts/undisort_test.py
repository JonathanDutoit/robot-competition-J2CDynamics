import cv2
import numpy as np
import glob
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
yaml_path = os.path.join(BASE_DIR, "../config/camera_calibration.yaml")

print("Loading:", yaml_path)
print("Exists:", os.path.exists(yaml_path))

fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)

K = fs.getNode("camera_matrix").mat()
dist = fs.getNode("dist_coeffs").mat()

fs.release()

print("Camera matrix:\n", K)
print("Distortion:\n", dist)

# -------------------------
# INPUT / OUTPUT
# -------------------------
images = glob.glob("calib_imgs/*.jpg")
output_folder = "undistorted"

os.makedirs(output_folder, exist_ok=True)

if len(images) == 0:
    raise RuntimeError("No images found")

# -------------------------
# IMAGE SIZE
# -------------------------
img = cv2.imread(images[0])
h, w = img.shape[:2]

# better projection matrix
newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))

print("New camera matrix:\n", newK)

# -------------------------
# PROCESS
# -------------------------
for fname in images:
    img = cv2.imread(fname)

    undistorted = cv2.undistort(img, K, dist, None, newK)

    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        undistorted = undistorted[y:y+rh, x:x+rw]

    out_path = os.path.join(output_folder, os.path.basename(fname))
    cv2.imwrite(out_path, undistorted)

    print("saved:", out_path)

print("Done.")