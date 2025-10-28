import open3d as o3d
import numpy as np
import cv2
import yaml

# Load point cloud
pcd = o3d.io.read_point_cloud("map.pcd")
points = np.asarray(pcd.points)

# Keep only points within a reasonable height (for ground projection)
z_min, z_max = -0.2, 1.0
points = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]

# Scale and translate points to positive image coordinates
resolution = 0.05  # meters per pixel
points[:, :2] = points[:, :2] / resolution
points[:, :2] -= np.min(points[:, :2], axis=0)

# Create empty image
map_size = np.max(points[:, :2], axis=0).astype(int) + 10
img = np.ones((map_size[1], map_size[0]), dtype=np.uint8) * 255

# Draw occupied points (black pixels)
for x, y in points[:, :2].astype(int):
    if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
        img[y, x] = 0

# Flip vertically (ROS expects origin at bottom-left)
img = cv2.flip(img, 0)

# Save map
cv2.imwrite("map.pgm", img)

# Save YAML file for ROS Navigation
map_yaml = {
    "image": "map.pgm",
    "resolution": resolution,
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}
with open("map.yaml", "w") as f:
    yaml.dump(map_yaml, f)

print("✅ Map generated: map.pgm + map.yaml")

