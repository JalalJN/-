import cv2
import numpy as np
import os

# =========================
# Paths
# =========================
folder = r"C:\Users\Jalal\Desktop\geoscan"

map_path = os.path.join(folder, "map.jpg")
output_path = os.path.join(folder, "map_with_real_markers.jpg")

# =========================
# Load map
# =========================
map_img = cv2.imread(map_path)

if map_img is None:
    print("ERROR: map.jpg not found")
    print("Check this path:", map_path)
    exit()

H, W = map_img.shape[:2]

# =========================
# ArUco dictionary


# =========================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# =========================
# Function to create marker
# =========================
def create_marker(marker_id, marker_size=300, margin=40):
    marker = cv2.aruco.generateImageMarker(
        aruco_dict,
        marker_id,
        marker_size
    )

    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    canvas = np.ones(
        (marker_size + 2 * margin, marker_size + 2 * margin, 3),
        dtype=np.uint8
    ) * 255

    canvas[
        margin:margin + marker_size,
        margin:margin + marker_size
    ] = marker_bgr

    return canvas

# =========================
# Create real task markers
# =========================
markers = {
    3: create_marker(3),
    23: create_marker(23),
    42: create_marker(42),
    117: create_marker(117),
}

# =========================
# Marker positions on the map

# =========================
positions = {
    3:   (int(W * 0.18), int(H * 0.25)),
    23:  (int(W * 0.60), int(H * 0.25)),
    42:  (int(W * 0.20), int(H * 0.65)),
    117: (int(W * 0.65), int(H * 0.65)),
}

# =========================
# Put markers on map
# =========================
for marker_id, marker_img in markers.items():
    x, y = positions[marker_id]
    mh, mw = marker_img.shape[:2]

    map_img[y:y + mh, x:x + mw] = marker_img

# =========================
# Save result
# =========================
cv2.imwrite(output_path, map_img)

print("Test map saved:")
print(output_path)

cv2.imshow("Map with real markers", map_img)
cv2.waitKey(0)
cv2.destroyAllWindows()