import cv2
import numpy as np
import os

# =========================
# Paths
# =========================
image_path = "map_with_real_markers.jpg"
output_path = "final_result.jpg"

# =========================
# Load image
# =========================
img = cv2.imread(image_path)

if img is None:
    raise FileNotFoundError(f"ERROR: image not found. Check this path: {image_path}")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

H, W = img.shape[:2]

# =========================
# Detect drone start coordinate
# =========================
# Камера смотрит вниз, поэтому центр изображения считаем точкой старта дрона
start_px = W // 2
start_py = H // 2

start_x_meter = start_px / W * 3.0
start_y_meter = start_py / H * 3.0

START_COORDINATE = (start_x_meter, start_y_meter)
start_coordinate = START_COORDINATE

print("DRONE START COORDINATE")
print("=======================")
print("Start:", round(start_x_meter, 3), round(start_y_meter, 3), "m")
print("=======================")

# =========================
# ArUco dictionary
# =========================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

corners, ids, rejected = detector.detectMarkers(gray)

# =========================
# Target dictionary from the task
# =========================
target_dictionary = {
    3:   {"type": "allowed",   "letter": "И"},
    23:  {"type": "allowed",   "letter": "Т"},
    42:  {"type": "forbidden", "letter": "М"},
    117: {"type": "forbidden", "letter": "О"},
}

found_targets = []

if ids is None:
    print("No ArUco markers detected")
else:
    print("Detected large markers:")
    print("=======================")

    # =========================
    # Process detected markers
    # =========================
    for i, marker_id in enumerate(ids.flatten()):
        pts = corners[i][0]
        area = cv2.contourArea(pts)

        # Отсекаем маленькие маркеры / шум
        if area < 50000:
            continue

        marker_id = int(marker_id)

        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))

        x_meter = cx / W * 3.0
        y_meter = cy / H * 3.0

        if marker_id in target_dictionary:
            target_type = target_dictionary[marker_id]["type"]
            letter = target_dictionary[marker_id]["letter"]
        else:
            target_type = "unknown"
            letter = "?"

        found_targets.append({
            "id": marker_id,
            "x": x_meter,
            "y": y_meter,
            "type": target_type,
            "letter": letter
        })

        print("ID:", marker_id)
        print("Area:", round(area, 2))
        print("Coordinates:", round(x_meter, 3), round(y_meter, 3), "m")
        print("Type:", target_type)
        print("Letter:", letter)
        print("=======================")

        # Draw result on image
        cv2.polylines(img, [pts.astype(int)], True, (0, 255, 0), 4)
        cv2.circle(img, (cx, cy), 12, (0, 0, 255), -1)

        cv2.putText(
            img,
            f"ID:{marker_id} {target_type}",
            (cx + 20, cy),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 0, 255),
            3
        )

# =========================
# Final word
# =========================
found_targets_sorted = sorted(found_targets, key=lambda item: item["id"])
final_word = "".join(item["letter"] for item in found_targets_sorted)

print("FINAL RESULT")
print("=======================")
print("Found IDs:", [item["id"] for item in found_targets_sorted])
print("Final word:", final_word)

# =========================
# Save output image
# =========================
cv2.imwrite(output_path, img)

print("Result saved:")
print(output_path)

# =========================
# Show image only when this file is run directly
# =========================
if __name__ == "__main__":
    cv2.imshow("Final result", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()