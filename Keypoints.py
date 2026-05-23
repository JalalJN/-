import cv2
import numpy as np
import os

def save_map_features(image_path, output_file="map_data.npz"):
    # 1. чтение карты
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("error: не нашёл файл")
        return

    # 2. ORB detector
    # nfeatures=10000
    orb = cv2.ORB_create(nfeatures=10000)

    # 3. выделение особых точек
    print("сейчас забираю данные")
    keypoints, descriptors = orb.detectAndCompute(img, None)

    # 4. keypoints
    kp_data = np.array([kp.pt for kp in keypoints])
    kp_size = np.array([kp.size for kp in keypoints])
    kp_angle = np.array([kp.angle for kp in keypoints])

    # 5. сохранение.npz
    np.savez_compressed(output_file, 
                        kp_pts=kp_data, 
                        kp_size=kp_size, 
                        kp_angle=kp_angle, 
                        descriptors=descriptors)
    
    print(f"уже сохранил {len(keypoints)} особых точек в файл: {output_file}")

# --- PATH ---
IMAGE_PATH = 'task_map.jpg' 
OUTPUT_FILE = 'map_data.npz'

if __name__ == "__main__":
    save_map_features(IMAGE_PATH, OUTPUT_FILE)
