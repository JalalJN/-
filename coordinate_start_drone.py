import cv2
import numpy as np

def get_drone_position_rectified(drone_npz_path, map_npz_path, image_path ,camera_angle_deg=-80):
    # 1. загрузка data
    drone_data = np.load(drone_npz_path)
    map_data = np.load(map_npz_path)
    
    drone_kp_pts = drone_data['kp_pts']
    drone_des = drone_data['descriptors']
    map_kp_pts = map_data['kp_pts']
    map_des = map_data['descriptors']
    
    # 2. Matcher with FLANN
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    matches = flann.knnMatch(drone_des, map_des, k=2)
    
    # (Lowe's ratio test)
    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)
            
    if len(good_matches) < 20:
        print("мало точек для локализации!")
        return None, None

    # 3. найти Homography
    src_pts = np.float32([drone_kp_pts[m.queryIdx] for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([map_kp_pts[m.trainIdx] for m in good_matches]).reshape(-1, 1, 2)
    
    # фильтрация RANSAC
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    if H is not None:
        # 4. найти центр drone
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        h, w = img.shape[:2]
        center_drone = np.array([[[w/2, h/2]]], dtype=np.float32)
        
        # преобразование центра из изображения в дроне
        center_map = cv2.perspectiveTransform(center_drone, H)
        
        drone_x = center_map[0][0][0]
        drone_y = center_map[0][0][1]
        
        return drone_x, drone_y
    
    return None, None

# --- PATH .npz ---
DRONE_NPZ = 'drone_features.npz'
MAP_NPZ = 'map_data.npz'

# --- PATH img from drone ---
IMG_PATH = '1.png'
# Gọi hàm
x, y = get_drone_position_rectified(DRONE_NPZ, MAP_NPZ, IMG_PATH)
if x:
    print(f"координат дрона сейчас: X={x:.2f}, Y={y:.2f}")