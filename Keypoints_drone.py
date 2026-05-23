import cv2
import numpy as np

def extract_and_merge_drone_features(image_paths, output_npz="drone_features.npz"):
    """
    image_paths: 4 изображения дрона [1,2,3,4]
    """
    all_kp_pts = []
    all_descriptors = []
    
    # инициализация ORB
    orb = cv2.ORB_create(nfeatures=10000)

    for path in image_paths:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"error in {path}")
            continue
            
        kp, des = orb.detectAndCompute(img, None)
        
        if des is not None:
            # созранить описание и координаты особых точек
            pts = np.array([p.pt for p in kp])
            all_kp_pts.append(pts)
            all_descriptors.append(des)

    # совмещать все особые точки 4 изображения в 1 матрицу
    if all_kp_pts and all_descriptors:
        combined_kp_pts = np.vstack(all_kp_pts)
        combined_des = np.vstack(all_descriptors)
        
        # сохранить в .npz
        np.savez_compressed(output_npz, kp_pts=combined_kp_pts, descriptors=combined_des)
        print(f"у е получил {output_npz}")
        return output_npz
    else:
        print("error, ")
        return None

# --- PATH for img ---
IMAGE_LIST = [
    '1.jpg',
    '2.jpg',
    '3.jpg',
    '4.jpg'
]

# execute
extract_and_merge_drone_features(IMAGE_LIST)