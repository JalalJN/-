import cv2
import numpy as np
import os

def save_map_features(image_path, output_file="map_features.npz"):
    # 1. Đọc ảnh bản đồ
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Lỗi: Không tìm thấy ảnh tại đường dẫn đã cho.")
        return

    # 2. Khởi tạo ORB detector
    # nfeatures=10000 giúp trích xuất đủ số lượng điểm đặc trưng cho bản đồ lớn
    orb = cv2.ORB_create(nfeatures=10000)

    # 3. Trích xuất Keypoints và Descriptors
    print("Đang trích xuất đặc trưng... Vui lòng đợi.")
    keypoints, descriptors = orb.detectAndCompute(img, None)

    # 4. Chuyển đổi Keypoints thành định dạng có thể lưu được
    # Keypoints là object, cần tách ra thành toạ độ (x, y) và các thông số khác
    kp_data = np.array([kp.pt for kp in keypoints])
    kp_size = np.array([kp.size for kp in keypoints])
    kp_angle = np.array([kp.angle for kp in keypoints])

    # 5. Lưu vào file .npz
    np.savez_compressed(output_file, 
                        kp_pts=kp_data, 
                        kp_size=kp_size, 
                        kp_angle=kp_angle, 
                        descriptors=descriptors)
    
    print(f"Đã lưu xong {len(keypoints)} điểm đặc trưng vào file: {output_file}")

# --- CẤU HÌNH ĐƯỜNG DẪN TẠI ĐÂY ---
IMAGE_PATH = 'task_map.jpg' 
OUTPUT_FILE = 'map_data.npz'

if __name__ == "__main__":
    save_map_features(IMAGE_PATH, OUTPUT_FILE)