import cv2
import threading
from pioneer_sdk2 import Pioneer, Camera, CameraType, ServoCamera, ServoPriority
import pioneer_sdk2

def wait_for_point(pioneer, event):
    pioneer.subscribe(lambda _: event.set(), pioneer_sdk2.Event.POINT_REACHED)
    event.wait()
    event.clear()

def pr_drone(pioneer_mini, camera, servo_camera, point_event):
     # Запуск дрона.
    pioneer_mini.arm()
    pioneer_mini.takeoff()

    # Подъём на высоту 3м.
    pioneer_mini.go_to_local_point(0.0, 0.0, 1.5, 0.0, 1)

    # Поворот камеры вниз.
    servo_camera.set_angle(-80, ServoPriority.HIGH)

    yaw_point = [0.0, 90.0, 180.0, 270.0]
    # Дожидаемся, пока дрон поднимется на высоту.
    wait_for_point(pioneer_mini, point_event)

    # Получаем фотографию с камеры.
    for i in range(4):
        frame = camera.get_cv_frame(1.0)
        cv2.imwrite(f'{i + 1}.jpg', frame)

        pioneer_mini.go_to_local_point_body_fixed(0.0, 0.0, 0.0, 90, 2)
        wait_for_point(pioneer_mini, point_event)

    # Определяем и сохраняем ключевые точки на карте в map_features.npz
    from Keypoints import save_map_features
    save_map_features('task_map.jpg')
    #
    # Это делается заранее, до запуска дрона.

    # Определяем ключевые точки дрона.
    # Сохраняем их в drone_features.npz
    from Keypoints_drone import extract_and_merge_drone_features
    extract_and_merge_drone_features(['1.jpg', '2.jpg', '3.jpg', '4.jpg'])

    # Определяем нынешние координаты дрона.
    from coordinate_start_drone import get_drone_position_rectified
    x, y = get_drone_position_rectified('drone_features.npz', 'map_data.npz', 'task_map.jpg')
    print(f"Координаты дрона сейчас: X={x:.2f}, Y={y:.2f}")

    # Построение маршрута.
    # from astar import waypoints
    # print(waypoints)

    # Спуск обратно.
    # pioneer_mini.go_to_local_point(0, 0, -2.5, 0, 1)  
    # wait_for_point(pioneer_mini, point_event)
    return x, y


def main():
    pioneer_mini = Pioneer()
    camera = Camera(camera_type=CameraType.MAIN)
    servo_camera = ServoCamera()
    point_event = threading.Event()

    # получение координаты дрона.
    x_start, y_start = pr_drone(pioneer_mini, camera, servo_camera, point_event)
    x_start = float(x_start/1000)
    y_start = float(y_start/1000)

    # запретные зоны.
    circlef = (1.5, 1.5, 1) 
    
    from tracking import generate_safe_zigzag_path

    track = generate_safe_zigzag_path(
    start_x=x_start, 
    start_y=y_start, 
    area_width=3.0, 
    area_length=3.0, 
    step_size=0.5, 
    forbidden_circle=circlef
)
    for i in range(len(track)):
        pioneer_mini.go_to_local_point_body_fixed(track[i][0], track[i][1], 0.0, 2)
        ...

    # Посадка дрона.
    pioneer_mini.land()
    pioneer_mini.disarm()

if __name__ == '__main__':
    main()
