import cv2
import threading
from pioneer_sdk2 import Pioneer, Camera, CameraType, ServoCamera, ServoPriority
import pioneer_sdk2

def wait_for_point(pioneer, event):
    pioneer.subscribe(lambda _: event.set(), pioneer_sdk2.Event.POINT_REACHED)
    event.wait()
    event.clear()

def main():
    pioneer_mini = Pioneer()
    camera = Camera(camera_type=CameraType.MAIN)
    servo_camera = ServoCamera()
    point_event = threading.Event()

    # Запуск дрона.
    pioneer_mini.arm()
    pioneer_mini.takeoff()

    # Подъём на высоту 3м.
    pioneer_mini.go_to_local_point(0, 0, 2.5, 1)

    # Поворот камеры вниз.
    servo_camera.set_angle(-80, ServoPriority.HIGH)

    # Дожидаемся, пока дрон поднимется на высоту.
    wait_for_point(pioneer_mini, point_event)

    # Получаем фотографию с камеры.
    frame = camera.get_cv_frame()
    cv2.imwrite('map_with_real_markers.jpg', frame)

    # Построение маршрута.
    from astar import waypoints
    print(waypoints)

    # Спуск обратно.
    pioneer_mini.go_to_local_point(0, 0, -2.5, 1)
    wait_for_point(pioneer_mini, point_event)

    # Посадка дрона.
    pioneer_mini.land()
    pioneer_mini.disarm()

if __name__ == '__main__':
    main()
