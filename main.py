import cv2
import time
from pioneer_sdk2 import Pioneer, Camera, CameraType

def main():
    pioneer_mini = Pioneer()
    camera = Camera(camera_type=CameraType.MAIN)
    
    # Переменные скоростей (vx, vy, vz, yaw_speed).
    speed = 0.5  # Скорость перемещения (м/с)
    yaw_speed = 30.0  # Скорость вращения (градусов/с)

    # Шаг 1 -- запуск дрона.
    pioneer_mini.arm()
    pioneer_mini.takeoff()

    # Шаг 2 -- подъём дрона на определённую высоту.
    pioneer_mini.go_to_global_point(0, 0, 4)

    # Шаг 3 -- получаем кадр для вывода на экран ПК.
    frame = camera.get_cv_frame()
    cv2.imwrite('map_with_real_markers.jpg', frame)
    
    # Шаг 4 -- получаем карту и генерируем точки.
    from astar import waypoints
    print(waypoints)
    
    # Шаг 5 -- спускаемся обратно.
    pioneer_mini.go_to_global_point(0, 0, 0)

    # Шаг 6 -- посадка дрона.
    pioneer_mini.land()
    pioneer_mini.disarm()

    # Корректное закрытие окон после выхода из цикла
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
