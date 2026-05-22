import cv2
import time
from pioneer_sdk2 import Pioneer, Camera, CameraType, ServoCamera, ServoPriority, Event

def main():
    pioneer_mini = Pioneer()
    camera = Camera(camera_type=CameraType.MAIN)
    servo_camera = ServoCamera()
    finished = False

    # Шаг 1 — запуск дрона
    pioneer_mini.arm()
    pioneer_mini.takeoff()

    # Шаг 2 — подъём на высоту 3м
    pioneer_mini.go_to_local_point(0, 0, 2.5, 1)

    # Поворот камеры вниз
    servo_camera.set_angle(-80, ServoPriority.HIGH)

    # Шаг 3 — колбэк при достижении точки подъёма: фото + маршрут + спуск
    def on_altitude_reached():
        frame = camera.get_cv_frame()
        cv2.imwrite('map_with_real_markers.jpg', frame)

        # Шаг 5 — спуск обратно
        pioneer_mini.go_to_local_point(0, 0, -2.5, 1)

        from astar import waypoints
        print(waypoints)

    # Шаг 4 — колбэк при достижении точки спуска: посадка
    def on_descent_reached_event(event):
        pioneer_mini.land()
        pioneer_mini.disarm()
        finished = True

    # Подписываемся: сначала на подъём, потом переподпишемся на спуск
    def on_point_reached_event(event):
        # Отписываемся, чтобы не сработало повторно
        pioneer_mini.unsubscribe(Event.POINT_REACHED)

        if not descent_started[0]:
            descent_started[0] = True
            on_altitude_reached()
            # Теперь ждём точку спуска
            pioneer_mini.subscribe(on_descent_reached_event, Event.POINT_REACHED)
        else:
            on_descent_reached_event(0)

    descent_started = [False]  # mutable flag для вложенной функции
    pioneer_mini.subscribe(on_point_reached_event, Event.POINT_REACHED)

    while not finished:
        time.sleep(10)

if __name__ == '__main__':
    main()