import math

def route_safely(current_x, current_y, target_x, target_y, circle, step_size):
    """
    Система навигации: Поиск пути от текущей точки к целевой.
    Если на пути препятствие (круг), дрон автоматически огибает его по краю.
    """
    path = []
    cx, cy, r = circle
    # Буферная зона немного больше радиуса, чтобы дрон не задевал края
    safe_r = r + step_size * 0.8
    safe_r_squared = safe_r ** 2
    
    x, y = current_x, current_y
    max_steps = 2000
    steps = 0
    
    while steps < max_steps:
        vx = target_x - x
        vy = target_y - y
        dist = math.hypot(vx, vy)
        
        # Если дрон очень близко к цели, прыжок прямо к ней и завершение
        if dist <= step_size * 1.01:
            path.append((vx, vy))
            x, y = target_x, target_y
            break
            
        steps += 1
        
        # Идеальное направление по прямой
        idx = (vx / dist) * step_size
        idy = (vy / dist) * step_size
        nx = x + idx
        ny = y + idy
        
        # ПРОВЕРКА СТОЛКНОВЕНИЯ
        if ((nx - cx)**2 + (ny - cy)**2) <= safe_r_squared:
            # Путь заблокирован -> Переход в режим скольжения по касательной
            vec_cx = x - cx
            vec_cy = y - cy
            ndist = math.hypot(vec_cx, vec_cy)
            if ndist == 0: ndist = 0.001
            vec_cx /= ndist
            vec_cy /= ndist
            
            # Поиск двух касательных векторов окружности
            tx1, ty1 = -vec_cy, vec_cx
            tx2, ty2 = vec_cy, -vec_cx
            
            # Выбор касательной, которая приближает дрон к цели (vx, vy)
            if (tx1 * vx + ty1 * vy) > (tx2 * vx + ty2 * vy):
                dx, dy = tx1 * step_size, ty1 * step_size
            else:
                dx, dy = tx2 * step_size, ty2 * step_size
                
            # Небольшое отталкивание от центра для плавности скольжения
            dx += vec_cx * (step_size * 0.3)
            dy += vec_cy * (step_size * 0.3)
            
            # Нормализация длины шага
            s_dist = math.hypot(dx, dy)
            dx = (dx / s_dist) * step_size
            dy = (dy / s_dist) * step_size
        else:
            # Путь свободен -> Движение по прямой
            dx, dy = idx, idy
            
        path.append((dx, dy))
        x += dx
        y += dy
        
    return path, x, y

def generate_safe_zigzag_path(start_x, start_y, area_width, area_length, step_size, forbidden_circle):
    """
    Генерирует траекторию полного покрытия сабана (карты) с автоматическим обходом запретных зон.
    """
    path = []
    cx, cy, r = forbidden_circle
    
    # 1. СОЗДАНИЕ ИДЕАЛЬНОГО СПИСКА ТОЧЕК ДЛЯ ПОКРЫТИЯ
    ideal_points = []
    steps_x = int(area_length / step_size)
    steps_y = int(area_width / step_size)
    direction_y = 1
    
    for i in range(steps_x):
        # Движение вдоль оси Y (смена направления вниз/вверх для зигзага)
        y_range = range(steps_y) if direction_y == 1 else reversed(range(steps_y))
        for j in y_range:
            px = i * step_size
            py = j * step_size
            
            # СОХРАНЕНИЕ только безопасных точек (вне красного круга)
            if (px - cx)**2 + (py - cy)**2 > (r + step_size)**2:
                ideal_points.append((px, py))
                
        direction_y *= -1
        
    # 2. ПОСЛЕДОВАТЕЛЬНОЕ ДВИЖЕНИЕ ПО ТОЧКАМ
    current_x = start_x
    current_y = start_y
    
    for target_x, target_y in ideal_points:
        # Вызов системы безопасной навигации от текущей точки к следующей целевой
        segment, current_x, current_y = route_safely(
            current_x, current_y, target_x, target_y, forbidden_circle, step_size
        )
        path.extend(segment)
        
    return path
