def generate_safe_zigzag_path_mm_to_m(start_x, start_y, area_width, area_length, step_size=100, forbidden_circle):
    """
    Генерирует относительные команды (dx, dy).
    Возвращаемый результат (dx, dy) будет в МЕТРАХ (м) для команд дрона.
    
    forbidden_circle: (center_x, center_y, radius) - Координаты центра и радиус в мм.
    """
    # Распаковка параметров круга (теперь с радиусом r)
    cx, cy = forbidden_circle  #(координат центра запретных зон)
    r_squared = 300 ** 2 
    
    # Расчет количества шагов
    steps_x = int(area_length / step_size)
    steps_y = int(area_width / step_size)
    
    valid_absolute_points = []
    direction_y = 1
    
    # 1. Сначала генерируем все абсолютные координаты (X, Y) в миллиметрах
    for i in range(steps_x):
        x_val = start_x + i * step_size
        
        # Меняем порядок прохода по Y для создания зигзага
        y_steps = range(steps_y) if direction_y == 1 else reversed(range(steps_y))
        
        for j in y_steps:
            y_val = start_y + j * step_size
            
            # ПРОВЕРКА КРУГЛОЙ ЗАПРЕТНОЙ ЗОНЫ (в мм)
            if ((x_val - cx) ** 2 + (y_val - cy) ** 2) <= r_squared:
                print(f"Пропуск точки: X={x_val:.1f}, Y={y_val:.1f} мм (Внутри круга)")
            else:
                valid_absolute_points.append((x_val, y_val))
                
        direction_y *= -1
        
    # 2. Вычисляем относительные команды (dx, dy) и переводим их в МЕТРЫ
    path_relative_meters = []
    current_x, current_y = start_x, start_y
    
    for px, py in valid_absolute_points:
        dx_mm = px - current_x
        dy_mm = py - current_y
        
        # Пропускаем команду, если дрон уже находится в нужной точке
        if dx_mm == 0 and dy_mm == 0:
            continue
            
        # ПЕРЕВОД В МЕТРЫ: Делим миллиметры на 1000
        dx_m = dx_mm / 1000.0
        dy_m = dy_mm / 1000.0
        
        path_relative_meters.append((dx_m, dy_m))
        
        # Обновляем виртуальную позицию
        current_x, current_y = px, py
        
    return path_relative_meters
