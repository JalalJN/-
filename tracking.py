def generate_safe_zigzag_path(start_x, start_y, area_width, area_length, step_size, forbidden_zone):
    """
    Генерирует безопасную зигзагообразную траекторию с учетом запретной зоны.
    forbidden_zone: (x_min, x_max, y_min, y_max) - Координаты границ запретной зоны в метрах.
    """
    path = []
    
    # Количество шагов по каждой оси
    steps_y = int(area_width / step_size)
    steps_x = int(area_length / step_size)
    
    direction_y = 1 
    
    # Переменные для отслеживания виртуальных координат (X, Y) дрона во время расчета
    current_x = start_x
    current_y = start_y

    for i in range(steps_x):
        for _ in range(steps_y):
            dx = 0.0
            dy = step_size * direction_y
            
            # Предварительный расчет будущих координат при выполнении этого шага
            next_x = current_x + dx
            next_y = current_y + dy
            
            # ПРОВЕРКА ЗАПРЕТНОЙ ЗОНЫ
            # Если будущие координаты находятся ВНУТРИ запретной зоны -> Пропускаем, не сохраняем в path
            if (forbidden_zone[0] <= next_x <= forbidden_zone[1]) and \
               (forbidden_zone[2] <= next_y <= forbidden_zone[3]):
                print(f"Пропуск точки ({next_x:.2f}, {next_y:.2f}), так как она находится в запретной зоне!")
            else:
                # Если безопасно, сохраняем команду dx, dy в список и обновляем виртуальные координаты
                path.append((dx, dy))
                current_x = next_x
                current_y = next_y
                
        # Шаг на новую линию (ось X)
        if i < steps_x - 1:
            dx = step_size
            dy = 0.0
            next_x = current_x + dx
            next_y = current_y + dy
            
            # Проверка запретной зоны при переходе на новую линию
            if not ((forbidden_zone[0] <= next_x <= forbidden_zone[1]) and \
                    (forbidden_zone[2] <= next_y <= forbidden_zone[3])):
                path.append((dx, dy))
                current_x = next_x
                current_y = next_y
                
        # Смена направления для следующей линии (зигзаг)
        direction_y *= -1
        
    return path