import numpy as np
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

# === Параметры арены ===
ARENA_SIZE = 3.0
RESOLUTION = 0.05
SIZE = int(ARENA_SIZE / RESOLUTION)  # 60x60

# === Запретная зона ===
FORBIDDEN_ZONE = {
    "x_min": 0.0, "x_max": 0.0,
    "y_min": 0.0, "y_max": 0.0,
}

# === Маркеры и стартовая координата ===
from detect_on_map import found_targets as MARKERS
from detect_on_map import start_coordinate

# === Старт и база ===
START = tuple(start_coordinate)  # защита от мутабельного объекта
BASE = START


def build_grid_array(forbidden_zone: dict, margin: float = 0.1) -> np.ndarray:
    """1 = проходимо, 0 = препятствие."""
    grid = np.ones((SIZE, SIZE), dtype=np.int32)

    x0 = max(0, int((forbidden_zone["x_min"] - margin) / RESOLUTION))
    x1 = min(SIZE, int((forbidden_zone["x_max"] + margin) / RESOLUTION))
    y0 = max(0, int((forbidden_zone["y_min"] - margin) / RESOLUTION))
    y1 = min(SIZE, int((forbidden_zone["y_max"] + margin) / RESOLUTION))

    grid[y0:y1, x0:x1] = 0

    sx, sy = m_to_grid(*START)
    if 0 <= sx < SIZE and 0 <= sy < SIZE:
        if grid[sy, sx] == 0:
            print(f"WARNING: стартовая точка {START} находится внутри запретной зоны!")
        grid[sy, sx] = 1

    return grid


def m_to_grid(x: float, y: float) -> tuple[int, int]:
    """Перевод координат из метров в индексы сетки."""
    gx = max(0, min(SIZE - 1, int(x / RESOLUTION)))
    gy = max(0, min(SIZE - 1, int(y / RESOLUTION)))
    return gx, gy


def grid_to_m(x: int, y: int) -> tuple[float, float]:
    """Перевод координат из сетки обратно в метры."""
    return x * RESOLUTION, y * RESOLUTION


def find_path(grid_array: np.ndarray, start_m: tuple, end_m: tuple) -> list[tuple]:
    """Возвращает путь в метрах или пустой список если путь не найден."""
    grid = Grid(matrix=grid_array)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

    sx, sy = m_to_grid(*start_m)
    ex, ey = m_to_grid(*end_m)

    path, _ = finder.find_path(grid.node(sx, sy), grid.node(ex, ey), grid)
    return [grid_to_m(p.x, p.y) for p in path]


def plan_mission(grid_array: np.ndarray) -> list[dict]:
    """
    Маршрут:
    START --> все allowed маркеры (жадный алгоритм) --> BASE.

    START определяется автоматически в detect_on_map.py.
    forbidden маркеры не являются целями маршрута.
    """
    allowed = [m for m in MARKERS if m["type"] == "allowed"]

    waypoints = []
    current = START
    remaining = allowed.copy()

    while remaining:
        # Жадный выбор ближайшего allowed-маркера (эвклидово расстояние)
        target = min(
            remaining,
            key=lambda m: ((m["x"] - current[0]) ** 2 + (m["y"] - current[1]) ** 2) ** 0.5
        )
        remaining.remove(target)

        path = find_path(grid_array, current, (target["x"], target["y"]))

        if not path:
            print(f"WARNING: путь к маркеру {target['id']} ({target['letter']}) не найден, пропускаем")
            continue

        waypoints.append({
            "action": "fly_to_marker",
            "id": target["id"],
            "letter": target["letter"],
            "type": target["type"],
            "from_m": current,
            "to_m": (target["x"], target["y"]),
            "path": path,
        })

        current = (target["x"], target["y"])

    # Возврат в точку старта
    path = find_path(grid_array, current, BASE)

    if not path:
        print("WARNING: путь к базе не найден!")

    waypoints.append({
        "action": "return_to_base",
        "id": None,
        "letter": None,
        "type": None,
        "from_m": current,
        "to_m": BASE,
        "path": path,
    })

    return waypoints


# === Запуск ===
grid_array = build_grid_array(FORBIDDEN_ZONE, margin=0.1)
waypoints = plan_mission(grid_array)

print("=== ПЛАН МИССИИ ===")
print("Start coordinate:", START)
print("Base coordinate:", BASE)
print("===================")

for i, wp in enumerate(waypoints):
    pts = len(wp["path"])
    print(f"{i + 1}. {wp['action']:20s} → {wp['to_m']}  ({pts} точек пути)")
    if pts == 0:
        print("   WARNING: путь не найден")