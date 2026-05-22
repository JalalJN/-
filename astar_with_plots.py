import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

# === Параметры арены ===
ARENA_SIZE = 3.0
RESOLUTION = 0.05
SIZE = int(ARENA_SIZE / RESOLUTION)  # 60x60

# === Запретная зона ===
FORBIDDEN_ZONE = {
    "x_min": 1.0, "x_max": 2.0,
    "y_min": 1.0, "y_max": 2.0,
}

# === Маркеры ===
from detect_on_map import found_targets as MARKERS

START = (0.1, 0.1)
BASE  = START

def build_grid_array(forbidden_zone: dict, margin: float = 0.1) -> np.ndarray:
    """1 = проходимо, 0 = препятствие."""
    grid = np.ones((SIZE, SIZE), dtype=np.int32)

    x0 = max(0, int((forbidden_zone["x_min"] - margin) / RESOLUTION))
    x1 = min(SIZE, int((forbidden_zone["x_max"] + margin) / RESOLUTION))
    y0 = max(0, int((forbidden_zone["y_min"] - margin) / RESOLUTION))
    y1 = min(SIZE, int((forbidden_zone["y_max"] + margin) / RESOLUTION))

    grid[y0:y1, x0:x1] = 0
    return grid


def m_to_grid(x: float, y: float) -> tuple[int, int]:
    return int(x / RESOLUTION), int(y / RESOLUTION)

def find_path(grid_array: np.ndarray, start_m: tuple, end_m: tuple) -> list[tuple]:
    """Возвращает путь в метрах."""
    # Grid нужно пересоздавать каждый раз — pathfinding модифицирует его внутри
    grid = Grid(matrix=grid_array)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

    sx, sy = m_to_grid(*start_m)
    ex, ey = m_to_grid(*end_m)

    start_node = grid.node(sx, sy)
    end_node   = grid.node(ex, ey)

    path, _ = finder.find_path(start_node, end_node, grid)

    # Конвертируем обратно в метры
    return [(p.x * RESOLUTION, p.y * RESOLUTION) for p in path]

def plan_mission(grid_array: np.ndarray) -> list[dict]:
    """
    Маршрут:
    START → все allowed маркеры (жадный порядок) → BASE
    Запрещённые маркеры игнорируются при планировании пути
    (дрон их не объезжает специально, просто не летит к ним)
    """
    allowed = [m for m in MARKERS if m["type"] == "allowed"]

    waypoints = []
    current = START

    # Жадный порядок — всегда ближайший следующий
    remaining = allowed.copy()
    while remaining:
        remaining.sort(key=lambda m: abs(m["x"] - current[0]) + abs(m["y"] - current[1]))
        target = remaining.pop(0)

        path = find_path(grid_array, current, (target["x"], target["y"]))
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

    # Финальный путь на базу
    path = find_path(grid_array, current, BASE)
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


def visualize(grid_array: np.ndarray, waypoints: list[dict]):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Запретная зона
    fz = FORBIDDEN_ZONE
    ax.add_patch(patches.Rectangle(
        (fz["x_min"], fz["y_min"]),
        fz["x_max"] - fz["x_min"],
        fz["y_max"] - fz["y_min"],
        linewidth=2, edgecolor="red", facecolor="red", alpha=0.25, label="Запретная зона"
    ))

    # Пути
    colors = ["royalblue", "darkorange", "green", "purple"]
    for i, wp in enumerate(waypoints):
        pts = wp["path"]
        if len(pts) > 1:
            xs, ys = zip(*pts)
            label = f'{wp["action"]} (ID={wp["id"]})' if wp["id"] else "return_to_base"
            ax.plot(xs, ys, color=colors[i % len(colors)], linewidth=2, label=label)

    # Маркеры
    for m in MARKERS:
        color = "green" if m["type"] == "allowed" else "red"
        marker_shape = "^" if m["type"] == "allowed" else "x"
        ax.plot(m["x"], m["y"], marker_shape, color=color, markersize=12, markeredgewidth=2)
        ax.annotate(
            f'ID{m["id"]} "{m["letter"]}" ({m["type"]})',
            (m["x"], m["y"]), textcoords="offset points",
            xytext=(6, 6), fontsize=8,
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7)
        )

    # Старт и база
    ax.plot(*START, "o", color="blue",   markersize=14, label="Старт")
    ax.plot(*BASE,  "D", color="purple", markersize=14, label="База")
    ax.annotate("СТАРТ", START, textcoords="offset points", xytext=(6, 6), fontsize=9, color="blue")
    ax.annotate("БАЗА",  BASE,  textcoords="offset points", xytext=(6, 6), fontsize=9, color="purple")

    ax.set_xlim(0, ARENA_SIZE)
    ax.set_ylim(0, ARENA_SIZE)
    ax.set_xlabel("X, м")
    ax.set_ylabel("Y, м")
    ax.set_title("Маршрут дрона — A* (pathfinding)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("mission_path.png", dpi=150)
    plt.show()


# === Запуск ===
grid_array = build_grid_array(FORBIDDEN_ZONE, margin=0.1)
waypoints  = plan_mission(grid_array)

print("=== ПЛАН МИССИИ ===")
for i, wp in enumerate(waypoints):
    pts = len(wp["path"])
    print(f"{i+1}. {wp['action']:20s} → {wp['to_m']}  ({pts} точек пути)")

visualize(grid_array, waypoints)