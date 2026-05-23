import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

# ========== ПАРАМЕТРЫ СРЕДЫ ==========
ARENA_SIZE = 3.0
RESOLUTION = 0.05
SIZE = int(ARENA_SIZE / RESOLUTION)

FORBIDDEN_ZONE = {
    "x_min": 1.0, "x_max": 2.0,
    "y_min": 1.0, "y_max": 2.0,
}

# Маркеры (смешанные: allowed и forbidden)
MARKERS = [
    {"id": 1, "x": 0.5, "y": 0.5, "letter": "A", "type": "allowed"},
    {"id": 2, "x": 2.5, "y": 0.5, "letter": "B", "type": "allowed"},
    {"id": 3, "x": 2.5, "y": 2.5, "letter": "C", "type": "forbidden"},  # запрещённый
    {"id": 4, "x": 0.5, "y": 2.5, "letter": "D", "type": "allowed"},
]

START = (0.0, 0.0)
BASE = START

# ========== A* УТИЛИТЫ ==========
def build_grid_array(forbidden_zone, margin=0.1):
    grid = np.ones((SIZE, SIZE), dtype=np.int32)
    x0 = max(0, int((forbidden_zone["x_min"] - margin) / RESOLUTION))
    x1 = min(SIZE, int((forbidden_zone["x_max"] + margin) / RESOLUTION))
    y0 = max(0, int((forbidden_zone["y_min"] - margin) / RESOLUTION))
    y1 = min(SIZE, int((forbidden_zone["y_max"] + margin) / RESOLUTION))
    grid[y0:y1, x0:x1] = 0
    return grid

def m_to_grid(x, y):
    return int(x / RESOLUTION), int(y / RESOLUTION)

def find_path(grid_array, start_m, end_m):
    grid = Grid(matrix=grid_array)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    sx, sy = m_to_grid(*start_m)
    ex, ey = m_to_grid(*end_m)
    start_node = grid.node(sx, sy)
    end_node = grid.node(ex, ey)
    path, _ = finder.find_path(start_node, end_node, grid)
    return [(p.x * RESOLUTION, p.y * RESOLUTION) for p in path]

def path_length(grid_array, a, b):
    path = find_path(grid_array, a, b)
    if not path:
        return np.inf
    return sum(np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) for i in range(len(path)-1))

# ========== ДЕЦЕНТРАЛИЗОВАННЫЙ ACO РОЙ ==========
class AntDrone:
    def __init__(self, drone_id, start_pos, grid_array, pheromone_map, allowed_markers, alpha=1.0, beta=2.0):
        self.id = drone_id
        self.x, self.y = start_pos
        self.vx, self.vy = 0.0, 0.0
        self.radius = 0.15
        self.active = True
        self.grid_array = grid_array
        self.pheromone = pheromone_map   # {marker_id: концентрация}
        self.allowed_markers = allowed_markers   # список разрешённых маркеров (каждый с ключом "visited")
        self.alpha = alpha
        self.beta = beta
        self.current_target = None
        self.current_path = []
        self.returning = False
        self.base = start_pos

    def distance_to(self, a, b):
        return np.hypot(a[0]-b[0], a[1]-b[1])

    def choose_next_marker(self):
        unvisited = [m for m in self.allowed_markers if not m["visited"]]
        if not unvisited:
            return None
        total = 0.0
        probs = []
        for m in unvisited:
            tau = self.pheromone.get(m["id"], 0.1) ** self.alpha
            dist = self.distance_to((self.x, self.y), (m["x"], m["y"]))
            eta = (1.0 / (dist + 0.01)) ** self.beta
            prob = tau * eta
            probs.append(prob)
            total += prob
        if total == 0:
            return random.choice(unvisited)
        probs = [p / total for p in probs]
        chosen = np.random.choice(unvisited, p=probs)
        return chosen

    def set_target(self, target_marker):
        if target_marker is None:
            return
        self.current_target = target_marker
        target_pos = (target_marker["x"], target_marker["y"])
        self.current_path = find_path(self.grid_array, (self.x, self.y), target_pos)
        if not self.current_path:
            print(f"Дрон {self.id}: нет пути к маркеру {target_marker['id']}")
            return
        self.returning = False

    def set_return_to_base(self):
        self.current_target = None
        self.current_path = find_path(self.grid_array, (self.x, self.y), self.base)
        self.returning = True

    def update_movement(self, other_drones):
        if not self.active:
            return
        fx, fy = 0.0, 0.0
        if self.current_path:
            target_x, target_y = self.current_path[0]
            dx = target_x - self.x
            dy = target_y - self.y
            dist = np.hypot(dx, dy)
            if dist > 0.05:
                k_att = 1.5
                fx += k_att * dx / dist
                fy += k_att * dy / dist
            if dist < 0.1:
                self.current_path.pop(0)
        # Отталкивание от других дронов
        for other in other_drones:
            if other is self or not other.active:
                continue
            dx = self.x - other.x
            dy = self.y - other.y
            dist = np.hypot(dx, dy)
            if dist < 0.5 and dist > 0.01:
                k_rep = 2.0 * (1.0 / dist - 1.0 / 0.5) / (dist * dist)
                fx += k_rep * dx / dist
                fy += k_rep * dy / dist
        # Отталкивание от запретной зоны
        fx, fy = self._avoid_forbidden_zone(fx, fy)
        self.vx += fx * 0.1
        self.vy += fy * 0.1
        speed = np.hypot(self.vx, self.vy)
        max_speed = 0.4
        if speed > max_speed:
            self.vx = self.vx / speed * max_speed
            self.vy = self.vy / speed * max_speed
        self.x += self.vx * 0.05
        self.y += self.vy * 0.05

    def _avoid_forbidden_zone(self, fx, fy):
        zone = FORBIDDEN_ZONE
        margin = 0.2
        if zone["x_min"] - margin < self.x < zone["x_max"] + margin and \
           zone["y_min"] - margin < self.y < zone["y_max"] + margin:
            center_x = (zone["x_min"] + zone["x_max"]) / 2
            center_y = (zone["y_min"] + zone["y_max"]) / 2
            dx = self.x - center_x
            dy = self.y - center_y
            dist = np.hypot(dx, dy)
            if dist > 0:
                fx += 2.0 * dx / dist
                fy += 2.0 * dy / dist
        return fx, fy

    def check_arrival(self):
        if self.returning:
            if np.hypot(self.x - self.base[0], self.y - self.base[1]) < 0.1:
                self.active = False
                return True
            return False
        if self.current_target is not None:
            tx, ty = self.current_target["x"], self.current_target["y"]
            if np.hypot(self.x - tx, self.y - ty) < 0.1:
                if not self.current_target["visited"]:
                    self.current_target["visited"] = True
                    self.pheromone[self.current_target["id"]] = self.pheromone.get(self.current_target["id"], 0.1) + 1.0
                next_marker = self.choose_next_marker()
                if next_marker is None:
                    self.set_return_to_base()
                else:
                    self.set_target(next_marker)
                return True
        return False


class DecentralizedSwarm:
    def __init__(self, n_drones, all_markers, forbidden_zone, start_pos, base_pos):
        self.n_drones = n_drones
        self.grid_array = build_grid_array(forbidden_zone, margin=0.1)
        # Фильтруем только разрешённые маркеры
        self.allowed_markers = [
            {**m, "visited": False} for m in all_markers if m.get("type") == "allowed"
        ]
        # Феромонная карта только для разрешённых маркеров
        self.pheromone = {m["id"]: 0.5 for m in self.allowed_markers}
        self.drones = []
        for i in range(n_drones):
            drone = AntDrone(i+1, start_pos, self.grid_array, self.pheromone, self.allowed_markers)
            first = drone.choose_next_marker()
            if first:
                drone.set_target(first)
            else:
                drone.set_return_to_base()
            self.drones.append(drone)
        self.start_pos = start_pos
        self.base = base_pos

    def update(self):
        for drone in self.drones:
            if drone.active:
                drone.update_movement(self.drones)
                drone.check_arrival()
        all_visited = all(m["visited"] for m in self.allowed_markers)
        all_inactive = all(not d.active for d in self.drones)
        if all_visited and all_inactive:
            print("Миссия завершена: все разрешённые маркеры посещены, все дроны на базе")
            return False
        return True

# ========== ВИЗУАЛИЗАЦИЯ ==========
class Visio:
    def __init__(self, swarm):
        self.swarm = swarm
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-0.5, 3.5)
        self.ax.set_ylim(-0.5, 3.5)
        self.ax.set_title("Децентрализованный рой дронов (только allowed маркеры)")
        self.ax.set_xlabel("X (м)")
        self.ax.set_ylabel("Y (м)")

        self.drone_dots, = self.ax.plot([], [], 'bo', markersize=6, alpha=0.8, label='Дроны')
        self.marker_dots = {}
        # Отображаем только разрешённые маркеры
        for m in swarm.allowed_markers:
            dot, = self.ax.plot(m["x"], m["y"], 'gx', markersize=12)
            self.marker_dots[m["id"]] = dot
            self.ax.text(m["x"]+0.05, m["y"]+0.05, m["letter"], fontsize=10)
        self.base_dot, = self.ax.plot([swarm.base[0]], [swarm.base[1]], 'r*', markersize=15, label='База')
        rect = plt.Rectangle((FORBIDDEN_ZONE["x_min"], FORBIDDEN_ZONE["y_min"]),
                             FORBIDDEN_ZONE["x_max"]-FORBIDDEN_ZONE["x_min"],
                             FORBIDDEN_ZONE["y_max"]-FORBIDDEN_ZONE["y_min"],
                             color='red', alpha=0.3)
        self.ax.add_patch(rect)
        self.ax.legend()
        self.running = True

    def init(self):
        self.drone_dots.set_data([], [])
        return self.drone_dots,

    def animate(self, frame):
        if self.running:
            self.running = self.swarm.update()
        active_x = [d.x for d in self.swarm.drones if d.active]
        active_y = [d.y for d in self.swarm.drones if d.active]
        self.drone_dots.set_data(active_x, active_y)
        for m in self.swarm.allowed_markers:
            if m["visited"]:
                self.marker_dots[m["id"]].set_marker('o')
                self.marker_dots[m["id"]].set_color('gray')
                self.marker_dots[m["id"]].set_markersize(6)
        return self.drone_dots,

    def show(self):
        self.ani = FuncAnimation(self.fig, self.animate, init_func=self.init, frames=2000, interval=50, blit=False)
        plt.show()

# ========== ЗАПУСК ==========
if __name__ == "__main__":
    import copy
    # MARKERS уже содержит запрещённые
    swarm = DecentralizedSwarm(n_drones=3,
                               all_markers=MARKERS,
                               forbidden_zone=FORBIDDEN_ZONE,
                               start_pos=START,
                               base_pos=BASE)
    vis = Visio(swarm)
    vis.show()
