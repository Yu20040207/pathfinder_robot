"""Conservative occupancy-grid path planning without motor commands."""

from dataclasses import dataclass
import heapq
import math


class PlanningError(ValueError):
    """A requested route cannot be planned on the supplied map."""


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or self.width * self.height != len(self.cells):
            raise ValueError("Invalid map dimensions")
        if self.width * self.height > 1_000_000:
            raise ValueError("Map too large for preview planner")
        if not all(math.isfinite(v) for v in (self.resolution, self.origin_x,
                                               self.origin_y, self.origin_yaw)) or self.resolution <= 0:
            raise ValueError("Invalid map geometry")

    def inside(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        return self.cells[y * self.width + x]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((c * dx + s * dy) / self.resolution),
                math.floor((-s * dx + c * dy) / self.resolution))

    def cell_center(self, x: int, y: int) -> tuple[float, float]:
        lx, ly = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly)


def _blocked(board: GridMap, clearance_m: float, threshold: int) -> set[tuple[int, int]]:
    if not math.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError("Invalid clearance")
    if not 1 <= threshold <= 100:
        raise ValueError("Invalid occupancy threshold")
    obstacles = {(x, y) for y in range(board.height) for x in range(board.width)
                 if board.value(x, y) < 0 or board.value(x, y) >= threshold}
    if clearance_m == 0:
        return obstacles
    radius_cells = math.ceil(clearance_m / board.resolution + 0.5)
    blocked = set(obstacles)
    for ox, oy in obstacles:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                edge_distance = math.hypot(max(abs(dx) - 0.5, 0),
                                           max(abs(dy) - 0.5, 0)) * board.resolution
                if edge_distance <= clearance_m:
                    x, y = ox + dx, oy + dy
                    if board.inside(x, y):
                        blocked.add((x, y))
    for y in range(board.height):
        for x in range(board.width):
            border_distance = min(x + 0.5, y + 0.5,
                                  board.width - x - 0.5,
                                  board.height - y - 0.5) * board.resolution
            if border_distance <= clearance_m:
                blocked.add((x, y))
    return blocked


def plan_path(board: GridMap, start_xy: tuple[float, float], goal_xy: tuple[float, float],
              *, clearance_m: float = 0.2, occupancy_threshold: int = 50) -> list[tuple[int, int]]:
    """Return a cell route using eight-neighbor A*; unknown cells are obstacles."""
    if not all(math.isfinite(v) for v in (*start_xy, *goal_xy)):
        raise PlanningError("Non-finite start or goal")
    start, goal = board.world_to_cell(*start_xy), board.world_to_cell(*goal_xy)
    if not board.inside(*start) or not board.inside(*goal):
        raise PlanningError("start or goal outside map")
    blocked = _blocked(board, clearance_m, occupancy_threshold)
    if start in blocked or goal in blocked:
        raise PlanningError("start or goal blocked")
    if start == goal:
        return [start]

    def heuristic(cell):
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    frontier = [(heuristic(start), 0.0, start)]
    best = {start: 0.0}
    parent = {}
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1),
                  (1, 1), (1, -1), (-1, 1), (-1, -1))
    while frontier:
        _, cost, cell = heapq.heappop(frontier)
        if cost > best[cell]:
            continue
        if cell == goal:
            route = [cell]
            while route[-1] != start:
                route.append(parent[route[-1]])
            route.reverse()
            return route
        for dx, dy in directions:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if not board.inside(*neighbor) or neighbor in blocked:
                continue
            if dx and dy and ((cell[0] + dx, cell[1]) in blocked or
                              (cell[0], cell[1] + dy) in blocked):
                continue
            new_cost = cost + math.hypot(dx, dy)
            if new_cost < best.get(neighbor, math.inf):
                best[neighbor] = new_cost
                parent[neighbor] = cell
                heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))
    raise PlanningError("no path through free cells")
