from dataclasses import dataclass
from typing import List, Optional, Tuple
import heapq
import random
import math


@dataclass
class Point3D:
    x: float
    y: float
    z: float
    
    def distance(self, other: 'Point3D') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Point3D):
            return False
        return (abs(self.x - other.x) < 1e-9 and
                abs(self.y - other.y) < 1e-9 and
                abs(self.z - other.z) < 1e-9)
    
    def __hash__(self) -> int:
        return hash((round(self.x, 1), round(self.y, 1), round(self.z, 1)))
    
    def __repr__(self) -> str:
        return f"Point3D({self.x:.1f}, {self.y:.1f}, {self.z:.1f})"
    
    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "z": self.z}
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Point3D':
        return cls(data["x"], data["y"], data["z"])


@dataclass
class Obstacle:
    x: float
    y: float
    z: float
    radius: float
    
    def contains(self, point: Point3D) -> bool:
        return Point3D(self.x, self.y, self.z).distance(point) < self.radius
    
    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "z": self.z, "radius": self.radius}
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Obstacle':
        return cls(data["x"], data["y"], data["z"], data["radius"])


class FlightPathCalculator:
    
    def __init__(
        self,
        min_altitude: float = 100.0,
        max_altitude: float = 10000.0,
        min_turn_radius: float = 500.0,
        grid_resolution: float = 100.0
    ):
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.min_turn_radius = min_turn_radius
        self.grid_resolution = grid_resolution
        self.obstacles: List[Obstacle] = []
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        self.obstacles.append(obstacle)
    
    def clear_obstacles(self) -> None:
        self.obstacles.clear()
    
    def is_valid_point(self, point: Point3D) -> bool:
        if point.z < self.min_altitude or point.z > self.max_altitude:
            return False
        for obs in self.obstacles:
            if obs.contains(point):
                return False
        return True
    
    def _heuristic(self, a: Point3D, b: Point3D) -> float:
        return a.distance(b)
    
    def _snap_to_grid(self, point: Point3D) -> Point3D:
        return Point3D(
            round(point.x / self.grid_resolution) * self.grid_resolution,
            round(point.y / self.grid_resolution) * self.grid_resolution,
            round(point.z / self.grid_resolution) * self.grid_resolution
        )
    
    def calculate_path_astar(
        self,
        start: Point3D,
        goal: Point3D,
        use_3d: bool = True
    ) -> Optional[List[Point3D]]:
        if not self.is_valid_point(start) or not self.is_valid_point(goal):
            return None
        
        open_set: List[Tuple[float, int, Point3D]] = []
        counter = 0
        
        heapq.heappush(open_set, (0, counter, start))
        
        came_from: dict = {}
        g_score: dict = {start: 0}
        
        directions = [
            (-self.grid_resolution, 0, 0),
            (self.grid_resolution, 0, 0),
            (0, -self.grid_resolution, 0),
            (0, self.grid_resolution, 0),
            (0, 0, -self.grid_resolution if use_3d else 0),
            (0, 0, self.grid_resolution if use_3d else 0),
        ]
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            if current.distance(goal) < self.grid_resolution:
                path = [current]
                node = current
                while node in came_from:
                    node = came_from[node]
                    path.append(node)
                path.reverse()
                return path
            
            for dx, dy, dz in directions:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighbor = Point3D(current.x + dx, current.y + dy, current.z + dz)
                
                if not self.is_valid_point(neighbor):
                    continue
                
                tentative_g = g_score[current] + current.distance(neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))
        
        return None
    
    def calculate_path_rrt(
        self,
        start: Point3D,
        goal: Point3D,
        max_iterations: int = 10000,
        step_size: float = 1.0
    ) -> Optional[List[Point3D]]:
        if not self.is_valid_point(start) or not self.is_valid_point(goal):
            return None
        
        nodes = [start]
        came_from: dict = {start: None}
        
        for _ in range(max_iterations):
            if random.random() < 0.1:
                random_point = goal
            else:
                random_point = Point3D(
                    random.uniform(0, 10000),
                    random.uniform(0, 10000),
                    random.uniform(self.min_altitude, self.max_altitude)
                )
            
            nearest = min(nodes, key=lambda n: n.distance(random_point))
            
            dx = random_point.x - nearest.x
            dy = random_point.y - nearest.y
            dz = random_point.z - nearest.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < step_size:
                continue
            
            ratio = step_size / dist
            new_point = Point3D(
                nearest.x + dx * ratio,
                nearest.y + dy * ratio,
                nearest.z + dz * ratio
            )
            
            if not self.is_valid_point(new_point):
                continue
            
            nodes.append(new_point)
            came_from[new_point] = nearest
            
            if new_point.distance(goal) < step_size:
                came_from[goal] = new_point
                break
        
        if goal not in came_from:
            return None
        
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        
        return path
    
    def smooth_path(self, path: List[Point3D]) -> List[Point3D]:
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            furthest = current_idx + 1
            
            for i in range(len(path) - 1, current_idx + 1, -1):
                can_reach = True
                
                for j in range(current_idx + 1, i):
                    if not self.is_valid_point(path[j]):
                        can_reach = False
                        break
                    
                    for obs in self.obstacles:
                        if obs.contains(path[j]):
                            can_reach = False
                            break
                    
                    if not can_reach:
                        break
                
                if can_reach:
                    furthest = i
                    break
            
            smoothed.append(path[furthest])
            current_idx = furthest
        
        return smoothed
    
    def calculate_path_length(self, path: List[Point3D]) -> float:
        return sum(path[i-1].distance(path[i]) for i in range(1, len(path)))
    
    def to_dict(self) -> dict:
        return {
            "min_altitude": self.min_altitude,
            "max_altitude": self.max_altitude,
            "min_turn_radius": self.min_turn_radius,
            "grid_resolution": self.grid_resolution,
            "obstacles": [o.to_dict() for o in self.obstacles]
        }
