import pytest
from flight_path import FlightPathCalculator, Point3D, Obstacle


class TestPoint3D:
    def test_distance(self):
        p1 = Point3D(0, 0, 0)
        p2 = Point3D(3, 4, 0)
        assert p1.distance(p2) == 5.0
    
    def test_equality(self):
        p1 = Point3D(1.0, 2.0, 3.0)
        p2 = Point3D(1.0, 2.0, 3.0)
        p3 = Point3D(1.1, 2.0, 3.0)
        assert p1 == p2
        assert p1 != p3
    
    def test_to_dict_from_dict(self):
        original = Point3D(1.5, 2.5, 3.5)
        data = original.to_dict()
        restored = Point3D.from_dict(data)
        assert original == restored


class TestObstacle:
    def test_contains(self):
        obs = Obstacle(0, 0, 0, 10)
        assert obs.contains(Point3D(5, 0, 0))
        assert not obs.contains(Point3D(15, 0, 0))


class TestFlightPathCalculator:
    def test_simple_2d_path(self):
        calc = FlightPathCalculator()
        path = calc.calculate_path_astar(
            Point3D(0, 0, 500),
            Point3D(500, 500, 500),
            use_3d=False
        )
        assert path is not None
        assert len(path) > 1
        assert path[0] == Point3D(0, 0, 500)
    
    def test_path_with_obstacles(self):
        calc = FlightPathCalculator()
        calc.add_obstacle(Obstacle(500, 500, 500, 200))
        
        path = calc.calculate_path_astar(
            Point3D(0, 0, 500),
            Point3D(1000, 1000, 500),
            use_3d=False
        )
        assert path is not None
    
    def test_unreachable_path(self):
        calc = FlightPathCalculator()
        calc.add_obstacle(Obstacle(500, 500, 500, 10000))
        
        path = calc.calculate_path_astar(
            Point3D(0, 0, 500),
            Point3D(1000, 1000, 500),
            use_3d=False
        )
        assert path is None
    
    def test_rrt_path(self):
        calc = FlightPathCalculator()
        path = calc.calculate_path_rrt(
            Point3D(0, 0, 500),
            Point3D(1000, 1000, 500),
            max_iterations=1000,
            step_size=100
        )
        assert path is not None
        assert path[0] == Point3D(0, 0, 500)
    
    def test_path_length(self):
        path = [
            Point3D(0, 0, 0),
            Point3D(3, 0, 0),
            Point3D(3, 4, 0)
        ]
        calc = FlightPathCalculator()
        length = calc.calculate_path_length(path)
        assert length == 7.0
    
    def test_altitude_constraints(self):
        calc = FlightPathCalculator(min_altitude=100, max_altitude=500)
        
        assert calc.is_valid_point(Point3D(0, 0, 300))
        assert not calc.is_valid_point(Point3D(0, 0, 50))
        assert not calc.is_valid_point(Point3D(0, 0, 600))
    
    def test_clear_obstacles(self):
        calc = FlightPathCalculator()
        calc.add_obstacle(Obstacle(500, 500, 500, 200))
        
        assert calc.calculate_path_astar(
            Point3D(0, 0, 500),
            Point3D(1000, 1000, 500),
            use_3d=False
        ) is None
        
        calc.clear_obstacles()
        
        assert calc.calculate_path_astar(
            Point3D(0, 0, 500),
            Point3D(1000, 1000, 500),
            use_3d=False
        ) is not None
    
    def test_smooth_path(self):
        calc = FlightPathCalculator()
        path = [
            Point3D(0, 0, 0),
            Point3D(100, 0, 0),
            Point3D(200, 100, 0),
            Point3D(300, 200, 0),
            Point3D(400, 300, 0)
        ]
        smoothed = calc.smooth_path(path)
        assert len(smoothed) <= len(path)
        assert smoothed[0] == path[0]
        assert smoothed[-1] == path[-1]
