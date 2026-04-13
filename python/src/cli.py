import argparse
import json
import sys
from pathlib import Path

from flight_path import FlightPathCalculator, Point3D, Obstacle


def load_from_json(filepath: str) -> tuple:
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    config = data.get('config', {})
    calc = FlightPathCalculator(
        min_altitude=config.get('min_altitude', 100),
        max_altitude=config.get('max_altitude', 10000),
        grid_resolution=config.get('grid_resolution', 100)
    )
    
    for obs_data in data.get('obstacles', []):
        calc.add_obstacle(Obstacle.from_dict(obs_data))
    
    start = Point3D.from_dict(data['start'])
    goal = Point3D.from_dict(data['goal'])
    
    return calc, start, goal


def save_to_json(filepath: str, path: list, length: float) -> None:
    output = {
        'path': [p.to_dict() for p in path],
        'total_distance': length,
        'waypoints': len(path)
    }
    with open(filepath, 'w') as f:
        json.dump(output, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description='Flight Path Calculator')
    parser.add_argument('--input', '-i', type=str, help='Input JSON file')
    parser.add_argument('--output', '-o', type=str, help='Output JSON file')
    parser.add_argument('--algorithm', '-a', choices=['astar', 'rrt'], default='astar',
                        help='Pathfinding algorithm')
    parser.add_argument('--2d', action='store_true', help='Use 2D mode (ignore altitude)')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='Start point coordinates')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='Goal point coordinates')
    
    args = parser.parse_args()
    
    if args.input:
        calc, start, goal = load_from_json(args.input)
    elif args.start and args.goal:
        calc = FlightPathCalculator()
        start = Point3D(*args.start)
        goal = Point3D(*args.goal)
    else:
        calc = FlightPathCalculator()
        start = Point3D(0, 0, 500)
        goal = Point3D(8000, 8000, 3000)
        print("Using default test coordinates")
    
    print(f"Calculating path from {start} to {goal}...")
    print(f"Algorithm: {args.algorithm.upper()}")
    print(f"Mode: {'2D' if args.2d else '3D'}")
    
    if args.algorithm == 'astar':
        path = calc.calculate_path_astar(start, goal, use_3d=not args.2d)
    else:
        path = calc.calculate_path_rrt(start, goal)
    
    if path:
        smoothed = calc.smooth_path(path)
        length = calc.calculate_path_length(smoothed)
        
        print(f"\nPath found!")
        print(f"  Waypoints: {len(smoothed)}")
        print(f"  Total distance: {length:.2f} meters")
        print(f"  Start: {smoothed[0]}")
        print(f"  End: {smoothed[-1]}")
        
        if args.output:
            save_to_json(args.output, smoothed, length)
            print(f"\nPath saved to {args.output}")
    else:
        print("\nNo valid path found!")
        sys.exit(1)


if __name__ == '__main__':
    main()
