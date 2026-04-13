#include "flight_path_calculator.h"
#include <iostream>
#include <iomanip>

using namespace flightpath;

void print_path(const std::vector<Point3D>& path, double length) {
    std::cout << "\nFlight Path (" << path.size() << " waypoints):\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Start: (" << path.front().x << ", " << path.front().y 
              << ", " << path.front().z << ")\n";
    std::cout << "End:   (" << path.back().x << ", " << path.back().y 
              << ", " << path.back().z << ")\n";
    std::cout << "Total distance: " << length << " meters\n";
}

int main() {
    FlightPathCalculator calculator;
    
    calculator.set_min_altitude(200.0);
    calculator.set_max_altitude(5000.0);
    calculator.set_grid_resolution(200.0);
    
    calculator.add_obstacle(Obstacle(3000, 3000, 1000, 800));
    calculator.add_obstacle(Obstacle(5000, 2000, 1500, 600));
    
    Point3D start(0, 0, 500);
    Point3D goal(8000, 8000, 3000);
    
    std::cout << "=== Flight Path Calculator ===\n";
    std::cout << "Calculating path from origin to (8000, 8000, 3000)...\n";
    
    auto astar_path = calculator.calculate_path_astar(start, goal, true);
    
    if (astar_path) {
        double length = calculator.calculate_path_length(*astar_path);
        print_path(*astar_path, length);
        
        auto smoothed = calculator.smooth_path(*astar_path);
        double smoothed_length = calculator.calculate_path_length(smoothed);
        std::cout << "\nSmoothed path: " << smoothed.size() << " waypoints, "
                  << smoothed_length << " meters\n";
    } else {
        std::cout << "A* path not found, trying RRT...\n";
        
        auto rrt_path = calculator.calculate_path_rrt(start, goal, 5000, 300);
        if (rrt_path) {
            double length = calculator.calculate_path_length(*rrt_path);
            print_path(*rrt_path, length);
        } else {
            std::cout << "No valid path found.\n";
        }
    }
    
    calculator.clear_obstacles();
    
    Point3D start2d(0, 0, 1000);
    Point3D goal2d(10000, 10000, 1000);
    
    std::cout << "\n=== 2D Path Test ===\n";
    auto path2d = calculator.calculate_path_astar(start2d, goal2d, false);
    if (path2d) {
        double length = calculator.calculate_path_length(*path2d);
        print_path(*path2d, length);
    }
    
    return 0;
}
