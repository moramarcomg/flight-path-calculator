#ifndef FLIGHT_PATH_CALCULATOR_H
#define FLIGHT_PATH_CALCULATOR_H

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <optional>
#include <functional>

namespace flightpath {

struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
    
    double distance(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    bool operator==(const Point3D& other) const {
        const double eps = 1e-9;
        return std::abs(x - other.x) < eps &&
               std::abs(y - other.y) < eps &&
               std::abs(z - other.z) < eps;
    }
};

struct Obstacle {
    Point3D center;
    double radius;
    
    Obstacle(double x, double y, double z, double r) 
        : center(x, y, z), radius(r) {}
    
    bool contains(const Point3D& p) const {
        return center.distance(p) < radius;
    }
};

struct PathNode {
    Point3D point;
    double g_cost;
    double f_cost;
    PathNode* parent;
    
    PathNode(const Point3D& p, double g, double f, PathNode* par = nullptr)
        : point(p), g_cost(g), f_cost(f), parent(par) {}
    
    struct Compare {
        bool operator()(const PathNode* a, const PathNode* b) const {
            return a->f_cost > b->f_cost;
        }
    };
};

class FlightPathCalculator {
public:
    FlightPathCalculator();
    
    void set_min_altitude(double alt) { min_altitude_ = alt; }
    void set_max_altitude(double alt) { max_altitude_ = alt; }
    void set_min_turn_radius(double r) { min_turn_radius_ = r; }
    void set_grid_resolution(double res) { grid_resolution_ = res; }
    
    void add_obstacle(const Obstacle& obs) { obstacles_.push_back(obs); }
    void clear_obstacles() { obstacles_.clear(); }
    
    std::optional<std::vector<Point3D>> calculate_path_astar(
        const Point3D& start, 
        const Point3D& goal,
        bool use_3d = true
    );
    
    std::optional<std::vector<Point3D>> calculate_path_rrt(
        const Point3D& start,
        const Point3D& goal,
        int max_iterations = 10000,
        double step_size = 1.0
    );
    
    std::vector<Point3D> smooth_path(const std::vector<Point3D>& path);
    double calculate_path_length(const std::vector<Point3D>& path) const;

private:
    double min_altitude_ = 100.0;
    double max_altitude_ = 10000.0;
    double min_turn_radius_ = 500.0;
    double grid_resolution_ = 100.0;
    
    std::vector<Obstacle> obstacles_;
    
    bool is_valid_point(const Point3D& p) const;
    double heuristic(const Point3D& a, const Point3D& b) const;
    Point3D snap_to_grid(const Point3D& p) const;
};

}

#endif
