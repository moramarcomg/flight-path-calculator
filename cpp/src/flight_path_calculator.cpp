#include "flight_path_calculator.h"
#include <random>
#include <sstream>
#include <iomanip>

namespace flightpath {

FlightPathCalculator::FlightPathCalculator() {}

bool FlightPathCalculator::is_valid_point(const Point3D& p) const {
    if (p.z < min_altitude_ || p.z > max_altitude_) {
        return false;
    }
    
    for (const auto& obs : obstacles_) {
        if (obs.contains(p)) {
            return false;
        }
    }
    
    return true;
}

double FlightPathCalculator::heuristic(const Point3D& a, const Point3D& b) const {
    return a.distance(b);
}

Point3D FlightPathCalculator::snap_to_grid(const Point3D& p) const {
    return Point3D(
        std::round(p.x / grid_resolution_) * grid_resolution_,
        std::round(p.y / grid_resolution_) * grid_resolution_,
        std::round(p.z / grid_resolution_) * grid_resolution_
    );
}

std::optional<std::vector<Point3D>> FlightPathCalculator::calculate_path_astar(
    const Point3D& start,
    const Point3D& goal,
    bool use_3d
) {
    if (!is_valid_point(start) || !is_valid_point(goal)) {
        return std::nullopt;
    }
    
    std::priority_queue<PathNode*, std::vector<PathNode*>, PathNode::Compare> open_set;
    std::unordered_set<std::string> closed_set;
    
    auto point_to_key = [](const Point3D& p) -> std::string {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << p.x << "," << p.y << "," << p.z;
        return oss.str();
    };
    
    PathNode* start_node = new PathNode(start, 0.0, heuristic(start, goal));
    open_set.push(start_node);
    
    const double directions[] = {
        -grid_resolution_, 0, 0,
        grid_resolution_, 0, 0,
        0, -grid_resolution_, 0,
        0, grid_resolution_, 0,
        0, 0, use_3d ? -grid_resolution_ : 0,
        0, 0, use_3d ? grid_resolution_ : 0
    };
    
    while (!open_set.empty()) {
        PathNode* current = open_set.top();
        open_set.pop();
        
        std::string key = point_to_key(current->point);
        if (closed_set.find(key) != closed_set.end()) {
            delete current;
            continue;
        }
        closed_set.insert(key);
        
        if (current->point.distance(goal) < grid_resolution_) {
            std::vector<Point3D> path;
            PathNode* node = current;
            while (node != nullptr) {
                path.push_back(node->point);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            
            while (!open_set.empty()) {
                delete open_set.top();
                open_set.pop();
            }
            
            return path;
        }
        
        for (size_t i = 0; i < 6; ++i) {
            Point3D neighbor(
                current->point.x + directions[i * 3],
                current->point.y + directions[i * 3 + 1],
                current->point.z + directions[i * 3 + 2]
            );
            
            if (!is_valid_point(neighbor)) {
                continue;
            }
            
            if (closed_set.find(point_to_key(neighbor)) != closed_set.end()) {
                continue;
            }
            
            double g = current->g_cost + current->point.distance(neighbor);
            double f = g + heuristic(neighbor, goal);
            
            open_set.push(new PathNode(neighbor, g, f, current));
        }
    }
    
    return std::nullopt;
}

std::optional<std::vector<Point3D>> FlightPathCalculator::calculate_path_rrt(
    const Point3D& start,
    const Point3D& goal,
    int max_iterations,
    double step_size
) {
    if (!is_valid_point(start) || !is_valid_point(goal)) {
        return std::nullopt;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(0, 10000);
    std::uniform_real_distribution<> y_dist(0, 10000);
    std::uniform_real_distribution<> z_dist(min_altitude_, max_altitude_);
    
    struct RRTNode {
        Point3D point;
        RRTNode* parent;
        RRTNode(const Point3D& p, RRTNode* par) : point(p), parent(par) {}
    };
    
    std::vector<RRTNode*> nodes;
    nodes.reserve(max_iterations);
    
    RRTNode* root = new RRTNode(start, nullptr);
    nodes.push_back(root);
    
    RRTNode* goal_node = nullptr;
    
    for (int i = 0; i < max_iterations; ++i) {
        Point3D random_point;
        
        std::uniform_real_distribution<> prob(0, 1);
        if (prob(gen) < 0.1) {
            random_point = goal;
        } else {
            random_point = Point3D(x_dist(gen), y_dist(gen), z_dist(gen));
        }
        
        RRTNode* nearest = nullptr;
        double min_dist = std::numeric_limits<double>::max();
        
        for (RRTNode* node : nodes) {
            double d = node->point.distance(random_point);
            if (d < min_dist) {
                min_dist = d;
                nearest = node;
            }
        }
        
        if (!nearest) continue;
        
        double dx = random_point.x - nearest->point.x;
        double dy = random_point.y - nearest->point.y;
        double dz = random_point.z - nearest->point.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < step_size) continue;
        
        double ratio = step_size / dist;
        Point3D new_point(
            nearest->point.x + dx * ratio,
            nearest->point.y + dy * ratio,
            nearest->point.z + dz * ratio
        );
        
        if (!is_valid_point(new_point)) continue;
        
        RRTNode* new_node = new RRTNode(new_point, nearest);
        nodes.push_back(new_node);
        
        if (new_point.distance(goal) < step_size) {
            goal_node = new_node;
            break;
        }
    }
    
    if (!goal_node) {
        for (RRTNode* node : nodes) {
            delete node;
        }
        return std::nullopt;
    }
    
    std::vector<Point3D> path;
    RRTNode* current = goal_node;
    while (current != nullptr) {
        path.push_back(current->point);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    
    for (RRTNode* node : nodes) {
        delete node;
    }
    
    return path;
}

std::vector<Point3D> FlightPathCalculator::smooth_path(const std::vector<Point3D>& path) {
    if (path.size() <= 2) return path;
    
    std::vector<Point3D> smoothed = {path[0]};
    size_t current_idx = 0;
    
    while (current_idx < path.size() - 1) {
        size_t furthest = current_idx + 1;
        
        for (size_t i = path.size() - 1; i > current_idx + 1; --i) {
            bool can_reach = true;
            
            for (size_t j = current_idx + 1; j < i; ++j) {
                if (!is_valid_point(path[j])) {
                    can_reach = false;
                    break;
                }
                
                for (const auto& obs : obstacles_) {
                    if (obs.contains(path[j])) {
                        can_reach = false;
                        break;
                    }
                }
                
                if (!can_reach) break;
            }
            
            if (can_reach) {
                furthest = i;
                break;
            }
        }
        
        smoothed.push_back(path[furthest]);
        current_idx = furthest;
    }
    
    return smoothed;
}

double FlightPathCalculator::calculate_path_length(const std::vector<Point3D>& path) const {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += path[i-1].distance(path[i]);
    }
    return length;
}

}
