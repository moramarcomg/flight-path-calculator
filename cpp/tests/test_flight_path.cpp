#include <gtest/gtest.h>
#include "flight_path_calculator.h"

using namespace flightpath;

class FlightPathTest : public ::testing::Test {
protected:
    FlightPathCalculator calculator;
};

TEST_F(FlightPathTest, PointDistance) {
    Point3D p1(0, 0, 0);
    Point3D p2(3, 4, 0);
    EXPECT_DOUBLE_EQ(p1.distance(p2), 5.0);
}

TEST_F(FlightPathTest, PointEquality) {
    Point3D p1(1.0, 2.0, 3.0);
    Point3D p2(1.0, 2.0, 3.0);
    Point3D p3(1.1, 2.0, 3.0);
    EXPECT_TRUE(p1 == p2);
    EXPECT_FALSE(p1 == p3);
}

TEST_F(FlightPathTest, SimplePathCalculation) {
    Point3D start(0, 0, 500);
    Point3D goal(500, 500, 500);
    
    auto path = calculator.calculate_path_astar(start, goal, false);
    ASSERT_TRUE(path.has_value());
    EXPECT_GT(path->size(), 1);
    EXPECT_EQ(path->front(), start);
}

TEST_F(FlightPathTest, PathWithObstacle) {
    calculator.add_obstacle(Obstacle(500, 500, 500, 200));
    
    Point3D start(0, 0, 500);
    Point3D goal(1000, 1000, 500);
    
    auto path = calculator.calculate_path_astar(start, goal, false);
    EXPECT_TRUE(path.has_value());
    
    for (const auto& point : *path) {
        EXPECT_FALSE(calculator.is_valid_point(point) == false);
    }
}

TEST_F(FlightPathTest, UnreachablePath) {
    calculator.add_obstacle(Obstacle(500, 500, 500, 10000));
    
    Point3D start(0, 0, 500);
    Point3D goal(1000, 1000, 500);
    
    auto path = calculator.calculate_path_astar(start, goal, false);
    EXPECT_FALSE(path.has_value());
}

TEST_F(FlightPathTest, RRTPathCalculation) {
    Point3D start(0, 0, 500);
    Point3D goal(1000, 1000, 500);
    
    auto path = calculator.calculate_path_rrt(start, goal, 1000, 100);
    EXPECT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), start);
}

TEST_F(FlightPathTest, PathLengthCalculation) {
    std::vector<Point3D> path = {
        Point3D(0, 0, 0),
        Point3D(3, 0, 0),
        Point3D(3, 4, 0)
    };
    
    double length = calculator.calculate_path_length(path);
    EXPECT_DOUBLE_EQ(length, 7.0);
}

TEST_F(FlightPathTest, AltitudeConstraints) {
    calculator.set_min_altitude(100);
    calculator.set_max_altitude(500);
    
    EXPECT_TRUE(calculator.is_valid_point(Point3D(0, 0, 300)));
    EXPECT_FALSE(calculator.is_valid_point(Point3D(0, 0, 50)));
    EXPECT_FALSE(calculator.is_valid_point(Point3D(0, 0, 600)));
}

TEST_F(FlightPathTest, ClearObstacles) {
    calculator.add_obstacle(Obstacle(500, 500, 500, 200));
    EXPECT_EQ(calculator.calculate_path_astar(
        Point3D(0, 0, 500), 
        Point3D(1000, 1000, 500), 
        false
    ).has_value(), false);
    
    calculator.clear_obstacles();
    EXPECT_TRUE(calculator.calculate_path_astar(
        Point3D(0, 0, 500), 
        Point3D(1000, 1000, 500), 
        false
    ).has_value());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
