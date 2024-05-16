#include <world.hpp>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


void World::plotWorld(void) {

    // Plot all the robots:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> u;
    std::vector<double> v;
    
    // Robot heading (theta) in radians
    double theta = M_PI / 4;  // 45 degrees

    // Components of the heading vector

    for (std::vector<Robot>::iterator it = this->robots.begin(); it != this->robots.end(); ++it) {
        x.push_back(it->x);
        y.push_back(it->y);
        u.push_back(std::cos(it->theta));
        v.push_back(std::sin(it->theta));
    }

    // Plot the robot's position and heading using a quiver plot
    plt::quiver(x, y, u, v);

    // Customize the plot
    plt::xlim(0, this->width);
    plt::ylim(0, this->height);
    plt::title("Robot Position and Heading");
    plt::xlabel("X position");
    plt::ylabel("Y position");

    // Show the plot
    plt::show();
    return;
}

void World::addRobotToWorld(Robot robot) {
    robots.push_back(robot);
}

void World::cleanWorld(void) {
    this->robots.clear();
}
