#include <world.hpp>
#include "matplotlibcpp.h"
#include <Python.h>
#include <random>


namespace plt = matplotlibcpp;

float getRandomNumber(void) {
    // Create a random number generator and seed it with a random device
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Define the range for the random numbers
    static std::uniform_real_distribution<> dis(0.0, 1.0);

    // Generate a random number
    float random_number = dis(gen);
    return random_number;
}


static inline void plotEllipse(float x, float y, const Eigen::Matrix2f& cov) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver(cov);
    Eigen::Vector2f eigenvalues = eigenSolver.eigenvalues();
    Eigen::Matrix2f eigenvectors = eigenSolver.eigenvectors();

    float angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
    float width = 2 * std::sqrt(eigenvalues(0));
    float height = 2 * std::sqrt(eigenvalues(1));

    std::vector<float> ellipse_x;
    std::vector<float> ellipse_y;
    int num_points = 100;
    for (int i = 0; i < num_points; ++i) {
        float theta = 2 * M_PI * i / num_points;
        float ex = width / 2 * std::cos(theta);
        float ey = height / 2 * std::sin(theta);
        float px = x + ex * std::cos(angle) - ey * std::sin(angle);
        float py = y + ex * std::sin(angle) + ey * std::cos(angle);
        ellipse_x.push_back(px);
        ellipse_y.push_back(py);
    }

    plt::plot(ellipse_x, ellipse_y, "r-");
}

void World::plotWorld(bool plotGt, bool PlotEkfEstimation) {

    if (plotGt) {
        // Plot all the robots:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> u;
        std::vector<double> v;

        // Components of the heading vector

        for (std::vector<Robot>::iterator it = this->robots.begin(); it != this->robots.end(); ++it) {
            x.push_back(it->stateGT(0));
            y.push_back(it->stateGT(1));
            u.push_back(std::cos(it->stateGT(2)));
            v.push_back(std::sin(it->stateGT(2)));
        }

        // Plot the robot's position and heading using a quiver plot
        plt::quiver(x, y, u, v);
    }
    if (PlotEkfEstimation) {
        // Plot all the robots:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> u;
        std::vector<double> v;

        // Components of the heading vector

        for (std::vector<Robot>::iterator it = this->robots.begin(); it != this->robots.end(); ++it) {
            x.push_back(it->ekf.state(0));
            y.push_back(it->ekf.state(1));
            u.push_back(std::cos(it->ekf.state(2)));
            v.push_back(std::sin(it->ekf.state(2)));
            Eigen::Matrix2f ellipseCov;
            ellipseCov << it->ekf.covMatrix(0, 0), it->ekf.covMatrix(0, 1),
                          it->ekf.covMatrix(1, 0), it->ekf.covMatrix(1, 1);
            plotEllipse(it->ekf.state(0), it->ekf.state(1), ellipseCov);
        }

        // Plot the robot's position and heading using a quiver plot
        
        plt::quiver(x, y, u, v);
        PyRun_SimpleString(
            "import matplotlib.pyplot as plt\n"
            "quiver = plt.gca().collections[-1]\n"
            "quiver.set_color('red')\n"
            "quiver.set_linewidth(0.01)\n"
        );

    }

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

void World::addRobotGroundTruth(Robot robot) {
    robots.push_back(robot);
}

void World::cleanWorld(void) {
    this->robots.clear();
}

Eigen::Vector3f World::getMeasurement(Robot robot) {
    return robot.stateGT;
}