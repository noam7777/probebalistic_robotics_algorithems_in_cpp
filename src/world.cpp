#include <world.hpp>
#include "matplotlibcpp.h"
#include <Python.h>
#include "utils.hpp"

namespace plt = matplotlibcpp;

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

std::vector<float> normalize(const std::vector<float>& values) {
    float minVal = *std::min_element(values.begin(), values.end());
    float maxVal = *std::max_element(values.begin(), values.end());
    std::vector<float> normalizedValues;
    for (float value : values) {
        normalizedValues.push_back((value - minVal) / (maxVal - minVal));
    }
    return normalizedValues;
}

std::string vectorToPythonListString(const std::vector<std::string>& vec) {
    std::string result = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        result += "'" + vec[i] + "'";
        if (i != vec.size() - 1) {
            result += ", ";
        }
    }
    result += "]";
    return result;
}

void World::plotWorld(bool plotGt, bool plotEkfEstimation, bool plotParticleFilterEstimation) {

    if (plotGt) {
        // Plot all the robots:
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> u;
        std::vector<float> v;

        // Components of the heading vector
        for (auto& robot : this->robots) {
            x.push_back(robot.stateGT(0));
            y.push_back(robot.stateGT(1));
            u.push_back(std::cos(robot.stateGT(2)));
            v.push_back(std::sin(robot.stateGT(2)));
        }

        // Plot the robot's position and heading using a quiver plot
        plt::quiver(x, y, u, v);
    }

    if (plotEkfEstimation) {
        // Plot all the robots:
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> u;
        std::vector<float> v;

        // Components of the heading vector
        for (auto& robot : this->robots) {
            x.push_back(robot.ekf.state(0));
            y.push_back(robot.ekf.state(1));
            u.push_back(std::cos(robot.ekf.state(2)));
            v.push_back(std::sin(robot.ekf.state(2)));
            Eigen::Matrix2f ellipseCov;
            ellipseCov << robot.ekf.covMatrix(0, 0), robot.ekf.covMatrix(0, 1),
                          robot.ekf.covMatrix(1, 0), robot.ekf.covMatrix(1, 1);
            plotEllipse(robot.ekf.state(0), robot.ekf.state(1), ellipseCov);
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

    if (plotParticleFilterEstimation) {
        // Plot all the robots:
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> u;
        std::vector<float> v;
        std::vector<float> quality;

        // Components of the heading vector
        for (auto& robot : this->robots) {
            for (auto& particle : robot.pf.stateParticles) {
                x.push_back(particle(0));
                y.push_back(particle(1));
                u.push_back(std::cos(particle(2)));
                v.push_back(std::sin(particle(2)));
                double qual = likelyhoodToGetMeasurementGpsCompassFromState(this->getMeasurement(robot), particle);
                quality.push_back(qual);
            }
        }

        // Normalize quality values to [0, 1] range for color mapping
        std::vector<float> normalizedQuality = normalize(quality);

        // Create a colormap based on the normalized quality values
        std::vector<std::string> colors;
        for (float normQual : normalizedQuality) {
            int r = static_cast<int>(255 * (1 - normQual));
            int g = 0;
            int b = static_cast<int>(255 * normQual);
            char color[8];
            snprintf(color, sizeof(color), "#%02x%02x%02x", r, g, b);
            colors.push_back(std::string(color));
        }

        // Plot the robot's position and heading using a quiver plot
        plt::quiver(x, y, u, v);

        std::string colorList = vectorToPythonListString(colors);

        PyRun_SimpleString(
            ("import matplotlib.pyplot as plt\n"
             "import numpy as np\n"
             "quiver = plt.gca().collections[-1]\n"
             "colors = " + colorList + "\n"
             "quiver.set_color(colors)\n"
             "quiver.set_cmap('coolwarm')\n"
             "plt.colorbar(quiver)\n").c_str()
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
