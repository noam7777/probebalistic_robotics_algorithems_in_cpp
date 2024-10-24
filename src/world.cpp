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


void World::plotWorld(bool plotGt, bool plotEkfEstimation, bool plotParticleFilterEstimation, bool plotLandmark) {

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
        std::vector<std::string> colors;

        // Components of the heading vector
        for (auto& robot : this->robots) {
            std::vector<float> quality;
            for (auto& particle : robot.pf.particles) {
                x.push_back(particle.state(0));
                y.push_back(particle.state(1));
                u.push_back(std::cos(particle.state(2)));
                v.push_back(std::sin(particle.state(2)));
                quality.push_back(particle.weight);
            }
            // Normalize quality values to [0, 1] range for color mapping
            std::vector<float> normalizedQuality = normalize(quality);

            // Create a colormap based on the normalized quality values
            for (float normQual : normalizedQuality) {
                int r = static_cast<int>(255 * (1 - normQual));
                int g = 0;
                int b = static_cast<int>(255 * normQual);
                char color[8];
                snprintf(color, sizeof(color), "#%02x%02x%02x", r, g, b);
                colors.push_back(std::string(color));
            }
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


    if (plotLandmark) {
        // Assuming landmark positions are stored in this->landmark (Eigen::Vector2f or similar)
        std::vector<float> landmark_x;
        std::vector<float> landmark_y;

        landmark_x.push_back(this->lendMark[0]);  // X-coordinate of landmark
        landmark_y.push_back(this->lendMark[1]);  // Y-coordinate of landmark

        // Create a dictionary for keyword arguments (marker style and color)
        std::unordered_map<std::string, std::string> keywords;
        keywords["marker"] = "s";  // "s" for square
        keywords["color"] = "g";   // "g" for green

        // Plot the landmark as green square scatter point
        plt::scatter(landmark_x, landmark_y, 100.0, keywords);
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


void World::addRobotToArchive(Robot robot) {
    robots.push_back(robot);
}

void World::cleanWorld(void) {
    this->robots.clear();
}

Eigen::Vector3f World::getGpsCompassMeasurement(Robot robot) {

    return robot.stateGT;
}
float World::getRangeFromLandmarkMeasurement(Robot robot) {
    Eigen::Vector2f robotPos;
    robotPos << robot.stateGT[0], robot.stateGT[1];
    
    float rangeFromLandmark = (robotPos - this->lendMark).norm();
    rangeFromLandmark = (rangeFromLandmark * LANDMARK_RANGE_ERROR_SCALE) + LANDMARK_RANGE_ERROR_BIAS;
    rangeFromLandmark = std::max(rangeFromLandmark, 0.5f);
    return rangeFromLandmark;
}


float World::getRssiMeasurementFromLandmark(Robot robot) {
    float rangeFromLandmark = getRangeFromLandmarkMeasurement(robot);
    float rssi = robot.rssiModelGT.calcRssiFromRange(rangeFromLandmark);
    return rssi;
}

void plotCircle(float centerX, float centerY, float radius) {
    const int numPoints = 100;  // Number of points to create a smooth circle
    std::vector<float> circleX(numPoints), circleY(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float angle = 2 * M_PI * i / numPoints;
        circleX[i] = centerX + radius * std::cos(angle);
        circleY[i] = centerY + radius * std::sin(angle);
    }

    plt::plot(circleX, circleY, "b-");  // Plot circle with a blue line
}



void World::animateRobotStates(bool plotGt, bool plotEkfEstimation, bool plotParticleFilterEstimation, bool plotLandmark) {
    // Iterate over the chronological robot states in the `this->robots` vector
    for (int frameIdx = 0; frameIdx < this->robots.size(); ++frameIdx) {  
        // Clear the current plot
        plt::clf();

        // Plot ground truth if enabled
        if (plotGt) {
            std::vector<float> x, y, u, v;

            // Extract the robot state for this time step
            Robot& robot = this->robots[frameIdx];
            x.push_back(robot.stateGT(0));
            y.push_back(robot.stateGT(1));
            u.push_back(std::cos(robot.stateGT(2)));
            v.push_back(std::sin(robot.stateGT(2)));

            // Plot the robot's position and heading
            plt::quiver(x, y, u, v);
        }

        // Plot EKF estimation if enabled
        if (plotEkfEstimation) {
            std::vector<float> x, y, u, v;

            // Extract the robot EKF state for this time step
            Robot& robot = this->robots[frameIdx];
            x.push_back(robot.ekf.state(0));
            y.push_back(robot.ekf.state(1));
            u.push_back(std::cos(robot.ekf.state(2)));
            v.push_back(std::sin(robot.ekf.state(2)));

            Eigen::Matrix2f ellipseCov;
            ellipseCov << robot.ekf.covMatrix(0, 0), robot.ekf.covMatrix(0, 1),
                          robot.ekf.covMatrix(1, 0), robot.ekf.covMatrix(1, 1);

            // Plot the EKF covariance ellipse
            plotEllipse(robot.ekf.state(0), robot.ekf.state(1), ellipseCov);

            // Plot the robot's EKF estimated position and heading
            plt::quiver(x, y, u, v);
            PyRun_SimpleString(
                "import matplotlib.pyplot as plt\n"
                "quiver = plt.gca().collections[-1]\n"
                "quiver.set_color('red')\n"
                "quiver.set_linewidth(0.01)\n"
            );
        }

        // Plot particle filter estimation if enabled
        if (plotParticleFilterEstimation) {
            std::vector<float> x, y, u, v;
            std::vector<std::string> colors;

            // Extract the robot particle filter state for this time step
            Robot& robot = this->robots[frameIdx];

            std::vector<float> quality;
            for (auto& particle : robot.pf.particles) {
                x.push_back(particle.state(0));
                y.push_back(particle.state(1));
                u.push_back(std::cos(particle.state(2)));
                v.push_back(std::sin(particle.state(2)));
                quality.push_back(particle.weight);
            }

            std::vector<float> normalizedQuality = normalize(quality);
            for (float normQual : normalizedQuality) {
                int r = static_cast<int>(255 * (1 - normQual));
                int b = static_cast<int>(255 * normQual);
                char color[8];
                snprintf(color, sizeof(color), "#%02x00%02x", r, b);
                colors.push_back(std::string(color));
            }

            plt::quiver(x, y, u, v);
            std::string colorList = vectorToPythonListString(colors);
            PyRun_SimpleString(
                ("import matplotlib.pyplot as plt\n"
                 "quiver = plt.gca().collections[-1]\n"
                 "colors = " + colorList + "\n"
                 "quiver.set_color(colors)\n"
                 "quiver.set_cmap('coolwarm')\n"
                 "plt.colorbar(quiver)\n").c_str()
            );
        }

        // Plot the landmark, circle, and line if enabled
        if (plotLandmark) {
            std::vector<float> landmark_x, landmark_y;
            landmark_x.push_back(this->lendMark[0]);
            landmark_y.push_back(this->lendMark[1]);

            std::unordered_map<std::string, std::string> keywords;
            keywords["marker"] = "s";
            keywords["color"] = "g";
            plt::scatter(landmark_x, landmark_y, 100.0, keywords);

            // Plot the circle around the landmark
            Robot& robot = this->robots[frameIdx];  // Get current robot state
            float radius = std::sqrt(std::pow(robot.stateGT(0) - this->lendMark[0], 2) + 
                                     std::pow(robot.stateGT(1) - this->lendMark[1], 2));
            plotCircle(this->lendMark[0], this->lendMark[1], radius);  // Call the custom plotCircle function

            // Plot the line connecting the robot and the landmark
            std::vector<float> line_x = {robot.stateGT(0), this->lendMark[0]};
            std::vector<float> line_y = {robot.stateGT(1), this->lendMark[1]};
            plt::plot(line_x, line_y, "k-");  // Thin black line
        }

        // Customize the plot
        plt::xlim(0, this->width);
        plt::ylim(0, this->height);
        plt::title("Robot Position and Heading (Frame " + std::to_string(frameIdx) + ")");
        plt::xlabel("X position");
        plt::ylabel("Y position");

        // Pause briefly to create the animation effect
        plt::pause(0.1);  // Adjust the pause duration for smoother animation
    }

    // Keep the plot window open after the animation
    plt::show();
}


