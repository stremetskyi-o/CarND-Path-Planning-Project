#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Car {
        double x, y, s, d, yaw, speed;
    };

class PathPlanner {

    public:

    PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y);

    vector<vector<double>> plan(Car &car, vector<vector<double>> &prevPath);

    private:
    vector<double> &maps_s;
    vector<double> &maps_x;
    vector<double> &maps_y;
    double maxPoints = 50;
    double dt = 0.02;
    double maxV = 22;
    double maxDS = maxV / (1 / dt);
    double maxA = 9.8;

    inline vector<double> toCartesian(double s, double d);
    double estimateDistance(double currentV, double targetV, double t);
    VectorXd calcCoeff(vector<double> &start, vector<double> &end, double T);
};

#endif // PATH_PLANNER_H
