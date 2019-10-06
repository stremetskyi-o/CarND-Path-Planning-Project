#include <iostream>
#include <string>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "path_planner.h"

PathPlanner::PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) : maps_s(maps_s), maps_x(maps_x), maps_y(maps_y) { }

vector<vector<double>> PathPlanner::plan(Car &car, vector<vector<double>> &prevPath) {
    vector<double> pathX;
    vector<double> pathY;
    double deltaS = 0.44;
    for (int i = 0; i < 50; i++) {
        double s = car.s + (i + 1) * deltaS;
        vector<double> xy = toCartesian(s, car.d);
        pathX.push_back(xy[0]);
        pathY.push_back(xy[1]);
    }
    vector<vector<double>> path;
    path.push_back(pathX);
    path.push_back(pathY);    
    return path;
}

inline vector<double> PathPlanner::toCartesian(double s, double d) {
    return getXY(s, d, maps_s, maps_x, maps_y);
}