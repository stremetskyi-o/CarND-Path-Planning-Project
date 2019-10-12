#include <iostream>
#include <string>

#include "helpers.h"
#include "path_planner.h"

PathPlanner::PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) : maps_s(maps_s), maps_x(maps_x), maps_y(maps_y) { }

vector<vector<double>> PathPlanner::plan(Car &car, vector<vector<double>> &prevPath) {
    vector<double> pathX;
    vector<double> pathY;
    double speed = car.speed;
    double s = car.s;
    double sEstimated = car.s + estimateDistance(car.speed, maxV, 1);
    vector<double> start = {s, speed, 0};
    vector<double> end = {sEstimated, maxV, 0};
    VectorXd coeff = calcCoeff(start, end, 1);
    for (int i = 0; i < maxPoints; i++) {
        VectorXd polynom(6);
        double t = dt * (i + 1);
        polynom << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
        s = coeff.dot(polynom);
        vector<double> xy = toCartesian(s, car.d);
        pathX.push_back(xy[0]);
        pathY.push_back(xy[1]);
    }
    vector<vector<double>> path;
    path.push_back(pathX);
    path.push_back(pathY);    
    return path;
}

double PathPlanner::estimateDistance(double currentV, double targetV, double t) {
    double d = targetV * (t - ((targetV - currentV) / maxA));
    return d;
}

VectorXd PathPlanner::calcCoeff(vector<double> &start, vector<double> &end, double t) {  
  double t2 = pow(t, 2);
  double t3 = pow(t, 3);
  double t4 = pow(t, 4);
  MatrixXd time(3, 3);
  time << t3, t4, pow(t, 5),
          3 * t2, 4 * t3, 5 * t4,
          6 * t, 12 * t2, 20 * t3;
  VectorXd s(3);
  s << end[0] - (start[0] + start[1] * t + start[2] * t2 / 2),
       end[1] - (start[1] + start[2] * t),
       end[2] - start[2];
  
  VectorXd result(6);
  result << start[0], start[1], start[2] / 2, time.inverse() * s;
  return result;
}

inline vector<double> PathPlanner::toCartesian(double s, double d) {
    return getXY(s, d, maps_s, maps_x, maps_y);
}