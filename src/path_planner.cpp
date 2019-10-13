#include <iostream>
#include <string>

#include "helpers.h"
#include "path_planner.h"

PathPlanner::PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) : maps_s(maps_s), maps_x(maps_x), maps_y(maps_y) { }

vector<vector<double>> PathPlanner::plan(Car &car, vector<vector<double>> &prevPath) {
    vector<double> pathX = prevPath[0];
    vector<double> pathY = prevPath[1];
    double startS, theta;
    Point start, startPrev;

    if (pathX.size() < 2) {
        startS = car.s;
        theta = deg2rad(car.yaw);
        start = {car.x, car.y};
        startPrev = {start.x - cos(theta), start.y - sin(theta)};
    } else {
        start = {pathX[pathX.size() - 1], pathY[pathY.size() - 1]};
        startPrev = {pathX[pathX.size() - 2], pathY[pathY.size() - 2]};
        theta = atan2(start.y - startPrev.y, start.x - startPrev.x);
        double sPrev = toFrenet(startPrev.x, startPrev.y, theta)[0];
        startS = toFrenet(start.x, start.y, theta)[0];
    }

    vector<double> splineX, splineY;
    splineX.push_back(startPrev.x);
    splineX.push_back(start.x);
    splineY.push_back(startPrev.y);
    splineY.push_back(start.y);

    int lane = 1;
    double targetD = 2 + lane * 4;

    for (int i = 0; i < 5; i++) {
        vector<double> xy = toCartesian(startS + 10 * (i + 1), targetD);
        splineX.push_back(xy[0]);
        splineY.push_back(xy[1]);
    }
    
    for (int i = 0; i < splineX.size(); i++) {
        double deltaX = splineX[i] - start.x;
        double deltaY = splineY[i] - start.y;

        splineX[i] = deltaX * cos(-theta) - deltaY * sin(-theta);
        splineY[i] = deltaX * sin(-theta) + deltaY * cos(-theta);
    }

    double targetV = car.speed + maxA * dt;
    if (targetV > maxV) {
        targetV = maxV;
    }

    spline pathSpline;
    pathSpline.set_points(splineX, splineY);

    double pointCount = maxPoints - pathX.size();
    double targetX = maxDS;
    double targetY = pathSpline(targetX);
    double targetDist = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double currentX = 0;
    double n = targetDist / (targetV * dt);

    for (int i = 1; i <= pointCount; i++) {
        double nextX = currentX + targetX / n;
        double nextY = pathSpline(nextX);
        currentX = nextX;

        pathX.push_back(start.x + nextX * cos(theta) - nextY * sin(theta));
        pathY.push_back(start.y + nextX * sin(theta) + nextY * cos(theta));
    }

    vector<vector<double>> path = {pathX, pathY};
    return path;
}

inline vector<double> PathPlanner::toCartesian(double startS, double d) {
    return getXY(startS, d, maps_s, maps_x, maps_y);
}

inline vector<double> PathPlanner::toFrenet(double x, double y, double theta) {
    return getFrenet(x, y, theta, maps_x, maps_y);
}