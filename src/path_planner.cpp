#include <iostream>
#include <string>

#include "helpers.h"
#include "path_planner.h"

using std::cout;
using std::endl;

PathPlanner::PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) : maps_s(maps_s), maps_x(maps_x), maps_y(maps_y) { }

vector<vector<double>> PathPlanner::plan(Car &car, vector<vector<double>> &prevPath, vector<OtherCar> &otherCars) {
    // Reset target speed if client reconnected
    if (car.speed == 0) {
        targetV = 0;
    }

    vector<double> pathX = prevPath[0];
    vector<double> pathY = prevPath[1];
    double theta;
    PointXY start, startPrev;
    PointSD startSD;

    if (pathX.size() < 2) {
        startSD = {car.s, car.d};
        theta = deg2rad(car.yaw);
        start = {car.x, car.y};
        startPrev = {start.x - cos(theta), start.y - sin(theta)};
    } else {
        start = {pathX[pathX.size() - 1], pathY[pathY.size() - 1]};
        startPrev = {pathX[pathX.size() - 2], pathY[pathY.size() - 2]};
        theta = atan2(start.y - startPrev.y, start.x - startPrev.x);
        vector<double> sd = toFrenet(start.x, start.y, theta);
        startSD = {sd[0], sd[1]};
    }

    int startLane = startSD.d / 4;

    // Choose next state
    vector<State> nextStates = fsm[state];
    vector<double> nextStateCosts;
    for (State &nextState : nextStates) {
        double cost;
        switch (nextState) {
        case LK:
            cost = 0;
            break;
        case PLCL:
        case PLCR: 
        case LCL:
        case LCR:
            cost = 100;
            break;
        }
        nextStateCosts.push_back(cost);
    }

    State nextState = nextStates[std::distance(nextStateCosts.begin(), std::min_element(nextStateCosts.begin(), nextStateCosts.end()))];
    if (state != nextState) {
        cout << "Transiting to state: " << nextState << endl;
        state = nextState;
    }

    // Calculate target speed and lane
    int targetLane;

    switch (state) {
    case LK: {
        targetLane = startLane;
        vector<OtherCar> carsInlane = filterCarsByLane(otherCars, startLane);
        OtherCar* closestCar = findClosestCar(carsInlane, startSD.s, startSD.s + 5);
        if (closestCar) {
            double otherCarV = sqrt(pow(closestCar->vx, 2) + pow(closestCar->vy, 2));
            targetV -= maxA * dt;
            if (targetV < otherCarV) {
                targetV = otherCarV;
            }
        } else {
            targetV += maxA * dt;
            if (targetV > maxV) {
                targetV = maxV;
            }
        }
        break;
    }
    case LCL:
        targetLane = startLane - 1;
        break;
    case LCR:
        targetLane = startLane + 1;
        break;
    }

    // Generate Trajectory
    double targetD = 2 + targetLane * 4;

    vector<double> splineX, splineY;
    splineX.push_back(startPrev.x);
    splineX.push_back(start.x);
    splineY.push_back(startPrev.y);
    splineY.push_back(start.y);

    for (int i = 0; i < 5; i++) {
        vector<double> xy = toCartesian(startSD.s + 10 * (i + 1), targetD);
        splineX.push_back(xy[0]);
        splineY.push_back(xy[1]);
    }
    
    for (int i = 0; i < splineX.size(); i++) {
        double deltaX = splineX[i] - start.x;
        double deltaY = splineY[i] - start.y;

        splineX[i] = deltaX * cos(-theta) - deltaY * sin(-theta);
        splineY[i] = deltaX * sin(-theta) + deltaY * cos(-theta);
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

vector<OtherCar> PathPlanner::filterCarsByLane(vector<OtherCar> &otherCars, int lane) {
    vector<OtherCar> result;
    for (auto &car : otherCars) {
        if (car.d > lane * 4 && car.d < (lane + 1) * 4) {
            result.push_back(car);
        }
    }
    return result;
}

OtherCar* PathPlanner::findClosestCar(vector<OtherCar> &otherCars, double fromS, double toS) {
    OtherCar* result = nullptr;
    double dist = -1;
    for (auto &car : otherCars) {
        if (car.s > fromS && car.s < toS && (dist == -1 || car.s - fromS < dist)) {
            result = &car;
            dist = car.s - fromS;
        }
    }
    return result;
}