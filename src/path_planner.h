#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using std::vector;
using std::map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using tk::spline;

struct Car {
    double x, y, s, d, yaw, speed;
};

struct OtherCar {
    double id, x, y, vx, vy, s, d;
};

struct OtherCarPrediction {
    double id, v, s, d;
};

struct PointXY {
    double x, y;
};

struct PointSD {
    double s, d;
};

enum State { INIT, LK, PLCL, PLCR, LCL, LCR };

struct Kinematics {
    double v, deltaS;
    int deltaLane, resultLane;
};

class PathPlanner {

    public:

    PathPlanner(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y);

    vector<vector<double>> plan(Car &car, vector<vector<double>> &prevPath, vector<OtherCar> &otherCars);

    private:
    vector<double> &maps_s;
    vector<double> &maps_x;
    vector<double> &maps_y;
    double maxPoints = 50;
    double dt = 0.02;
    double maxV = 22;
    double maxDS = maxV / (1 / dt);
    double maxA = 9.8;
    double targetV = 0;
    double horizon = 30;
    double buffer = 5;
    double laneWidth = 4;
    State state = INIT;
    map<State, vector<State>> fsm = {
        {INIT, {LK}},
        {LK, {LK, PLCL, PLCR}},
        {PLCL, {LK, PLCL, LCL}},
        {PLCR, {LK, PLCR, LCR}},
        {LCL, {LK, LCL}},
        {LCR, {LK, LCR}}
    };
    vector<string> fsmNames = {"INIT", "LK", "PLCL", "PLCR", "LCL", "LCR"};

    inline vector<double> toCartesian(double s, double d);
    inline vector<double> toFrenet(double x, double y, double theta);
    vector<OtherCar> filterCarsByLane(vector<OtherCar> &otherCars, int lane);
    vector<OtherCarPrediction> predictCars(vector<OtherCar> &otherCars, double t);
    OtherCarPrediction* findClosestCar(vector<OtherCarPrediction> &otherCars, double fromS, double toS);
    Kinematics keepLaneKinematics(Car &car, int currentPathSize, vector<OtherCar> &otherCars, PointSD &startSD, int startLane);
    Kinematics prepareLaneChangeKinematics(Car &car, int currentPathSize, vector<OtherCar> &otherCars, PointSD &startSD, int startLane, int deltaLane);
    Kinematics laneChangeKinematics(Car &car, int currentPathSize, vector<OtherCar> &otherCars, PointSD &startSD, int startLane, int deltaLane);
    double calcCost(Kinematics kinematics);
};

#endif // PATH_PLANNER_H
