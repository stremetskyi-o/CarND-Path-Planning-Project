#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

using std::vector;

struct Car {
        double x, y, s, d, yaw, speed;
    };

class PathPlanner {

    public:

    vector<vector<double>> plan(Car &car, vector<vector<double>> &prevPath);
};

#endif // PATH_PLANNER_H
