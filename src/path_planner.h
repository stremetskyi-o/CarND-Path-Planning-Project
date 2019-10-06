#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

using std::vector;

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

    inline vector<double> toCartesian(double s, double d);
};

#endif // PATH_PLANNER_H
