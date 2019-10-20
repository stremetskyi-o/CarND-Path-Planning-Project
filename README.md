# **CarND-Path-Planning-Project**
## Project Writeup

### 1. Writeup

This writeup describes implementation steps that were taken to address project rubric points.

### 2. Implementation
Implementation started from creating separate **PathPlaner** class to encapsulate planning logic - **main.cpp** is now only responsible for calling **plan** function with corresponding arguments:

```c++
Car car = {car_x, car_y, car_s, car_d, car_yaw, car_speed};
vector<vector<double>> prev_vals = {previous_path_x, previous_path_y};
vector<OtherCar> otherCars;
for (auto &sf: sensor_fusion) {
    otherCars.push_back({sf[0], sf[1], sf[2], sf[3], sf[4], sf[5], sf[6]});
}
vector<vector<double>> next_vals = planner.plan(car, prev_vals, otherCars);
```

After some testing, I've found it easier to apply *Spline* trajectory generation over *Polynomial*. The main points were that I have failed to derive starting acceleration properly and that it required to use 2 sets of equations for S and D. Using Splines, however, only required to use initial yaw of the vehicle, *X* coordinate calculated as a fraction of distance and *Y* calculated from Spline function.

Spline calculation implemented in *path_planner.cpp* [lines 18-35](src/path_planner.cpp#L18-L35) and [lines 96-139](src/path_planner.cpp#L96-L139), it consists of several steps:

1. Get starting position and previous position from the previous path or calculate them from the vehicle's current position and yaw
2. After the target speed and lane are calculated, I fill `splineX`, `splineY` vectors with known previous points, then 2 new points are calculated in Frenet space to incorporate the change in D coordinate, the S is selected to be bigger than the distance that vehicle can travel.
3. With all points in place we need to shift coordinate system so the starting point appears at (0, 0), this will simplify further calculations:
```c++
double deltaX = splineX[i] - start.x;
double deltaY = splineY[i] - start.y;
splineX[i] = deltaX * cos(-theta) - deltaY * sin(-theta);
splineY[i] = deltaX * sin(-theta) + deltaY * cos(-theta);
```
4. Next we calculate a set of *X* coordinates with length between 50 minus the size of the previous path and amount of points traveled at max speed with 0.02 seconds increment. Using spline we then get *Y* coordinates and shift them both to the global coordinate system and return appended path to event loop for passing it to the simulator.

To choose between possible speeds and lanes, an FSM was used, and for each successor states, a cost was calculated. The FSM states are
* Keep Lane - go in the current lane with the max speed if possible or the speed of the closest car in front
* Prepare Lane Change Left and Right - go in the current lane and try to adjust the speed to the adjacent lane where the gap is present
* Lane Change Left and Right - execute lane changing while going at the same speed

The data from sensor fusion was extrapolated to predict where other vehicles should be at the end of Ego vehicle's current path. For every state, V, ΔS, ΔLane, Lane<sub>result</sub> were stored. These values are used for cost calculation and generating path points. For calculating cost I've selected 3 functions with weights 0.6, 0.3 and 0.1:
* Max speed cost: *1 - V<sub>lane</sub> / V<sub>max</sub>*
* Obstacle distance and speed cost: *1 - e<sup>-V<sub>lane</sub> / |ΔS|</sup>*
* Lane change cost: *1 - e<sup>-|ΔLane|</sup>*
The selected functions mean that we want to travel at max speed, have a gap between obstacles which should increase with speed increase and don't want to change lanes if everything else is equal;

### 3. Results
The car can drive smoothly on the road, keep the desired speed, and pass other vehicles while sticking to the set of restrictions.

### 4. Discussion
Some situations can make the car to choose the solution that is not ideal:
* When all 3 lanes in front of the Ego vehicle are occupied at certain conditions, the vehicle may decide to switch lanes back and force even though changing lanes has a higher cost, but they can become equal when one of the vehicles in front starts accelerating and then decelerating or vice versa.
* When Ego vehicle is following another vehicle in the leftmost or rightmost lane and decides to pass it, it can be blocked by the vehicle in the adjacent lane and stuck in this state as long as both vehicles speeds are equal

The first drawback can be addressed by penalizing frequent lane changes, and the second by either biasing Ego vehicle position to the center lane to have more room for maneuvers or by checking all lanes to make a better decision.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Original ReadMe
Original ReadMe is available [here](README-Udacity.md)