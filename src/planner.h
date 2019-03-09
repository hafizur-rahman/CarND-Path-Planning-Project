#ifndef PLANNER_H
#define PLANNER_H

#include "helpers.h"
#include "spline.h"
#include "vehicle.h"
#include "road.h"

class Planner {
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    int state;

    int target_lane;
    double ref_x;
    double ref_y;
    double ref_yaw;
    double ref_vel;

    int kl_ts;

protected:
    void keepLane(Vehicle& car);
    void changeLane(int target_lane);    

    tk::spline makeSpline(Vehicle& ego_car, vector<double>& previous_path_x, vector<double>& previous_path_y);

public:
    Planner(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s);
    ~Planner(){};

    void plan(
        Road& r, Vehicle& ego_car,
        vector<double>& previous_path_x,
        vector<double>& previous_path_y,
        vector<double>& next_x_vals,
        vector<double>& next_y_vals);
};

#endif