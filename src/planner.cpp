#include <math.h>
#include "planner.h"

using namespace std;

Planner::Planner(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s) {
    this->ref_vel = 0.0;
    this->target_lane = 1;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}

void Planner::keepLane(Vehicle& car) {
    this->target_lane = car.lane();
}

void Planner::changeLane(int target_lane) {
    this->target_lane = target_lane;
}

tk::spline Planner::makeSpline(Vehicle& ego_car, vector<double>& previous_path_x, vector<double>& previous_path_y) {
    vector<double> x;
    vector<double> y;

    int prev_size = previous_path_x.size();

    if (previous_path_x.size() < 2) {
        double prev_car_x = ego_car.getX() - cos(ego_car.getYaw());
        double prev_car_y = ego_car.getY() - sin(ego_car.getYaw());

        x.push_back(prev_car_x);
        x.push_back(ego_car.getX());

        y.push_back(prev_car_y);        
        y.push_back(ego_car.getY());
    } else {
        x.push_back(previous_path_x[prev_size - 2]);
        x.push_back(previous_path_x[prev_size - 1]);

        y.push_back(previous_path_y[prev_size - 2]);
        y.push_back(previous_path_y[prev_size - 1]);
    }

    // Target points in the future in freenet coordinate
    double dist[] = { 30, 60, 90};
    for (int i = 0; i < 3; i ++) {
        vector<double> nextWayPoints = getXY(
            ego_car.getS() + dist[i],
            2 + 4 * target_lane,
            map_waypoints_s,
            map_waypoints_x,
            map_waypoints_y
        );

        x.push_back(nextWayPoints[0]);
        y.push_back(nextWayPoints[1]);
    }

    // Now transform to car coordinate
    ref_x = ego_car.getX();
    ref_y = ego_car.getY();
    ref_yaw = deg2rad(ego_car.getYaw());

    if (prev_size > 2) {
        ref_x = previous_path_x[prev_size -1];
        ref_y = previous_path_y[prev_size -1];

        double ref_x_prev = previous_path_x[prev_size -2];
        double ref_y_prev = previous_path_y[prev_size -2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    }

    for (int i = 0; i < x.size(); i++) {
        double shift_x = x[i] - ref_x;
        double shift_y = y[i] - ref_y;

        x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline s;
    s.set_points(x, y);

    return s;
}

void Planner::plan(
    Road& r, Vehicle& ego_car, 
    vector<double>& previous_path_x,
    vector<double>& previous_path_y,
    vector<double>& next_x_vals,
    vector<double>& next_y_vals
) {
    int prev_size = previous_path_x.size();

    double MAX_SPEED = 49.5;
    double MAX_ACC = 0.224;
    double speed_diff = 0.0;

    int current_lane = ego_car.lane();

    bool is_car_ahead = r.isCarAhead(ego_car, prev_size);
    bool is_safe_left = r.isSafeLaneChange(ego_car, current_lane - 1, prev_size);
    bool is_safe_right = r.isSafeLaneChange(ego_car, current_lane + 1, prev_size);

    if (is_car_ahead) {
        if (is_safe_left) {
            changeLane(current_lane - 1);
        } else if (is_safe_right) {
            changeLane(current_lane + 1);
        } else {            
            // TODO maintain lane speed
            speed_diff -= MAX_ACC;            
        }
    } else {
        if ((current_lane == 0 && is_safe_right) || (current_lane == 2 && is_safe_left)) {
            // if not in center lane try to move to center lane
            changeLane(1);
        }
        
        if (ref_vel < MAX_SPEED) {
            speed_diff += MAX_ACC;
        }
    }

    // Create spline
    tk::spline s = makeSpline(ego_car, previous_path_x, previous_path_y);

    // Start with all the previous path points from last time
    for (int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Compute how to break up spline points so we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    // Fill up the rest of the path planner to always output 50 points
    for (int i = 1; i < 50 - prev_size; i++) {
        ref_vel += speed_diff;
        
        if (ref_vel > MAX_SPEED) {
            ref_vel = MAX_SPEED;
        } else if (ref_vel < MAX_ACC) {
            ref_vel = MAX_ACC;
        }

        double N = (target_dist/(.02* ref_vel/2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}