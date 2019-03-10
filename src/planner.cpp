#include <math.h>
#include "planner.h"

using namespace std;

Planner::Planner(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s) {
    this->ref_vel = 0.0;
    this->target_lane = 1;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;

    this->kl_ts = 0;
}

void Planner::keepLane(Vehicle& car) {
    this->target_lane = car.lane();
}

void Planner::changeLane(int target_lane) {
    this->target_lane = target_lane;
    this->kl_ts = 0;
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
    double MAX_ACC = 0.22;
    double speed_diff = 0.0;

    int current_lane = ego_car.lane();
    
    vector<Vehicle> front_cars;
    vector<Vehicle> back_cars;
    for (int i = 0; i < 3; i++) {
        front_cars.push_back(r.findCarAtLane(i, ego_car, prev_size, true));
        back_cars.push_back(r.findCarAtLane(i, ego_car, prev_size, false));
    }

    double front_gap = 0;
    double left_front_gap = 0;
    double left_back_gap = 0;
    double right_front_gap = 0;
    double right_back_gap = 0;

    double is_close = false;

    Vehicle& car_ahead = front_cars[current_lane];

    if (car_ahead.getId() >= 0) {
        front_gap = abs(car_ahead.predictS(prev_size) - ego_car.getS());
        is_close = front_gap < 30;
    }

    if (current_lane > 0) {
        if (front_cars[current_lane-1].getId() >= 0) {
            left_front_gap = abs(front_cars[current_lane-1].predictS(prev_size) - ego_car.getS());
        }

        if (back_cars[current_lane-1].getId() >= 0) {
            left_back_gap = ego_car.getS() - back_cars[current_lane-1].predictS(prev_size);
        }
    }

    if (current_lane < 2) {
        if (front_cars[current_lane+1].getId() >= 0) {
            right_front_gap = abs(front_cars[current_lane+1].predictS(prev_size) - ego_car.getS());
        }
        if (back_cars[current_lane+1].getId() >= 0) {
            right_back_gap = ego_car.getS() - back_cars[current_lane+1].predictS(prev_size);
        }
    }

    printf("F:%.2f, LF: %.2f, LB: %.2f, RF: %.2f, RB: %.2f\n", front_gap, left_front_gap, left_back_gap, right_front_gap, right_back_gap);

    bool is_safe_left = current_lane > 0 && left_front_gap > 30 && left_back_gap > 30;
    bool is_safe_right = current_lane < 2 && right_front_gap > 30 && right_back_gap > 30;

    bool no_recent_lane_chane = kl_ts > 10;
    
    if (is_close) {
        int target_lane = ego_car.lane();            
        if (is_safe_left && is_safe_right) {
            target_lane = left_front_gap > right_front_gap ? current_lane - 1 : current_lane + 1;
        } else if (is_safe_left) {
            target_lane = current_lane - 1;
        } else if (is_safe_right) {
            target_lane = current_lane + 1;
        }

        if (no_recent_lane_chane && ego_car.lane() != target_lane) {
            printf("Changing lane from %d to %d\n", current_lane, target_lane);

            changeLane(target_lane);
            ref_vel -= MAX_ACC;
        } else {
            if (ref_vel > car_ahead.getV()) {
                // Slow down
                ref_vel -= MAX_ACC;
            }

            if (front_gap < 15) {
                speed_diff -= MAX_ACC/2;
            }

            kl_ts += 1;
        }
    } else {
        bool is_change_lane = (current_lane == 0 && is_safe_right) || (current_lane == 2 && is_safe_left);
        if (is_change_lane && no_recent_lane_chane) {
            printf("Changing lane from %d to %d\n", current_lane, 1);

            // if not in center lane try to move to center lane
            changeLane(1);
            ref_vel -= MAX_ACC;
        } else {
            kl_ts += 1;
        }
        
        double back_car_speed = back_cars[ego_car.lane()].getV();

        if (ref_vel < MAX_SPEED && !is_change_lane) {
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