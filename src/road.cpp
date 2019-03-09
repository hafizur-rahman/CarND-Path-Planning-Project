#include "road.h"
#include <stdio.h>

Road::Road(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane) {
    this->left_lane = left_lane;
    this->center_lane = center_lane;
    this->right_lane = right_lane;
}

vector<Vehicle> Road::getLaneStatus(int lane) {
    if (lane == 0) {
        return this->left_lane;
    } else if (lane == 1) {
        return this->center_lane;
    } else /* if (lane == 2) */ {
        return this->right_lane;
    }
}

Vehicle& Road::findCarAhead(Vehicle& ego_car, int ts) {
    int current_lane = ego_car.lane();

    vector<Vehicle> lane_status = getLaneStatus(current_lane);
           
    int closest_car_idx = -1;
    float min_dist = 999999;
    Vehicle car_ahead = Vehicle();

    for (int i = 0; i < lane_status.size(); i++) {
        Vehicle& v = lane_status[i];
        
        float dist = v.predictS(ts) - ego_car.getS();
        if (dist > 0 && dist < min_dist) {            
            min_dist = dist;
            closest_car_idx = i;
        }
    }

    return closest_car_idx >= 0 ? lane_status[closest_car_idx] : car_ahead;
}

bool Road::isSafeLaneChange(Vehicle& ego_car, int target_lane, int ts) {
    if (ego_car.lane() == target_lane || target_lane < 0 || target_lane > 2) {
        return false;
    }

    bool unsafe = false;
    vector<Vehicle> lane_status = getLaneStatus(target_lane);
    for (int i = 0; i < lane_status.size(); i++) {
        Vehicle& v = lane_status[i];       

        bool is_close = ((ego_car.getS() - 30) < v.predictS(ts) && (ego_car.getS() + 30) > v.predictS(ts));        

        unsafe |= is_close;
    }

    return !unsafe;
}