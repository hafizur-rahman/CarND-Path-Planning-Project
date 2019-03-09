#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "vehicle.h"

using namespace std;

class Road {
    vector<Vehicle> left_lane;
    vector<Vehicle> center_lane;
    vector<Vehicle> right_lane;

public:
    Road(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane);
    ~Road(){};

    vector<Vehicle> getLaneStatus(int lane);

    Vehicle& findCarAtLane(int lane_id, Vehicle& ego_car, int ts, bool is_front);
    bool isSafeLaneChange(Vehicle& ego_car, int target_lane, int ts);
};

#endif
