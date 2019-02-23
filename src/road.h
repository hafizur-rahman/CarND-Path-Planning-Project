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

    bool isCarAhead(Vehicle& ego_car, int ts);
    bool isSafeLaneChange(Vehicle& ego_car, int target_lane, int ts);
};

#endif
