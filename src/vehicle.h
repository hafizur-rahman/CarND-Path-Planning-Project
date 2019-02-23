#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

using namespace std;

class Vehicle {
protected:
    int id;
    double x;
    double y;
    double v;
    double s;
    double d;
    double yaw;

public:
    Vehicle();
    Vehicle(int id, double x, double y, double v, double s, double d);
    ~Vehicle(){};

    int getId();
    double getX();
    double getY();
    double getV();
    double getS();
    double getD();
    double getYaw();
    
    double predictS(int ts);

    int lane();
    void updateStatus(double x, double y, double v, double s, double d, double yaw);
};

#endif // VEHICLE_H
