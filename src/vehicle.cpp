#include "vehicle.h"

using namespace std;

Vehicle::Vehicle() {
    this->id = -1;
}

Vehicle::Vehicle(int id, double x, double y, double v, double s, double d) {
    this->id = id;
    this->x = x;
    this->y = y;
    this->v = v;
    this->s = s;
    this->d = d;
}
    

int Vehicle::getId() {
    return this->d;
}

double Vehicle::getX(){
    return this->x;
}

double Vehicle::getY(){
    return this->y;
}

double Vehicle::getV(){
    return this->v;
}

double Vehicle::getS(){
    return this->s;
}

double Vehicle::getD(){
    return this->d;
}

double Vehicle::getYaw(){
    return this->yaw;
}

int Vehicle::lane() {
    int lane = this->d / 4;

    return (this-> d > 0 && this->d < 12) ? lane : -1;
}

void Vehicle::updateStatus(double x, double y, double v, double s, double d, double yaw) {
    this->x = x;
    this->y = y;
    this->v = v;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
}

double Vehicle::predictS(int ts) {
    return s + ts * 0.02 * v;
}