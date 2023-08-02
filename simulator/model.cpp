#include <iostream>

using namespace std;

double __target_speed[2] = {0.0, 0.}; 
double __current_acceleration[2] = {0., 0.};

double current_position[2] = {0., 0.};
double __current_speed[2] = {0., 0.};
void setSpeedInHz(double speed1, double speed2) {
  __target_speed[0] = speed1 * 1000;
  __target_speed[1] = speed2 * 1000;
}
void moveByAcceleration(double acceleration1, double acceleration2) {
  __current_acceleration[0] = acceleration1;
  __current_acceleration[1] = acceleration2;
}
double getCurrentPosition(int i) {
  return current_position[i] / 1000.0;
}
double getCurrentSpeedInMilliHz(int i) {
  return __current_speed[i];
}
double getCurrentAcceleration(int i) {
  return __current_acceleration[i];
}

double dt=0.01;

void move() {

  // cout<< "speed1: " << __current_speed[0] / 1000.0 << " speed2: " << __current_speed[1] / 1000. << endl;

  __current_speed[0] += __current_acceleration[0] * dt * 1000.0;
  __current_speed[1] += __current_acceleration[1] * dt * 1000.0;

  // cout<< "speed2: " << __current_speed[0] / 1000.0 << ", " << __current_speed[1] / 1000.0 << endl;

  // if( __current_speed[0] > 0 && __current_speed[0] > __target_speed[0] ) {
  //   __current_speed[0] = __target_speed[0];
  // }else if( __current_speed[0] < 0 && __current_speed[0] < -__target_speed[0] ) {
  //   __current_speed[0] = -__target_speed[0];
  // }
  // if( __current_speed[1] > 0 && __current_speed[1] > __target_speed[1] ) {
  //   __current_speed[1] = __target_speed[1];
  // }else if( __current_speed[1] < 0 && __current_speed[1] < -__target_speed[1] ) {
  //   __current_speed[1] = -__target_speed[1];
  // }

  current_position[0] += round(__current_speed[0] * dt);
  current_position[1] += round(__current_speed[1] * dt);

  // current_position[0] += __target_speed[0] * dt;
  // current_position[1] += __target_speed[1] * dt;

  // cout << "pos: " << current_position[0]/1000 << " " << current_position[1]/1000 << endl;
  // cout << "speed: " << __target_speed[0]/1000.0 << " " << __target_speed[1]/1000.0 << endl;
  // cout << "max speed: " << __target_speed[0]/1000.0 << " " << __target_speed[1]/1000.0 << endl;
}