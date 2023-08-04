#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

#include "hardware/motor_model.cpp"
#include "hardware/hall_model.cpp"


MotorModel motor1 = MotorModel();
MotorModel motor2 = MotorModel();

HallModel hall1 = HallModel(60.0 * 3.14/180.0, 20.0);
HallModel hall2 = HallModel(60.0 * 3.14/180.0, 20.0);

MotorModel *stepper1 = &motor1;
MotorModel *stepper2 = &motor2;


#include "control.cpp"

int main() {
    ofstream fileout( "output.txt" );

    hall1.setPosition(0.0);
    hall2.setPosition(0.0);

    double pt[2];
    int T = 250000;

    double ps[2];
    double v[2];
    double a[2];
    arm.setRandomPosition();

    int count = -1;
    for(int i = 0; i < T; i++) {
      // cout<< "i: " << i << endl;
      move();
      arm.getJointPositionInRadians(ps);
      arm.getJointSpeedInSteps(v);
      arm.getJointAccelerationInSteps(a);
      // cout << "p: " << ps[0] << " " << ps[1] << " v " << v[0] << " " << v[1] << " a: " << a[0] << " " << a[1] << endl;
      fileout << ps[0] << " " << ps[1] << " " << v[0] << " " << v[1] << " " << a[0] << " " << a[1] << " " << target_speeds[0] << " " << target_speeds[1] << " " << max_speeds[2][0] << " " << max_speeds[2][1] << " " << error << endl;
      if(i%4 !=0) {
        continue;
      }
      if(arm.isHomed()) {
        count = i;
      }
      if(count != -1 && i-count > 400) {
        break;
      }
      home();
    }

    fileout.close();

    return 0;
}
