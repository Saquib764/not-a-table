#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

#include "hardware/motor_model.cpp"


MotorModel motor1 = MotorModel();
MotorModel motor2 = MotorModel();


MotorModel *stepper1 = &motor1;
MotorModel *stepper2 = &motor2;


#include "control.cpp"


bool read_line(ifstream& file, double* value) {
    string line;
    while(true) {
      if(!getline(file, line)) {
          return false;
      }
      if(line[0] != '#') {
          break;
      }
    }
    istringstream iss(line);
    string token;
    getline(iss, token, ' ');
    double x;
    try {
        x = stof(token);
    } catch (const exception& e) {
        cerr << "Error converting string to double: " << token<< e.what() << endl;
    }
    getline(iss, token, ' ');
    double y;
    try {
        y = stof(token);
    } catch (const exception& e) {
        cerr << "Error converting string to double: " << e.what() << endl;
    }
    value[0] = x;
    value[1] = y;
    return true;
}

int main() {
    string filename = "../test_designs/square.thr.txt"; // Replace "example.txt" with the desired file name

    ofstream fileout( "output.txt" );
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Error opening the file: " << filename << endl;
        return 1;
    }

    double pt[2];
    int T = 1000000;

    double ps[2];
    double v[2];
    double a[2];

    for(int i = 0; i < T; i++) {
      // cout<< "i: " << i << endl;
      move();
      arm.getJointPositionInRadians(ps);
      arm.getJointSpeedInSteps(v);
      arm.getJointAccelerationInSteps(a);
      // cout << "p: " << ps[0] << " " << ps[1] << " v " << v[0] << " " << v[1] << " a: " << a[0] << " " << a[1] << endl;
      fileout << ps[0] << " " << ps[1] << " " << v[0] << " " << v[1] << " " << a[0] << " " << a[1] << " " << target_speeds[0] << " " << target_speeds[1] << " " << max_speeds[2][0] << " " << max_speeds[2][1] << " " << error << endl;
      if(i%10 !=0) {
        continue;
      }
      int should_read_next = follow_trajectory();
      if(should_read_next == 1) {
        bool has_value = read_line(file, pt);
        if(!has_value) {
            continue;
        }

        add_point_to_trajectory(pt[0], pt[1]);
      }
      if(should_read_next == 2) {
        // design complete
        cout << "Design complete" << endl;
        break;
      }
    }

    file.close();
    fileout.close();

    return 0;
}