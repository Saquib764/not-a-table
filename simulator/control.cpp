#include <iostream>
#include <cmath>
using namespace std;

// #include "model.cpp"

#include "../src/arm_model.cpp"

#define MICROSTEPS                64
#define STEPS_PER_REV             200
#define PI                        3.14159265358979323846

double K = STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
double R = 0.63/2;


ArmModel arm = ArmModel(stepper1, stepper2, R, K);

int point_count = 0;

// Define constants
const int MAX_SPEED = 1000;
const int MAX_ACCELERATION = 3 * MAX_SPEED;

// Define global variables
int current_target_indexes[2] = {2, 2};

// Define target arrays
double keypoints[5][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
double targets[5][2] =   {{0.0, 0.0}, {0., 0.0}, {0., 0.0}, {0., 0.0}, {0., 0.0}};
double target_speeds_dir[4] = {0.0, 0.0, 0.0, 0.0};
double target_directions[4][2] = {{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}};
double angles_at_keypoints[4] = {0, 0, 0, 0};
double max_speeds[4][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
bool should_stop[4] = {false, false, false, false};

double target_speeds[2] = {0.0, 0.0};

double error = 0;
double last_error = 0;
double speed_integral[2] = {0.0, 0.0};

void to_xy(double a1, double a2, double& x, double& y) {
  x = R * cos(a1) + R * cos(a1 + a2);
  y = R * sin(a1) + R * sin(a1 + a2);
}

// Function to follow the trajectory
int follow_trajectory() {
  if (current_target_indexes[0] >= 5 && current_target_indexes[1] >= 5) {
    return 2;
  }
  // wait for other index to catch up
  if(current_target_indexes[0] >= 5) {
    current_target_indexes[0] = 4;
  }
  if(current_target_indexes[1] >= 5) {
    current_target_indexes[1] = 4;
  }

  int current_target_index = max(current_target_indexes[0], current_target_indexes[1]);

  error = 0;
  double current_acceleration[2] = {0, 0};
  double current_position[2] = {0, 0};
  arm.getJointPositionInSteps( current_position );

  double current_speed[2] = {0, 0};
  arm.getJointSpeedInSteps( current_speed );


  double displacement_to_target[2] = {
    targets[current_target_index][0] - current_position[0],
    targets[current_target_index][1] - current_position[1]
  };
  double distance_to_go[2] = {
    displacement_to_target[0] * target_directions[current_target_index - 1][0],
    displacement_to_target[1] * target_directions[current_target_index - 1][1]
  };


  target_speeds[0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  target_speeds[1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(target_speeds[1]) > MAX_SPEED) {
    target_speeds[0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    target_speeds[1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }

  target_speeds[0] = max_speeds[current_target_indexes[0] - 1][0];
  target_speeds[1] = max_speeds[current_target_indexes[1] - 1][1];

  // cout << "ola: " << displacement_to_target[0] * target_directions[current_target_indexes[0] - 1][0] << " " << displacement_to_target[1] * target_directions[current_target_indexes[1] - 1][1] << endl << endl;

  bool has_reached[2] = {false, false};
  if (distance_to_go[0] < 2.0 ) {
    has_reached[0] = true;
  }
  if (distance_to_go[1] < 2.0) {
    has_reached[1] = true;
  }

  if (has_reached[0] && has_reached[1]) {
    current_target_indexes[0]++;
    current_target_indexes[1]++;
    return 1;
  }

  double original_displacement[2] = {
    targets[current_target_indexes[0]][0] - targets[current_target_indexes[0] - 1][0],
    targets[current_target_indexes[0]][1] - targets[current_target_indexes[0] - 1][1]
  };

  double speed_adjust[2] = {0., 0.};
  // if(current_target_indexes[0] > current_target_indexes[1]) {
  //   speed_adjust[0] = - 0.3 * current_speed[0];
  //   speed_adjust[1] =   0.3 * current_speed[1];
  // }
  // if(current_target_indexes[0] < current_target_indexes[1]) {
  //   speed_adjust[0] =   0.3 * current_speed[0];
  //   speed_adjust[1] =  -0.3 * current_speed[1];
  // }

  if (abs(original_displacement[0]) > 5) {
    // Correct the motor with the smaller distance
    double r1 = abs((current_position[0] - targets[current_target_indexes[0] - 1][0]) / original_displacement[0]);
    
    double expected_position = targets[current_target_indexes[0] - 1][1] + r1 * original_displacement[1];
    error = expected_position - current_position[1];

    // error = max((double)-10.0, min((double)10., error));

    if( error > 1) {
      speed_adjust[1] = max((double)-10.0, min((double)10., error)) * 0.8;
      last_error = error;
      // speed_adjust[0] = -error * 0.1 ;
      // current_acceleration[1] += speed_adjust[1];
    }else if(error < -1) {
      speed_adjust[0] = -max((double)-10.0, min((double)10., error)) * 0.8 ;
      last_error = error;
    }
  }

  double next_speed[2] = {0.0, 0.0};
  if(current_target_indexes[0] < 4 && current_target_indexes[1] < 4) {
    next_speed[0] = max_speeds[current_target_indexes[0]][0];
    next_speed[1] = max_speeds[current_target_indexes[1]][1];
  }

  double D = 10.0;
  if (distance_to_go[0] < D  && distance_to_go[0] > 0) {
    double t = max(0.0, min(distance_to_go[0] / D, 1.0));
    // target_speeds[0] = target_speeds[0] + (next_speed[0] - target_speeds[0]) * (1 - t) * 0.5  ;
  }

  if ( distance_to_go[1] < D && distance_to_go[1] >0) {
    double t = max(0.0, min(distance_to_go[1] / D, 1.0));
    cout << "t: " << t << endl;
    // target_speeds[1] = target_speeds[1] + (next_speed[1] - target_speeds[1]) * (1 - t)  * 0.5 ;
  }

  if( has_reached[0] ) {
    target_speeds[0] = 0.0;
    speed_adjust[0] = 0.0;
  }
  if( has_reached[1] ) {
    target_speeds[1] = 0.0;
    speed_adjust[1] = 0.0;
  }
  
  speed_integral[0] += target_speeds[0] - current_speed[0];
  speed_integral[1] += target_speeds[1] - current_speed[1];
  speed_integral[0] = max(-10.0, min(10.0, speed_integral[0]));
  speed_integral[1] = max(-10.0, min(10.0, speed_integral[1]));

  // if(abs(target_speeds[0] - current_speed[0]) < 5) {
  //   speed_integral[0] = 0.0;
  // }
  // if(abs(target_speeds[1] - current_speed[1]) < 5) {
  //   speed_integral[1] = 0.0;
  // }
  
  // speed_adjust[0] = 0.0;
  // speed_adjust[1] = 0.0;
  current_acceleration[0] = (target_speeds[0] + speed_adjust[0] - current_speed[0]) * 5.0;
  current_acceleration[1] = (target_speeds[1] + speed_adjust[1] - current_speed[1]) * 5.0;
  
  // Last steps push
  double LD = 40.0;
  if (distance_to_go[0] < LD  && !has_reached[0] && abs(current_speed[0]) < 0.1) {
    current_acceleration[0] = displacement_to_target[0] * 2.0;
  }

  // current_acceleration[0] = max(current_acceleration[0], 0.1);
  // arm.setSpeedInHz( target_speeds[0], target_speeds[1] );
  // setSpeedInHz( abs(_max_speeds[0]), abs(_max_speeds[1]) );

  arm.moveByAcceleration(current_acceleration[0], current_acceleration[1]);

  if(true) {
    cout << "distance to target: " << displacement_to_target[0] * target_directions[current_target_indexes[0] - 1][0]<< ", " << displacement_to_target[1] * target_directions[current_target_indexes[1] - 1][1] << endl;
    cout << "has reached: " << has_reached[0]<< ", " << has_reached[1] << endl;
    cout << "max speed: " << max_speeds[current_target_indexes[0] - 1][0] << ", " << max_speeds[current_target_indexes[1] - 1][1] << endl;
    cout << "next max: " << next_speed[0] << ", " << next_speed[1] << endl;
    cout << "target speed: " << target_speeds[0] << ", " << target_speeds[1] << endl;
    cout << "current speed: " << current_speed[0] << ", " << current_speed[1] << endl;
    cout << "current acceleration: " << current_acceleration[0] << ", " << current_acceleration[1] - current_acceleration[0] << endl;
    cout << "current target index: " << current_target_indexes[0] << " " << current_target_indexes[1] << endl << endl;
    cout << "point count: " << point_count << endl;
  }
  // do nothing, chasing target
  return 0;
}

// Function to add a point to the trajectory
void add_point_to_trajectory(double a1, double a2) {
  point_count++;
  double _pt[2];
  cout << "adding point: " << a1 << ", " << a2 << endl;
  to_xy(a1, a2, _pt[0], _pt[1]);
  double _lpt[2];
  to_xy(keypoints[4][0], keypoints[4][1], _lpt[0], _lpt[1]);
  double angle_to_new_point = atan2(_pt[1] - _lpt[1], _pt[0] - _lpt[0]) * 180.0 / PI;
  double target_speed_to_new_point = -1;

  if (abs(angle_to_new_point - angles_at_keypoints[3]) > 20) {
    // Stop
    target_speed_to_new_point = 0;
  }

  // Shift all points to the left
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      keypoints[i][j] = keypoints[i + 1][j];
      targets[i][j] = targets[i + 1][j];
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      target_directions[i][j] = target_directions[i + 1][j];
      max_speeds[i][j] = max_speeds[i + 1][j];
    }
    angles_at_keypoints[i] = angles_at_keypoints[i + 1];
    target_speeds_dir[i] = target_speeds_dir[i + 1];
    should_stop[i] = should_stop[i + 1];
  }

  // Add new point to the end
  keypoints[4][0] = a1;
  keypoints[4][1] = a2;

  targets[4][0] = int(3 * a1 * K);
  targets[4][1] = int(9 * a2 * K);
  target_speeds_dir[3] = target_speed_to_new_point;

  if (keypoints[4][0] - keypoints[3][0] > 0) {
    target_directions[3][0] = 1;
  } else if (keypoints[4][0] - keypoints[3][0] < 0) {
    target_directions[3][0] = -1;
  }

  if (keypoints[4][1] - keypoints[3][1] > 0) {
    target_directions[3][1] = 1;
  } else if (keypoints[4][1] - keypoints[3][1] < 0) {
    target_directions[3][1] = -1;
  }

  current_target_indexes[0]--;
  current_target_indexes[1]--;

  angles_at_keypoints[3] = angle_to_new_point;

  if (target_directions[3][0] != target_directions[2][0] || target_directions[3][1] != target_directions[2][1]) {
    should_stop[3] = true;
  } else {
    should_stop[3] = target_speed_to_new_point != 0;
  }

  double displacement_to_target[2] = {
    targets[4][0] - targets[3][0],
    targets[4][1] - targets[3][1]
  };
  max_speeds[3][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  max_speeds[3][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(max_speeds[3][1]) > MAX_SPEED) {
    max_speeds[3][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    max_speeds[3][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }
  double current_position[2] = {0, 0};
  arm.getJointPositionInSteps( current_position );
  if(true) {
    // cout targets array
    cout << "targets: ";
    for (int i = 0; i < 5; i++) {
      cout << targets[i][0] << ", " << targets[i][1] << " | ";
    }
    cout << endl;

    // cout target_directions array
    cout << "target_directions: ";
    for (int i = 0; i < 4; i++) {
      cout << target_directions[i][0] << ", " << target_directions[i][1] << " | ";
    }
    cout << endl;

    // cout max_speeds array
    cout << "max_speeds: ";
    for (int i = 0; i < 4; i++) {
      cout << max_speeds[i][0] << ", " << max_speeds[i][1] << " | ";
    }
    cout << endl;

    // cout should_stop array
    cout << "should_stop: ";
    for (int i = 0; i < 4; i++) {
      cout << should_stop[i] << " | ";
    }
    cout << endl;

    // cout angles_at_keypoints array
    cout << "angles_at_keypoints: ";
    for (int i = 0; i < 4; i++) {
      cout << angles_at_keypoints[i] << " | ";
    }
    cout << endl;

    // cout current_position
    cout << "current_position: " << current_position[0] << ", " << current_position[1] << endl;
    // cout current_target_index
    cout << "current_target_index: " << current_target_indexes[0] << " " << current_target_indexes[1] << endl << endl;

  }
}

void move() {
  stepper1->move();
  stepper2->move();
}

bool is_hall_sensor_detected = false;
double position_at_max_speed = 0.0;
double max_hall_value = 0.0;
double homing_started_at_angle = 0.0;
bool is_homing = false;
bool has_started_in_hall_region = false;
void home() {
  if(!arm.is_homed[0]) {
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    arm.getJointPositionInRadians(pos);
    arm.getJointPositionInSteps(pos_steps);

    double value = hall1.readValue( pos[0] ) - 2000.0;
    for(int i = 1; i<5; i++) {
      value += hall1.readValue( pos[0] ) - 2000.0;
    }
    value /= 5.0;

    if(!is_homing) {
      // start homing
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      if( value > 100.0 ) {
        // get out of hall region
        arm.setSpeedInHz(100.0, 100.0);
        arm.moveByAcceleration(-500.0, 500.0);
        has_started_in_hall_region = true;
      }else{
        arm.setSpeedInHz(200.0, -200.0);
        arm.moveByAcceleration(500.0, -500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      arm.setSpeedInHz(200.0, -200.0);
      arm.moveByAcceleration(500.0, -500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        cout << "value1: " << value << endl;
        arm.setSpeedInHz(50.0, -50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[0];
        max_hall_value = value;
        homing_started_at_angle = pos[0];
      }
      if(is_hall_sensor_detected && abs(pos[0] - homing_started_at_angle) > 45.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        cout << "max at : " << position_at_max_speed << endl;
        arm.stepper1->moveToPositionInSteps(position_at_max_speed);
        arm.is_homed[0] = true;
        is_hall_sensor_detected = false;
        is_homing = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  }else if(!arm.is_homed[1]) {
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    arm.getJointPositionInRadians(pos);
    arm.getJointPositionInSteps(pos_steps);

    double value = hall1.readValue( pos[1] ) - 2000.0;
    for(int i = 1; i<5; i++) {
      value += hall1.readValue( pos[1] ) - 2000.0;
    }
    value /= 5.0;

    if(!is_homing) {
      // start homing
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      if( value > 100.0 ) {
        // get out of hall region
        arm.setSpeedInHz(0.0, -100.0);
        arm.moveByAcceleration(0.0, -500.0);
        has_started_in_hall_region = true;
      }else{
        arm.setSpeedInHz(0.0, 200.0);
        arm.moveByAcceleration(0.0, 500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      arm.setSpeedInHz(0.0, 200.0);
      arm.moveByAcceleration(0.0, 500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        cout << "value1: " << value << endl;
        arm.setSpeedInHz(0.0, 50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[1];
        max_hall_value = value;
        homing_started_at_angle = pos[1];
      }
      if(is_hall_sensor_detected && abs(pos[1] - homing_started_at_angle) > 45.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        cout << "max at : " << position_at_max_speed << endl;
        arm.stepper2->moveToPositionInSteps(position_at_max_speed);
        arm.is_homed[1] = true;
        is_hall_sensor_detected = false;
        is_homing = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  }

}