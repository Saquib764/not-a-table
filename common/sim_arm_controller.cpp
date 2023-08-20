#include "../include/sim_arm_controller.h"

double MAX_ACCELERATION = 600.0;
int MAX_POINTS = 5;

SimArmController::SimArmController(SimArmModel *arm){
  this->arm = arm;

  // Define constants
  MAX_SPEED = 600;
  MAX_ACCELERATION = 3 * MAX_SPEED;

  has_started = false;
  has_finished = false;
  has_all_targets = false;
  
  // Define global variables
  current_target_indexes[0] = 2;
  current_target_indexes[1] = 2;

  // Define target arrays
  reset();

  target_speeds[0] = 0.0;
  target_speeds[1] = 0.0;

  error[0] = 0.0;
  error[1] = 0.0;
}

double SimArmController::mod(double x, double y){
  double r = x - y * floor(x/y);
  if(r < 0) {
    r += y;
    r = mod(r, y);
  }
  return r;
}

void SimArmController::reset(){
  has_started = false;
  has_finished = false;
  has_all_targets = false;

  // Reinitialise trajectory to zero
  for(int i = 0; i < MAX_POINTS; i++) {
    targets[i][0] = 0;
    targets[i][1] = 0;
    keypoints[i][0] = 0;
    keypoints[i][1] = 0;
    time_at_keypoints[i] = 0;
  }
  for(int i = 0; i < MAX_POINTS-1; i++) {
    target_speeds_dir[i] = 0;
    target_directions[i][0] = 0;
    target_directions[i][1] = 0;
    angles_at_keypoints[i] = 0;
    max_speeds[i][0] = 0;
    max_speeds[i][1] = 0;
    should_stop[i] = false;
  }
}

int SimArmController::get_current_target_index(double t){
  // Get time index
  int time_index = -1;
  for(int i = 0; i < MAX_POINTS-1; i++) {
    if(time_at_keypoints[i] <= t && t < time_at_keypoints[i + 1]) {
      time_index = i;
      break;
    }
  }
  if(time_index == -1) {
    time_index = MAX_POINTS-1;
  }
  return time_index;
}

void SimArmController::get_target_speed(double t, double *speeds){
  // Get time index
  int time_index = get_current_target_index(t);
  if(time_index == MAX_POINTS - 1) {
    speeds[0] = max_speeds[time_index][0];
    speeds[1] = max_speeds[time_index][1];
    return;
  }

  // Smoothen speed
  double T = 0.15;
  double delta_t = 0.0;
  if(time_at_keypoints[time_index + 1] - t < T/2) {
    delta_t = 0.5 - (time_at_keypoints[time_index + 1] - t) / T;
    speeds[0] = max_speeds[time_index][0] * (1 - delta_t) + max_speeds[time_index + 1][0] * delta_t;
    speeds[1] = max_speeds[time_index][1] * (1 - delta_t) + max_speeds[time_index + 1][1] * delta_t;
  }
  if(t - time_at_keypoints[time_index] < T/2) {
    delta_t = (t - time_at_keypoints[time_index] + T/2) / T;
    speeds[0] = max_speeds[time_index][0] *  delta_t + max_speeds[time_index - 1][0] * (1 - delta_t);
    speeds[1] = max_speeds[time_index][1] * delta_t + max_speeds[time_index - 1][1] * (1 - delta_t);
  }
}

void SimArmController::get_target_position(double t, double *target_position){
  // Get time index
  int time_index = get_current_target_index(t);
  if(time_index == MAX_POINTS - 1) {
    target_position[0] = targets[time_index][0] ;
    target_position[1] = targets[time_index][1] ;
    return;
  }
  double delta_t = (t - time_at_keypoints[time_index]) / (time_at_keypoints[time_index + 1] - time_at_keypoints[time_index]);

  // Get target position
  target_position[0] = targets[time_index][0] * (1 - delta_t) + targets[time_index + 1][0] * delta_t;
  target_position[1] = targets[time_index][1] * (1 - delta_t) + targets[time_index + 1][1] * delta_t;
}

void SimArmController::get_target_acceleration(double t, double *accelerations) {
  int time_index = get_current_target_index(t);
  accelerations[0] = 0;
  accelerations[1] = 0;
  if(time_index == 4) {
    return;
  }

  double T = 0.1;
  if(time_at_keypoints[time_index + 1] - t < T/2) {
    accelerations[0] = (max_speeds[time_index +1][0] - max_speeds[time_index][0]) / T ;
    accelerations[1] = (max_speeds[time_index +1][1] - max_speeds[time_index][1]) / T ;
  }
  if(t - time_at_keypoints[time_index] < T/2) {
    accelerations[0] = (max_speeds[time_index][0] - max_speeds[time_index - 1][0]) / T ;
    accelerations[1] = (max_speeds[time_index][1] - max_speeds[time_index - 1][1]) / T ;
  }
}

int SimArmController::follow_trajectory() {
  double t = (micros() - start_time) / 1000000.0;
  int current_target_index = get_current_target_index(t) + 1;
  if (current_target_index >= MAX_POINTS && has_all_targets) {
    has_finished = true;
    target_speeds[0] = 0.0;
    target_speeds[1] = 0.0;
    arm->stopMove();
    return 2;
  }

  if(current_target_index > 2 && !has_all_targets) {
    return 1;
  }
  double current_acceleration[2] = {0, 0};
  double current_position[2] = {0, 0};
  arm->getJointPositionInSteps( current_position );

  double current_speed[2] = {0, 0};
  arm->getJointSpeedInSteps( current_speed );


  get_target_speed(t, target_speeds);

  double expected_position[2];
  get_target_position(t, expected_position);

  double expected_acceleration[2];
  get_target_acceleration(t, expected_acceleration);

  error[0] = expected_position[0] - current_position[0];
  error[1] = expected_position[1] - current_position[1];

  double speed_adjust[2];
  speed_adjust[0] = 5 * error[0];
  speed_adjust[1] = 5 * error[1];


  // speed_adjust[0] = 0.0;
  // speed_adjust[1] = 0.0;
  current_acceleration[0] = (target_speeds[0] + speed_adjust[0] - current_speed[0]) * 7.0 + expected_acceleration[0];
  current_acceleration[1] = (target_speeds[1] + speed_adjust[1] - current_speed[1]) * 7.0 + expected_acceleration[1];
  
  if(current_acceleration[0] > MAX_ACCELERATION) {
    current_acceleration[0] = MAX_ACCELERATION;
  }
  if(current_acceleration[0] < -MAX_ACCELERATION) {
    current_acceleration[0] = -MAX_ACCELERATION;
  }
  if(current_acceleration[1] > 3*MAX_ACCELERATION) {
    current_acceleration[1] = 3*MAX_ACCELERATION;
  }
  if(current_acceleration[1] < -3*MAX_ACCELERATION) {
    current_acceleration[1] = -3*MAX_ACCELERATION;
  }
  arm->moveByAcceleration( current_acceleration[0], current_acceleration[1] );

  // do nothing, chasing target
  // cout << "Current pos:   " << current_position[0] << ", " << current_position[1] << endl;
  // cout << "Time: " << t << endl;
  // cout <<"Target Speed: " << target_speeds[0] << ", " <<target_speeds[1] << endl;

  // cout <<"Current Speed: " << current_speed[0] << ", " << current_speed[1] << endl;

  // cout <<"Dist to go: " << distance_to_go[0] << ", " << distance_to_go[1] << endl;
  // cout <<"Acceleration: " << current_acceleration[0] << ", " << current_acceleration[1] << endl << endl;
  return 0;
}

void SimArmController::add_point_to_trajectory(double a1, double a2){
  if(!has_started) {
    has_started = true;
    start_time = micros();
  }
  double _pt[2];
  arm->to_xy(a1, a2, _pt[0], _pt[1]);
  double _lpt[2];
  arm->to_xy(keypoints[MAX_POINTS-1][0], keypoints[MAX_POINTS-1][1], _lpt[0], _lpt[1]);
  double angle_to_new_point = atan2(_pt[1] - _lpt[1], _pt[0] - _lpt[0]) * 180.0 / PI;
  double target_speed_to_new_point = -1;

  if (abs(angle_to_new_point - angles_at_keypoints[MAX_POINTS-2]) > 20) {
    // Stop
    target_speed_to_new_point = 0;
  }

  // Shift all points to the left
  for (int i = 0; i < MAX_POINTS-1; i++) {
    for (int j = 0; j < 2; j++) {
      keypoints[i][j] = keypoints[i + 1][j];
      targets[i][j] = targets[i + 1][j];
      time_at_keypoints[i] = time_at_keypoints[i + 1];
    }
  }

  for (int i = 0; i < MAX_POINTS-2; i++) {
    for (int j = 0; j < 2; j++) {
      target_directions[i][j] = target_directions[i + 1][j];
      max_speeds[i][j] = max_speeds[i + 1][j];
    }
    angles_at_keypoints[i] = angles_at_keypoints[i + 1];
    target_speeds_dir[i] = target_speeds_dir[i + 1];
    should_stop[i] = should_stop[i + 1];
  }

  // Add new point to the end
  keypoints[MAX_POINTS-1][0] = a1;
  keypoints[MAX_POINTS-1][1] = a2;

  targets[MAX_POINTS-1][0] = int(3 * a1 * arm->steps_per_radian);
  targets[MAX_POINTS-1][1] = int(9 * a2 * arm->steps_per_radian);
  target_speeds_dir[MAX_POINTS-2] = target_speed_to_new_point;

  if (keypoints[MAX_POINTS-1][0] - keypoints[MAX_POINTS-2][0] > 0) {
    target_directions[MAX_POINTS-2][0] = 1;
  } else if (keypoints[MAX_POINTS-1][0] - keypoints[MAX_POINTS-2][0] < 0) {
    target_directions[MAX_POINTS-2][0] = -1;
  }

  if (keypoints[MAX_POINTS-1][1] - keypoints[MAX_POINTS-2][1] > 0) {
    target_directions[MAX_POINTS-2][1] = 1;
  } else if (keypoints[MAX_POINTS-1][1] - keypoints[MAX_POINTS-2][1] < 0) {
    target_directions[MAX_POINTS-2][1] = -1;
  }

  current_target_indexes[0]--;
  current_target_indexes[1]--;

  angles_at_keypoints[MAX_POINTS-2] = angle_to_new_point;

  if (target_directions[MAX_POINTS-2][0] != target_directions[MAX_POINTS-3][0] || target_directions[MAX_POINTS-2][1] != target_directions[MAX_POINTS-3][1]) {
    should_stop[MAX_POINTS-2] = true;
    // change speed of previous point by half
    max_speeds[MAX_POINTS-3][0] = max_speeds[MAX_POINTS-3][0] * 0.5;
    max_speeds[MAX_POINTS-3][1] = max_speeds[MAX_POINTS-3][1] * 0.5;
    time_at_keypoints[MAX_POINTS-2] =  time_at_keypoints[MAX_POINTS-3] + 2 * (time_at_keypoints[MAX_POINTS-2] - time_at_keypoints[MAX_POINTS-3]);
  } else {
    should_stop[MAX_POINTS-2] = target_speed_to_new_point != 0;
  }

  double displacement_to_target[2] = {
    targets[MAX_POINTS-1][0] - targets[MAX_POINTS-2][0],
    targets[MAX_POINTS-1][1] - targets[MAX_POINTS-2][1]
  };

  int bigger_distance_index = 0;
  if (abs(displacement_to_target[0]) < abs(displacement_to_target[1])) {
    bigger_distance_index = 1;
  }
  double a = MAX_ACCELERATION * 0.5;
  double u = max_speeds[MAX_POINTS-3][bigger_distance_index];
  // double u

  max_speeds[MAX_POINTS-2][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  max_speeds[MAX_POINTS-2][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(max_speeds[MAX_POINTS-2][1]) > 3 * MAX_SPEED) {
    max_speeds[MAX_POINTS-2][0] = 3 * MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    max_speeds[MAX_POINTS-2][1] = 3 * MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }

  if(should_stop[MAX_POINTS-2]) {
    cout << "halving speed" << endl;
    max_speeds[MAX_POINTS-2][0] = max_speeds[MAX_POINTS-2][0] * 0.5;
    max_speeds[MAX_POINTS-2][1] = max_speeds[MAX_POINTS-2][1] * 0.5;
  }


  double v = max_speeds[MAX_POINTS-2][bigger_distance_index];
  double time_to_accelerate = (v - u) / a;

  double s = u * time_to_accelerate + 0.5 * a * time_to_accelerate * time_to_accelerate;

  double S_max = displacement_to_target[bigger_distance_index];

  // if

  double delta_t = abs( (S_max) / v);

  time_at_keypoints[MAX_POINTS-1] = time_at_keypoints[MAX_POINTS-2] + delta_t + time_to_accelerate * 0.25;

  // max_speeds[MAX_POINTS-2][0] += 3 * target_directions[MAX_POINTS-2][0];
  // max_speeds[MAX_POINTS-2][1] += 3 * target_directions[MAX_POINTS-2][1];

  double current_position[2] = {0, 0};
  arm->getJointPositionInSteps( current_position );
  // print stuff for debugging

  // Serial.print("Targets: ");
  // for(int i=0; i<5; i++) {
  //   Serial.print("| " + String(targets[i][0]) + ", " + String(targets[i][1]));
  // }
  // Serial.println(" ");
  
  // Serial.print("Target Speed: ");
  // for(int i=0; i<4; i++) {
  //   Serial.print("| " + String(max_speeds[i][0]) + ", " + String(max_speeds[i][1]));
  // }
  // Serial.println(" ");
}

