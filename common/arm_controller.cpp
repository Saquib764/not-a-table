#include "arm_controller.h"


// double T = 1.2;
// int target_index = 4;
// double last_target_time = 0.0;

// double trace_error_sum = 0.0;
// double max_trace_error = 0.0;
double D = 50;
ArmController::ArmController(ArmModel *arm){
  this->arm = arm;

  // Define constants
  MAX_SPEED = 50 * MICROSTEPS;
  MAX_ACCELERATION = 50 * MICROSTEPS;
  
  has_started = false;
  has_finished = false;
  has_all_targets = false;
  

  // Define target arrays
  reset();

  target_speeds[0] = 0.0;
  target_speeds[1] = 0.0;

  error[0] = 0.0;
  error[1] = 0.0;
}

double ArmController::mod(double x, double y){
  double r = x - y * floor(x/y);
  if(r < 0) {
    r += y;
    r = mod(r, y);
  }
  return r;
}

void ArmController::force_stop(){
  arm->stepper1->forceStop();
  arm->stepper2->forceStop();
}

void ArmController::reset(){
  has_started = false;
  has_finished = false;
  has_all_targets = false;

  SPEED_LIMIT_RATIO = 1;
  
  target_index = 6;
  number_of_targets = 0;
  // Reinitialise trajectory to zero
  for(int i = 0; i < MAX_POINTS; i++) {
    targets[i][0] = 0;
    targets[i][1] = 0;
    keypoints[i][0] = 0;
    keypoints[i][1] = 0;
    time_at_keypoints[i] = 0;

    position_targets[i][0] = 0;
    position_targets[i][1] = 0;
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

void ArmController::get_goal(double *current_position, double *goal) {
  goal[0] = targets[target_index][0] - current_position[0];
  goal[1] = targets[target_index][1] - current_position[1];

  // double d = D * 0.8;

  // if(should_stop[current_target_index + 1]) {
  //   d = 20;
  // }
  
  // if(abs(goal[0]) < d && abs(goal[1]) < d) {
  //   // find goal in the next segment
  //   current_target_index = current_target_index + 1;
  //   goal[0] = targets[current_target_index][0] - current_position[0];
  //   goal[1] = targets[current_target_index][1] - current_position[1];
  // }
}

int ArmController::get_target_index(double *position){
  int cur_target = target_index;
  double distance = abs(targets[cur_target][0] - position[0]) + abs(targets[cur_target][1] - position[1]);
  if( distance < 50) {
    target_index = target_index + 1;
  }
  return target_index;
  // int cur_pos = 9;
  // int closest_distance = 100000000;
  // for(int i = MAX_POINTS - 3; i > 0; i--) {
  //   double distance = abs(targets[i][0] - position[0]) + abs(targets[i][1] - position[1]);
  //   if( distance < closest_distance) {
  //     cur_pos = i;
  //     closest_distance = distance;
  //   }
  // }
  // if(cur_pos == 0) {
  //   cur_pos = 8;
  // }
  // int d11 = sign(targets[cur_pos][0] - position[0]);
  // int d12 = sign(targets[cur_pos][1] - position[1]);
  // int d21 = sign(targets[cur_pos+1][0] - position[0]);
  // int d22 = sign(targets[cur_pos+1][1] - position[1]);

  // if(d11 * d21 <= 0 || d12 * d22 <= 0) {
  //   cur_pos = cur_pos + 1;
  // }

  // return cur_pos - 1;
}

void ArmController::get_target_position(double t, double *target_position){
  // Get time index
  int time_index = get_target_index(target_position);
  if(time_index == MAX_POINTS - 1) {
    target_position[0] = targets[time_index][0] ;
    target_position[1] = targets[time_index][1] ;
    return;
  }
}

void ArmController::get_target_speed(double *current_position, double *speeds){
  // Get time index
  double goal[2] = {0, 0};
  get_goal(current_position, goal);

  SPEED_LIMIT_RATIO = 1.0;

  double UPDATED_MAX_SPEED = MAX_SPEED * SPEED_LIMIT_RATIO;
  speeds[0] = UPDATED_MAX_SPEED * goal[0] / (abs(goal[0]) + 0.001);
  speeds[1] = UPDATED_MAX_SPEED * goal[1] / (abs(goal[0]) + 0.001);
  if(abs(speeds[1]) > UPDATED_MAX_SPEED) {
    speeds[0] = UPDATED_MAX_SPEED * goal[0] / (abs(goal[1]) + 0.001);
    speeds[1] = UPDATED_MAX_SPEED * goal[1] / (abs(goal[1]) + 0.001);
  }
}

// void ArmController::get_target_acceleration(double *positions, double *speeds, double *accelerations) {
//   double dt = 0.1;
//   double next_positions[2] = {
//     positions[0] + speeds[0] * dt,
//     positions[1] + speeds[1] * dt
//   };
//   double next_speeds[2];
//   get_target_speed(next_positions, next_speeds);
//   accelerations[0] = (next_speeds[0] - speeds[0]) / dt;
//   accelerations[1] = (next_speeds[1] - speeds[1]) / dt;

//   enforce_guards(accelerations, MAX_ACCELERATION);
//   // cout << "acc: " << accelerations[0] << ", " << accelerations[1] << endl;
// }

int ArmController::follow_trajectory() {
  if(number_of_targets < 3) {
    return 1;
  }
  double current_position[2] = {0, 0};
  arm->getJointPositionInSteps( current_position );
  double t = (micros() - start_time) / 1000000.0;
  int current_target_index = get_target_index(current_position);

  if (current_target_index >= MAX_POINTS-2 && has_all_targets) {
    Serial.println("Finished");
    has_finished = true;
    target_speeds[0] = 0.0;
    target_speeds[1] = 0.0;
    arm->stopMove();
    return 2;
  }

  // Serial.println("Current Target Index: " + String(current_target_index));
  // Serial.println("Current targets: " + String(targets[current_target_index][0]) + ", " + String(targets[current_target_index][1]));

  double displacement_to_go[2] = {
    targets[current_target_index][0] - current_position[0],
    targets[current_target_index][1] - current_position[1]
  };
  
  EVERY_N_MILLISECONDS(1000) {
    Serial.println("Disp: " + String(displacement_to_go[0]) + ", " + String(displacement_to_go[1]));
    Serial.println("Target: " + String(targets[current_target_index][0]) + ", " + String(targets[current_target_index][1]));
    Serial.println("Pos: " + String(current_position[0]) + ", " + String(current_position[1]));
    Serial.println("Ind: " + String(current_target_index) + " of " + String(number_of_targets));
    Serial.println("-------------------------------");
  }

  if(current_target_index > 2 && !has_all_targets) {
    return 1;
  }
  // t = t - last_target_time;
  double current_acceleration[2] = {0, 0};
  double current_speed[2] = {0, 0};
  arm->getJointSpeedInSteps( current_speed );

  get_target_speed(current_position, target_speeds);

  current_acceleration[0] = (target_speeds[0] - current_speed[0]);
  current_acceleration[1] = (target_speeds[1] - current_speed[1]);

  enforce_guards(current_acceleration, MAX_ACCELERATION);

  arm->moveByAcceleration( current_acceleration[0], current_acceleration[1] );

  // do nothing, chasing target
  // Serial.println("Target Speed: "+ String(target_speeds[0]) + ", " + String(target_speeds[1]));

  // Serial.println("Current Speed: "  + String(current_speed[0]) + ", " + String(current_speed[1]));

  // Serial.println("Acceleration: "  + String(current_acceleration[0]) + ", " + String(current_acceleration[1]));
  return 0;
}

void ArmController::add_point_to_trajectory(double a1, double a2){
  // Serial.println("Adding point to trajectory: " + String(a1) + ", " + String(a2));
  if(!has_started) {
    has_started = true;
    start_time = micros();
  }
  double dt1 = abs(targets[MAX_POINTS-1][0]-int(3 * a1 * arm->steps_per_radian));
  double dt2 = abs(targets[MAX_POINTS-1][1]-int(3 * a2 * arm->steps_per_radian));
  if(dt1 < 50 && dt2 < 50) {
    return;
  }
  number_of_targets ++;
  target_index--;
  // double t = (micros() - start_time) / 1000000.0;
  // int current_target_index = get_current_target_index(t) + 1;
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
      
      position_targets[i][j] = position_targets[i + 1][j];
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
    time_to_target[i] = time_to_target[i + 1];
  }

  // Add new point to the end
  keypoints[MAX_POINTS-1][0] = a1;
  keypoints[MAX_POINTS-1][1] = a2;

  targets[MAX_POINTS-1][0] = int(3 * a1 * arm->steps_per_radian);
  targets[MAX_POINTS-1][1] = int(3 * a2 * arm->steps_per_radian);

  position_targets[MAX_POINTS-1][0] = int(3 * a1 * arm->steps_per_radian);
  position_targets[MAX_POINTS-1][1] = int(3 * a2 * arm->steps_per_radian);

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

  angles_at_keypoints[MAX_POINTS-2] = angle_to_new_point;

  should_stop[MAX_POINTS-2] = false;
  if (target_directions[MAX_POINTS-2][0] != target_directions[MAX_POINTS-3][0] || target_directions[MAX_POINTS-2][1] != target_directions[MAX_POINTS-3][1]) {
    should_stop[MAX_POINTS-2] = true;
  }

  double displacement_to_target[2] = {
    targets[MAX_POINTS-1][0] - targets[MAX_POINTS-2][0],
    targets[MAX_POINTS-1][1] - targets[MAX_POINTS-2][1]
  };

  int bigger_distance_index = 0;
  if (abs(displacement_to_target[0]) < abs(displacement_to_target[1])) {
    bigger_distance_index = 1;
  }
  double u = max_speeds[MAX_POINTS-3][bigger_distance_index];
  // double u

  max_speeds[MAX_POINTS-2][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  max_speeds[MAX_POINTS-2][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(max_speeds[MAX_POINTS-2][1]) > MAX_SPEED) {
    max_speeds[MAX_POINTS-2][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    max_speeds[MAX_POINTS-2][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }

  double current_position[2] = {0, 0};
  arm->getJointPositionInSteps( current_position );
}

