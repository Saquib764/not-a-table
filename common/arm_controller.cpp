#include "arm_controller.h"


ArmController::ArmController(ArmModel *arm){
  this->arm = arm;

  // Define constants
  MAX_SPEED = 200;
  MAX_ACCELERATION = 3 * MAX_SPEED;
  
  // Define global variables
  current_target_indexes[0] = 2;
  current_target_indexes[1] = 2;

  // Define target arrays
  reset();

  target_speeds[0] = 0.0;
  target_speeds[1] = 0.0;

  error = 0;
  last_error = 0;
  speed_integral[0] = 0.0;
  speed_integral[1] = 0.0;
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
  // Reinitialise trajectory to zero
  for(int i = 0; i < 5; i++) {
    targets[i][0] = 0;
    targets[i][1] = 0;
    keypoints[i][0] = 0;
    keypoints[i][1] = 0;
  }
  for(int i = 0; i < 4; i++) {
    target_speeds_dir[i] = 0;
    target_directions[i][0] = 0;
    target_directions[i][1] = 0;
    angles_at_keypoints[i] = 0;
    max_speeds[i][0] = 0;
    max_speeds[i][1] = 0;
    should_stop[i] = false;
  }
}

int ArmController::follow_trajectory() {
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
  arm->getJointPositionInSteps( current_position );

  double current_speed[2] = {0, 0};
  arm->getJointSpeedInSteps( current_speed );


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

  // setSpeedInHz( target_speeds[0], target_speeds[1] );
  // setSpeedInHz( abs(_max_speeds[0]), abs(_max_speeds[1]) );
  arm->moveByAcceleration( current_acceleration[0], current_acceleration[1] );

  // do nothing, chasing target
  Serial.println("Target Speed: "+ String(target_speeds[0]) + ", " + String(target_speeds[1]));

  Serial.println("Current Speed: "  + String(current_speed[0]) + ", " + String(current_speed[1]));

  double s1 = arm->stepper1->getCurrentSpeedInMilliHz() / 1000.0;
  double s2 = arm->stepper2->getCurrentSpeedInMilliHz() / 1000.0;
  Serial.println("Current Speed2: "  + String(s1) + ", " + String(s2));
  return 0;
}

void ArmController::add_point_to_trajectory(double a1, double a2){
  double _pt[2];
  arm->to_xy(a1, a2, _pt[0], _pt[1]);
  double _lpt[2];
  arm->to_xy(keypoints[4][0], keypoints[4][1], _lpt[0], _lpt[1]);
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
  arm->getJointPositionInSteps( current_position );
  // print stuff for debugging


  Serial.print("Targets: ");
  for(int i=0; i<5; i++) {
    Serial.print("| " + String(targets[i][0]) + ", " + String(targets[i][1]));
  }
  Serial.println(" ");
  
  Serial.print("Target Speed: ");
  for(int i=0; i<4; i++) {
    Serial.print("| " + String(max_speeds[i][0]) + ", " + String(max_speeds[i][1]));
  }
  Serial.println(" ");
}