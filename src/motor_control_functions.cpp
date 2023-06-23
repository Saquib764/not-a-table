#include "motor_control_functions.h"

#define MAX_SPEED                 0.04  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                16
#define STEPS_PER_REV             200
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s


// Total steps per revolution = 200 * 16 = 3200

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup_driver(TMC2209Stepper &driver, int EN_PIN, int MS1, int MS2) {
  Serial.println("Setting up driver");
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // NEVER increase the baud rate, its adds 10ms delay. probably related to buffer overflow and flush cycle
  SERIAL_PORT.begin(9600);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(3);                 // Enables driver in software
  // driver.pdn_disable(true);
  driver.rms_current(500);        // Set motor RMS current
  driver.microsteps( MICROSTEPS );          // Set microsteps to 1/16th
  // driver.irun(31);

  driver.intpol(true);               // Interpolate to 256 steps, smooth stepping even with 0 microsteps.

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}

double K = STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
float R = 0.63/2;

void setup_arm(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t HOMING_1, uint8_t DIR_2, uint8_t STEPPER_2, uint8_t HOMING_2) {
  engine.init();
  stepper1 = engine.stepperConnectToPin(STEPPER_1);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_1);
    stepper1->setEnablePin(EN_PIN);
    stepper1->setAutoEnable(true);

    // stepper1->keepRunning();
  }
  stepper2 = engine.stepperConnectToPin(STEPPER_2);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2);
    stepper2->setEnablePin(EN_PIN);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(0);
    stepper2->setAcceleration(0);

    // stepper2->keepRunning();
  }
}

long previous_targets[2] = {0, 0};
long current_targets[2] = {0, 0};
long next_targets[2] = {0, 0};
double next_speeds[2] = {0.0, 0.0};
double current_speed_targets[2] = {0.0, 0.0};
double current_accelerations[2] = {0.0, 0.0};
int caution_distances[2] = { 0,0};

double error_integral = 0.0;

bool has_new_target = false;

double* compute_speeds_to_next_target(double* speeds, long * current_positions, long * next_positions, double* max_speeds) {
  // Serial.println("MAx: " + String(max_speeds[0]) + ", " + String(max_speeds[1]));
  long distances[2] = {next_positions[0] - current_positions[0], next_positions[1] - current_positions[1]};
  
  if(distances[0] == 0) {
    speeds[0] = 0.0;
    speeds[1] = max_speeds[1];
  }else if(distances[1] == 0) {
    speeds[0] = max_speeds[0];
    speeds[1] = 0.0;
  }else {
    speeds[0] = max_speeds[0];
    speeds[1] = abs(max_speeds[0] * distances[1] / distances[0]);
    if(speeds[1] > max_speeds[1]) {
      speeds[0] = abs(max_speeds[1] * distances[0] / distances[1]);
      speeds[1] = max_speeds[1];
    }
  }

  return speeds;
}

bool move_arm(long int * delta, double theta1, double theta2) {
  double max_speeds[2] = {
    min( 18.0 * MAX_SPEED * K / (R * (5*abs(cos(theta2/2)) + 1)), MAX_ANGULAR_SPEED),
    min( 18.0 * MAX_SPEED * K / R, MAX_ANGULAR_SPEED)
  };

  long int target1 = 3 * theta1 * K;
  long int target2 = 3 * 3 * (theta2 + theta1/3.0) * K;

  double speed_1;
  double speed_2;

  if(target1 != next_targets[0] || target2 != next_targets[1]) {
    // Serial.print("Target 1: " + String(target1) + ", " + String(theta1) + ". Current pos: ");
    Serial.println(stepper1->getCurrentPosition() / (K * 3));
    next_targets[0] = target1;

    
    // Serial.print("Target 2: " + String(target2) + ", " + String(theta2) + ". Current pos: ");
    Serial.println(stepper2->getCurrentPosition() / (9*K) - stepper1->getCurrentPosition() / (9*K));
    next_targets[1] = target2;

    compute_speeds_to_next_target(next_speeds, current_targets, next_targets, max_speeds);
    has_new_target = true;
  }
  delta[0] = current_targets[0] - stepper1->getCurrentPosition();
  delta[1] = current_targets[1] - stepper2->getCurrentPosition();

  double current_speeds[2] = {
    stepper1->getCurrentSpeedInMilliHz() / 1000.0,
    stepper2->getCurrentSpeedInMilliHz() / 1000.0
  };

  if(delta[0] == 0 && delta[1] == 0 && has_new_target) {
    long initial_displacement[2] = {
      next_targets[0] - current_targets[0],
      next_targets[1] - current_targets[1]
    };

    Serial.println("Speed computed: " + String(next_speeds[0]) + ", " + String(next_speeds[1]));
    Serial.println("Direction: " + String(initial_displacement[0]) + ", " + String(initial_displacement[1]));

    current_accelerations[0] = next_speeds[0] / ACCELERATION_TIME;
    current_accelerations[1] = next_speeds[1] / ACCELERATION_TIME;

    stepper1->setSpeedInHz(next_speeds[0]);
    stepper2->setSpeedInHz(next_speeds[1]);
    stepper1->setAcceleration( current_accelerations[0] );
    stepper2->setAcceleration( current_accelerations[1] );
    stepper1->moveTo(next_targets[0]);
    stepper2->moveTo(next_targets[1]);

    previous_targets[0] = current_targets[0];
    previous_targets[1] = current_targets[1];
    current_targets[0] = next_targets[0];
    current_targets[1] = next_targets[1];
    next_speeds[0] = 0.0;
    next_speeds[1] = 0.0;
    delta[0] = current_targets[0] - stepper1->getCurrentPosition();
    delta[1] = current_targets[1] - stepper2->getCurrentPosition();
    has_new_target = false;
  }

  long initial_displacement[2] = {
    current_targets[0] - previous_targets[0],
    current_targets[1] - previous_targets[1]
  };

  // Implement smooth follow-on keypoints
  if( delta[0] <= current_speeds[0] * ACCELERATION_TIME && delta[1] <= current_speeds[1] * ACCELERATION_TIME ) {
    long next_displacement[2] = {
      next_targets[0] - current_targets[0],
      next_targets[1] - current_targets[1]
    };
    if(next_displacement[0] * initial_displacement[0] > 0) {
      stepper1->setAcceleration( 0.0 );
    }else {
      stepper1->setAcceleration( current_speeds[0] / ACCELERATION_TIME );
    }

    if(next_displacement[1] * initial_displacement[1] > 0) {
      stepper2->setAcceleration( 0.0 );
    }else {
      stepper2->setAcceleration( current_speeds[2] / ACCELERATION_TIME );
    }

    stepper1->applySpeedAcceleration();
    stepper2->applySpeedAcceleration();
  }

  // use larger distance as reference
  double error[2] = { 0.0, 0.0 };
  int motor_to_slow = -1;
  
  int s = 0;
  int b = 1;
  if( initial_displacement[0] != 0 && initial_displacement[1] != 0 ) {
    error[0] = 0.0;
    if(abs(initial_displacement[0]) > abs(initial_displacement[1])) {
      b = 0;
      s = 1;
    }
    error[1] = abs(delta[b]) - abs(delta[s] * initial_displacement[b]) / abs(initial_displacement[s]);
    // error[1] > 0 => motor 2 is too slow=> slow down motor 1
    // error[1] < 0 => motor 2 is too fast=> slow down motor 2
    // we always slow down the motor
    error_integral += error[1];
    error_integral = min(100.0, max(-100.0, error_integral));
  }else {
    error_integral = 0.0;
  }
  
  double f = 100.0;
  // stepper1->setSpeedInHz( current_speed_targets[0] * (1 - ( f * error[1] + error_integral)/abs(current_speed_targets[0])) );

  // if(abs(error[0]) > 1) {
  //   stepper1->setAcceleration( - f * 10* error[1] * current_speed_targets[0]/abs(current_speed_targets[0]) );
  // }else{
  //   stepper1->setAcceleration( current_accelerations[0] );
  // }

  // stepper2->setSpeedInHz( current_speed_targets[1] * (1 + (f * error[1] + error_integral) / abs(current_speed_targets[1])) );
  
  // if(abs(error[0]) > 1) {
  //   stepper2->setAcceleration( f* 10 * error[1] * current_speed_targets[1]/abs(current_speed_targets[1]) );
  // }else{
  //   stepper2->setAcceleration( current_accelerations[1] );
  // }

  // stepper1->applySpeedAcceleration();
  // stepper2->applySpeedAcceleration();

  EVERY_N_MILLISECONDS(2000) {
    Serial.println("Error: "+ String(error[0]) + ", " + String(error[1]));
    Serial.println("Distance from target: "+ String(delta[0]) + ", " + String(delta[1]));
    Serial.println("Speeds: " + String(stepper1->getCurrentSpeedInMilliHz()/1000.0) + ", " + String(stepper2->getCurrentSpeedInMilliHz()/1000.0) );
  }
  return !has_new_target;
}

void force_stop() {
  stepper1->forceStop();
  stepper2->forceStop();
}