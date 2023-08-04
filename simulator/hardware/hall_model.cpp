#include "hall_model.h"

HallModel::HallModel(double field_strength, double noise_amplitude) {
  // initialize this instance's variables
  this->position = 0.0;
  this->field_strength = field_strength;
  this->noise_amplitude = noise_amplitude;
}

void HallModel::setPosition(double position) {
  this->position = position;
}

double HallModel::readValue(double motor_position) {
  srand(2);
  double pi = 3.141592653;
  // handle polar coordinates
  motor_position = motor_position + pi;
  motor_position = motor_position - floor(motor_position / (2 * pi)) * (2*pi) - pi;
  double distance = motor_position - this->position;
  double noise = 2.0 * ((double)rand() / RAND_MAX - 1) * this->noise_amplitude;
  
  double r = distance / this->field_strength;
  double value = 2000.0 + 1000/ (1000.0 * r * r + 1.0);
  value = round(value  + noise);
  return value;
}

