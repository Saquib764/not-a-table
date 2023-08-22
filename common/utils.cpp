#include "../include/utils.h"

void enforce_guards(double *value, double limit) {
  value[0] = enforce_guards(value[0], limit);
  value[1] = enforce_guards(value[1], limit);
}

double enforce_guards(double value, double limit) {
  if(value > limit) {
    value = limit;
  }
  if(value < -limit) {
    value = -limit;
  }
  return value;
}