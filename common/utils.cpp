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

int sign(double value) {
  if(value > 0) {
    return 1;
  }
  if(value < 0) {
    return -1;
  }
  return 0;
}
