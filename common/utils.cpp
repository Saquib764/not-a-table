#include "../include/utils.h"

void enforce_guards(double *value, double limit) {
  if(value[0] > limit) {
    value[0] = limit;
  }
  if(value[0] < -limit) {
    value[0] = -limit;
  }
  if(value[1] > limit) {
    value[1] = limit;
  }
  if(value[1] < -limit) {
    value[1] = -limit;
  }
}