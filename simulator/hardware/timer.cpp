#include "timer.h"


double micros() {
  return current_time * 1000000.0;
}

void update_time(double dt) {
  current_time += dt;
}
