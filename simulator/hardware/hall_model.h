#ifndef Hall_Model_h
#define Hall_Model_h

#include <cmath>

class HallModel{
  private:
    /* data */
  public:
    // constructors:
    HallModel(double field_strength, double noise_amplitude);

    void setPosition(double position);
    double readValue(double motor_position);

    double position;
    double field_strength;
    double noise_amplitude;
};

#endif