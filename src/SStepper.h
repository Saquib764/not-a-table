/*
 * Define and control single stepper
 *
 * 
*/

// ensure this library description is only included once
#ifndef SStepper_h
#define SStepper_h

class SStepper {
  public:
    // constructors:
    SStepper(int DIR_PIN, int STEP_PIN, int HOMING_PIN);

    void one_step(int direction=1, int wait=20);

    int DIR_PIN;
    int STEP_PIN;
    int HOMING_PIN;
};
#endif