#include "main.h"

// Constants and Regular Values for Lift Value
const int lift_acceleration = 850;
const int lift_deceleration = 800;

double liftSpeed = 0;
double liftTarget = 0;






// Basic Functions

double getLiftPosition() {
  return lift.get_position();
}

void liftPower(int voltage) {
  lift.move_voltage(voltage);
}
void liftSetTarget(int target) {
  liftSpeed = 0;
  liftTarget = target;
}

// Autonomous Functions

void liftSlew(double liftTargetSpeed) {
  int step;

  if (abs(liftSpeed) < abs(liftTargetSpeed)) {
    step = lift_acceleration;
  }
  else {
    step = lift_deceleration;
  }

  if (liftTargetSpeed > liftSpeed + step) {
    liftSpeed += step;
  }
  else if (liftTargetSpeed < liftSpeed - step) {
    liftSpeed -= step;
  }
  else {
    liftSpeed = liftTargetSpeed;
  }

  liftPower(liftSpeed);
}

//Op Control Functions






// Tasking and Init
