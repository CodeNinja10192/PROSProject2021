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

void liftUp() {
  lift.move_velocity(100);
}
void liftDown() {
  lift.move_velocity(-100);
}
void liftStop() {
  lift.move_velocity(0);
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

void liftOpControl() {
  if (master.get_digital(DIGITAL_R1)) {
    liftUp();
  }
  else if (master.get_digital(DIGITAL_R2)) {
    liftDown();
  }
  else {
    liftStop();
  }


}





// Tasking and Init
void liftTask(void* parameter) {
  if (pros::competition::is_connected()) {
    while (true) {
      liftOpControl();
      pros::delay(20);
    }
  }
  else if (pros::competition::is_autonomous()) {
    int previousError = 0;
    while (true) {
      int sp = liftTarget;

      double kP = 0.3;
      double kI = 0.3;
      double kD = 0.3;

      int sv = getLiftPosition();

      int error = sp - sv;
      int integral = integral + error;
      int derivative = error - previousError;

      if (error == 0) {
        integral = 0;
      }
      int power = (error * kP) + (integral * kI) + (derivative * kD);
      previousError = error;
      liftSlew(power);
      pros::delay(20);
    }
  }
}
