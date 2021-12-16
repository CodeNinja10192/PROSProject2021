#include "main.h"
int intakeTarget = 0;
//Intake Control (OLD)

/*
void intakeHelper(int value) {
  intake.move(value);
}
void conveyorHelper(int value) {
  conveyor.move(value);
}
void setIntakeMotors() {
  intakeHelper(127);
}
void setConveyorMotors() {
  conveyorHelper(-127);

}
*/

// Intake Control (NEW)

void intakeIn() {
  intake.move_voltage(12000);
}
void outTake() {
  intake.move_voltage(-12000);
}
void intakeStop() {
  intake.move_voltage(0);
}




void intakeRelativeMove(int counts, int vel) {
  intake.move_relative(counts, vel);
}
void intakePower(int voltage) {
  intake.move_voltage(voltage);
}
void intakeOpControl() {
  if (master.get_digital(DIGITAL_L1)) {
    intakeIn();
  }
  else if (master.get_digital(DIGITAL_L2)) {
    outTake();
  }
  else {
    intakeStop();
  }
}

void intakeTask(void* parameter) {
  if (pros::competition::is_connected()) {
    while (true) {
      intakeOpControl();
      pros::delay(20);
    }
  }
  else if (pros::competition::is_autonomous()) {
    int previousError = 0;
    while (true) {
      int sp = intakeTarget;

      double kP = 0.3;
      double kI = 0.3;
      double kD = 0.3;

      int sv = intake.get_position();

      int error = sp - sv;
      int integral = integral + error;
      int derivative = error - previousError;

      if (error == 0) {
        integral = 0;
      }
      int power = (error * kP) + (integral * kI) + (derivative * kD);
      previousError = error;
      intakePower(power);
      pros::delay(20);
    }
  }
}
