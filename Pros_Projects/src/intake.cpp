#include "main.h"

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
  while (true) {
    intakeOpControl();
    pros::delay(20);
  }
}

void intakeInit() {
  pros::Task conveyorTask(intakeTask);
}
