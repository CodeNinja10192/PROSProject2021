#include "main.h"


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
