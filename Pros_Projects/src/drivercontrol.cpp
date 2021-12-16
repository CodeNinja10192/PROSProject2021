#include "main.h"
int driveTarget = 0;


void moveDrive(int left, int right) {
  driveLeftBack.move(left);
  driveLeftFront.move(left);
  driveRightBack.move(right);
  driveRightFront.move(right);
}






void setDriveMotorsTank() {
  int leftJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  if (abs(leftJoystick) < 10) {
    leftJoystick = 0;
  }
  if (abs(rightJoystick) < 10) {
    rightJoystick = 0;
  }


  moveDrive(leftJoystick, rightJoystick);
}

// ARCADE CONTROL

void setDriveMotorsArcade() {
  int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  if (abs(power) < 10) {
    power = 0;
  }
  if (abs(turn) < 10) {
    turn = 0;
  }
  int left = power + turn;
  int right = power - turn;
  moveDrive(left, right);

}
void reset() {
  driveLeftBack.tare_position();
  driveRightBack.tare_position();
  driveLeftFront.tare_position();
  driveRightFront.tare_position();
  moveDrive(0, 0);
}



// Autonomous Functions




// Tasking

/*
void driveTask(void* parameter) {
  while (true) {
    setDriveMotorsArcade();
    pros::delay(20);
  }
}

void drivePIDTask(void* parameter) {

  int previousError = 0;
  while (true) {
    int sp = driveTarget;

    double kP = 0.3;
    double kI = 0.3;
    double kD = 0.3;

    int sv = driveLeftBack.get_position();

    int error = sp - sv;
    int integral = integral + error;
    int derivative = error - previousError;

    if (error == 0) {
      integral = 0;
    }
    int power = (error * kP) + (integral * kI) + (derivative * kD);
    previousError = error;
    moveDrive(power, power);
    pros::delay(20);
  }
}

*/
void driveTask(void* parameter) {
  if (pros::competition::is_autonomous()) {
    int previousError = 0;
    while (true) {
      int sp = driveTarget;

      double kP = 0.3;
      double kI = 0.3;
      double kD = 0.3;

      int sv = driveLeftBack.get_position();

      int error = sp - sv;
      int integral = integral + error;
      int derivative = error - previousError;

      if (error == 0) {
        integral = 0;
      }
      int power = (error * kP) + (integral * kI) + (derivative * kD);
      previousError = error;
      moveDrive(power, power);
      pros::delay(20);
    }
  }
  else if (pros::competition::is_connected()) {
    while (true) {
      setDriveMotorsArcade();
      pros::delay(20);
    }
  }
}


// Drive Modifiers

void setBrakeMode(int modeNumber) {
  //0 = COAST
  //1 = BRAKE
  //2 = HOLD
  pros::motor_brake_mode_e_t brakeMode;

  switch (modeNumber) {
    case 0:
      brakeMode = MOTOR_BRAKE_COAST;
      break;
    case 1:
      brakeMode = MOTOR_BRAKE_BRAKE;
      break;
    case 2:
      brakeMode = MOTOR_BRAKE_HOLD;
      break;
  }
  driveLeftBack.set_brake_mode(brakeMode);
  driveLeftFront.set_brake_mode(brakeMode);
  driveRightBack.set_brake_mode(brakeMode);
  driveRightFront.set_brake_mode(brakeMode);
}
