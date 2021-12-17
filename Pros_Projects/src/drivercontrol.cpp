#include "main.h"

double driveTarget = 0;
double leftTarget = 0;
double rightTarget = 0;
double turnTarget = 0;

double driveSpeed = 0;
int leftSpeed = 0;
int rightSpeed = 0;

int drive_acceleration = 4;
int drive_deceleration = 256;

bool isTurning = false;

double leftPreviousError = 0;
double rightPreviousError = 0;
double turnPreviousError = 0;

double maxSpeed = 127;

void moveDrive(int left, int right) {
  driveLeftBack.move(left);
  driveLeftFront.move(left);
  driveRightBack.move(right);
  driveRightFront.move(right);
}
double getLeftPos() {
  return driveLeftFront.get_position();
}
double getRightPos() {
  return driveRightFront.get_position();
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



// Slew Functions

void driveSlew(double leftTarget, double rightTarget) {
  int leftStep;
  int rightStep;

  // Left Slew
  if ((abs(leftSpeed)) < abs(leftTarget)) {
    leftStep = drive_acceleration;
  }
  else {
    leftStep = drive_deceleration;
  }

  if (leftTarget > leftSpeed + leftStep) {
    leftTarget += leftStep;
  }
  else if (leftTarget < leftSpeed - leftStep) {
    leftSpeed -= leftStep;
  }
  else {
    leftSpeed = leftTarget;
  }

  // Right Slew
  if ((abs(rightSpeed)) < abs(rightTarget)) {
    rightStep = drive_acceleration;
  }
  else {
    rightStep = drive_deceleration;
  }

  if (rightTarget > rightSpeed + rightStep) {
    rightTarget += rightStep;
  }
  else if (rightTarget < rightSpeed - rightStep) {
    rightSpeed -= rightStep;
  }
  else {
    rightSpeed = rightTarget;
  }
  moveDrive(leftSpeed, rightSpeed);
}



// Autonomous Functions

void leftMoveAsync(double target) {
  reset();
  driveTarget = target;

}
void leftMove(double target) {
  leftMoveAsync(target);
}

void rightMoveAsync(double target) {
  reset();
  rightTarget = target;
}
void rightMove(double target) {
  rightMoveAsync(target);
}
void moveForward(double distance) {
  leftMoveAsync(distance);
  rightMoveAsync(distance);
}
void moveBackward(double distance) {
  double backward_distance = -1 * distance;
  moveForward(backward_distance);
}
void turnAsync(double target) {
  reset();
  turnTarget = target;
  isTurning = true;
}
void turn(double target) {
  turnAsync(target);
}

// Tasking

void driveTask(void* parameter) {
  reset();
  // Turn PID Constants
  double tP = 0.3;
  double tI = 0.3;
  double tD = 0.3;
  // Drive PID Constants
  double dP = 0.3;
  double dI = 0.3;
  double dD = 0.3;
  while (true) {
    if (!pros::competition::is_autonomous()) {
      setDriveMotorsArcade();
    }
    else {
      if (isTurning) {
        double turnPos = (getLeftPos() - getRightPos()) / 2;
        double turnError = turnTarget - turnPos;
        double turnIntegral = turnIntegral + turnError;
        if (turnError == 0) {
          turnIntegral = 0;
        }
        double turnDerivative = turnError - turnPreviousError;
        turnPreviousError = turnError;
        double turnPower = turnError * tP + turnIntegral * tI + turnDerivative * tD;
        if (turnPower > maxSpeed) {
          turnPower = maxSpeed;
        }
        else if (turnPower < -maxSpeed) {
          turnPower = -maxSpeed;
        }
        driveSlew(turnPower, -turnPower);


      }
      else {
        // Left Calc.
        double leftPos = getLeftPos();
        double leftError = leftTarget - leftPos;
        double leftDerivative = leftError - leftPreviousError;
        double leftIntegral;
        leftPreviousError = leftError;
        if (abs(leftError) > 20) {
          leftIntegral += leftError;
        }
        else {
          leftIntegral = 0;
        }
        double leftPower = leftError * dP + leftIntegral * dI + leftDerivative * dD;
        if (leftPower > 127) {
          leftPower = 127;
        }
        else if (leftPower < -127) {
          leftPower = -127;
        }
        // Right Calc.

        double rightPos = getRightPos();
        double rightError = rightTarget - rightPos;
        double rightDerivative = rightError - rightPreviousError;
        double rightIntegral;
        rightPreviousError = rightError;
        if (abs(rightError) > 20) {
          rightIntegral += rightError;
        }
        else {
          rightIntegral = 0;
        }
        double rightPower = rightError * dP + rightIntegral * dI + rightDerivative * dD;
        if (rightPower > 127) {
          rightPower = 127;
        }
        else if (leftPower < -127) {
          rightPower = -127;
        }


        driveSlew(leftPower, rightPower);

      }

    }
    pros::delay(20);

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
