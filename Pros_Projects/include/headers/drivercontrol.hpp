#include "main.h"

double getLeftPos();
double getRightPos();
void moveDrive(int left, int right);
void setDriveMotorsTank();
void setDriveMotorsArcade();
void reset();
void setBrakeMode(int modeNumber);
void driveTask(void* parameter);
void leftSlew(double leftTarget);
void rightSlew(double rightTarget);
void leftMoveAsync(double target);
void leftMove(double target);
void rightMoveAsync(double target);
void rightMove(double target);
void moveForward(double distance);
void moveBackward(double distance);
void turnAsync(double target);
void turn(double target);
