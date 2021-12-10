#include "main.h"

// TANK CONTROL
void moveDrive(int left, int right);
void setDriveMotorsTank();
void setDriveMotorsArcade();
void reset();
void setBrakeMode(int modeNumber);
void driveTask(void* parameter);
void drivePIDTask(void* parameter);
void driveInit();
void driveAutonomousInit();
