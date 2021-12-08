#include "main.h"



// Intake Control (OLD)

/*
void intakeHelper(int value);
void setIntakeMotors();
void conveyorHelper(int value);
void setConveyorMotors();
*/

// Intake Control (NEW)

void intakeIn();
void outTake();
void intakeStop();
void intakeRelativeMove(int degrees, int vel);
void intakePower(int voltage);
void intakeOpControl();
void intakeTask(void* parameter);
void intakeInit();
