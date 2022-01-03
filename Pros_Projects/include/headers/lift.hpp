#include "main.h"
//Basic Functions
double getLiftPosition();
void liftPower(int voltage);
void liftSlew(double liftTargetSpeed);
void liftSetTarget(int target);

//Specific Lift Functions (Use for Op Control)
void liftUp();
void liftDown();
void liftStop();
void liftOpControl();

// Tasking and Init
void liftTask(void* parameter);
