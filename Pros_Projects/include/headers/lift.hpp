#include "main.h"
//Basic Functions
double getLiftPosition();
void liftPower(int voltage);
void liftSlew(double liftTargetSpeed);
void liftSetTarget(int target);

//Specific Lift Functions (Use for Op Control)


// Tasking and Init
void liftPID(void* parameter);
void liftOpControl(void* parameter);
void liftPIDInit();
void liftOpControlInit();
