#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// void run_emiel_motor_test();
bool turnStepsPerSecond(uint32_t steps_per_second, uint8_t dir);
void setupMotorControl();
void enableMotor();
void disableMotor();

#endif // MOTOR_CONTROL_H