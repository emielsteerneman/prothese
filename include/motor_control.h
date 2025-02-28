#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void run_emiel_motor_test();
bool turn_steps_per_second(uint32_t steps_per_second, uint8_t dir);
void setup_motor_control();
void enable_motor();
void disable_motor();

#endif // MOTOR_CONTROL_H