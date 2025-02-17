#ifndef MOTOR_STUFF_H
#define MOTOR_STUFF_H

#include <stdint.h>

void run_emiel_motor_test();
bool turn_steps_per_second(uint32_t steps_per_second);
void setup_motor_control();
void enable_motor();
void disable_motor();

#endif // MOTOR_STUFF_H