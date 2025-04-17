#ifndef ALGORITHMS_H
#define ALGORITHMS_H

struct algorithm1  // Struct to hold algorithm data
{
    unsigned long omega_x_trigger_timestamp = 0; // Stores the last time gx was triggered
    const unsigned long OMEGA_X_COOLDOWN_PERIOD = 500; // Cooldown period in milliseconds
    unsigned long roll_trigger_timestamp = 0; // Stores the last time roll was triggered
    const unsigned long ROLL_COOLDOWN_PERIOD = 1000; // Cooldown period in milliseconds
    bool omega_x_triggered = false; // Stores the last time gx was triggered
    int motor_direction = 0; // 0 = flex, 1 = extend
};

struct algorithm2  // Struct to hold algorithm data
{
    const float OMEGA_X_EXTEND_THRESHOLD = -7;
    const float OMEGA_X_FLEX_THRESHOLD = 7;
    bool extend_motor_running = false;
    bool flex_motor_running = false;
    bool extend_cooldown_passed = false;
    bool flex_cooldown_passed = false;
    const unsigned long EXTRA_COOLDOWN_PERIOD = 500;
    unsigned long omega_x_extend_trigger_timestamp = 0;
    unsigned long omega_x_flex_trigger_timestamp = 0;
    unsigned long extend_stop_timestamp = 0;
    unsigned long flex_stop_timestamp = 0;
};

void algorithm1(float omega_x, float elbow_angle);
void algorithm2(float omega_x, float elbow_angle);


#endif
