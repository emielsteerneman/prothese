#ifndef UTILS_H
#define UTILS_H

struct INTERRUPT
{
    volatile bool interrupt = false;
};

struct LEFTORRIGHT
{
    float transformation_matrix[3][3]; // Will be set based on input
    bool is_left_prosthetic = false;
};

float convertToRadians(float value_in_degrees);
void timerInterrupt();
void waitForLRInput(LEFTORRIGHT& LorR, bool& emergency_stop, bool& unsafe_encoder_measurements, bool& timer_interrupt, INTERRUPT& timer, bool BLUETOOTH);


#endif
