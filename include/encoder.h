#ifndef ENCODER_H
#define ENCODER_H

struct encoder_data {
    uint16_t encoder_value = 0;
    double elbow_angle = 0;
    double previous_elbow_angle = 0;
    double previous_encoder_value = 0; // previous encoder value
    unsigned long previous_encoder_timestamp = 0;
};

// functions

#endif