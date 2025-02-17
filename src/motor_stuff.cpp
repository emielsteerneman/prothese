#include "motor_stuff.h"

#include <ArduinoBLE.h>
#include <Serial.h>
#include <nrfx_pwm.h>
#include <stdint.h>

#define MOTOR_DIR_PIN 4
#define MOTOR_ENABLE_PIN 5
#define NRFX_STEP_PIN 4  // Use Arduino pin D14 (nRF GPIO P0.04)

#define LOW  0  // This is not needed but it removes the dumb incorrect IntelliSense error
#define HIGH 1  // This is not needed but it removes the dumb incorrect IntelliSense error

nrfx_pwm_t PWM_INSTANCE = NRFX_PWM_INSTANCE(0);
nrfx_pwm_config_t PWM_CONFIGURATION = {0};
nrf_pwm_sequence_t PWM_SEQUENCE = {0};
uint16_t duty_cycle = 0; // DO NOT MODIFY THIS ANYWHERE. 

void enable_motor(){
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void disable_motor(){
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void setup_motor_control(){
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, LOW);
    
    disable_motor();

    duty_cycle = 0;

    PWM_CONFIGURATION = {
        .output_pins = { NRFX_STEP_PIN | NRFX_PWM_PIN_INVERTED, 
                            NRFX_PWM_PIN_NOT_USED,
                            NRFX_PWM_PIN_NOT_USED, 
                            NRFX_PWM_PIN_NOT_USED },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_8MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 0,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    PWM_SEQUENCE = {
        .values = {.p_common = &duty_cycle },
        .length          = 1,
        .repeats         = 0,
        .end_delay       = 0
    };
}

bool turn_steps_per_second(uint32_t steps_per_second){
    
    // The clock runs at 8MHz. This means it counts 8 000 000 "ticks" in one second.
    // Every cycle of the PWM is two steps. One up, one down. If we want to have N steps
    // per second, we need to complete N/2 cycles per second. This means that we need to
    // divide 8 000 000 ticks over N/2 cycles. Each cycle therefore consists of 
    // 8 000 000 / (N/2) ticks. This will be our .top_value.
    uint32_t n_ticks_per_cycle = 8000000 / (steps_per_second / 2);

    // The type of the .top_value is uint16_t. This means that the maximum value is 65535. If 
    // we exceed this value, we will get an overflow. We need to check if the n_ticks_per_cycle
    // is larger than 65535. If it is, we need to return false.
    if(65535 < n_ticks_per_cycle){
        return false;
    }

    // Now, we can set the .top_value to the n_ticks_per_cycle.
    PWM_CONFIGURATION.top_value = (uint16_t) n_ticks_per_cycle;

    // The duty cycle should be at 50%. In other words, .top_value / 2. (n_ticks_per_cycle / 2)
    duty_cycle = (uint16_t) n_ticks_per_cycle / 2;

    // (Re)-initialize the PWM instance with the new configuration.
    nrfx_pwm_uninit(&PWM_INSTANCE);
    nrfx_pwm_init(&PWM_INSTANCE, &PWM_CONFIGURATION, NULL);
    // Start the PWM with the new duty cycle.
    nrfx_pwm_simple_playback(&PWM_INSTANCE, &PWM_SEQUENCE, 1, NRFX_PWM_FLAG_LOOP);

    enable_motor();

    return true;
}

void test1_pwm(){
    setup_motor_control();
    delay(1000);
    
    int direction = 1;
    int dir = LOW;

    uint32_t steps_per_second = 1000;
    while(true){

        steps_per_second += direction * 200;
        if(steps_per_second < 1000 || 50000 < steps_per_second){
            direction = -direction;
            if(steps_per_second < 1000){
                if(dir == LOW){
                    dir = HIGH;
                } else {
                    dir = LOW;
                }
                digitalWrite(MOTOR_DIR_PIN, dir);
            }
        }
        turn_steps_per_second(steps_per_second);
        delay(25);
    }
}

void run_emiel_motor_test(){

    if(!Serial){
        Serial.begin(115200);
    }

    delay(1000);
    Serial.println("Hello from emiel_motor_test!");

    test1_pwm();    
}