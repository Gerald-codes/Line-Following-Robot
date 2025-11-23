#include "servo.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Servo control variables
static uint servo_slice;
static uint servo_channel;
static uint32_t servo_wrap;
static int current_angle = 90;  // Track current angle

void servo_init(unsigned int servoPin) {
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(servoPin);
    servo_channel = pwm_gpio_to_channel(servoPin);
    
    uint32_t clock_freq = 125000000;
    uint32_t divider = 64;
    servo_wrap = (clock_freq / divider / SERVO_FREQ) - 1;
    
    pwm_set_clkdiv(servo_slice, divider);
    pwm_set_wrap(servo_slice, servo_wrap);
    pwm_set_enabled(servo_slice, true);
}

void servo_set_angle(int angle) {
    int adjusted_angle = angle + SERVO_OFFSET;
    
    if (adjusted_angle < 0) adjusted_angle = 0;
    if (adjusted_angle > 180) adjusted_angle = 180;
    
    uint32_t pulse_width = SERVO_MIN_PULSE + 
                          ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * adjusted_angle / 180);
    
    uint32_t level = (servo_wrap * pulse_width) / 20000;
    
    pwm_set_chan_level(servo_slice, servo_channel, level);
    current_angle = adjusted_angle;
}

int servo_get_angle(void) {
    return current_angle;
}