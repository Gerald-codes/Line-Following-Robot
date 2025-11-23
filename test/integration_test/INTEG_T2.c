/**
 * INTEG-T2.c
 *
 * Integration Test ID: INTEG-T2
 * Description:
 *   Integration test for line following using:
 *      - ONE IR line sensor (ADC)
 *      - Motor direction control (no PWM)
 *      - Simple steering logic
 *
 * Purpose:
 *   Verify closed-loop response:
 *       IR sensor → steering decision → motor direction
 *
 * Logs:
 *   - IR sensor value
 *   - steering decision
 *   - motor actions (FWD/LEFT/RIGHT)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// --------------------------------------------------------
// PIN DEFINITIONS
// --------------------------------------------------------
#define IR_LINE_ADC     26       // ADC0 = GP26

// Motor driver pins (direction only)
#define M1A 8      // Motor 1 control A
#define M1B 9      // Motor 1 control B
#define M2A 10     // Motor 2 control A
#define M2B 11     // Motor 2 control B

// --------------------------------------------------------
// PARAMETERS
// --------------------------------------------------------
#define LINE_THRESHOLD  2000   // Tune this based on calibration

// --------------------------------------------------------
// MOTOR HELPERS
// --------------------------------------------------------

static void motor_forward(void)
{
    gpio_put(M1A, 1);
    gpio_put(M1B, 0);
    gpio_put(M2A, 1);
    gpio_put(M2B, 0);
}

static void motor_turn_left(void)
{
    gpio_put(M1A, 0);
    gpio_put(M1B, 1);
    gpio_put(M2A, 1);
    gpio_put(M2B, 0);
}

static void motor_turn_right(void)
{
    gpio_put(M1A, 1);
    gpio_put(M1B, 0);
    gpio_put(M2A, 0);
    gpio_put(M2B, 1);
}

static void motor_stop(void)
{
    gpio_put(M1A, 0);
    gpio_put(M1B, 0);
    gpio_put(M2A, 0);
    gpio_put(M2B, 0);
}

// --------------------------------------------------------
static uint16_t read_line_sensor(void)
{
    adc_select_input(IR_LINE_ADC);
    return adc_read();
}

// --------------------------------------------------------
static void init_hardware(void)
{
    stdio_init_all();
    sleep_ms(1000);

    // ADC init
    adc_init();
    adc_gpio_init(26);  // GP26 = IR sensor

    // Motor pin init
    gpio_init(M1A); gpio_set_dir(M1A, GPIO_OUT);
    gpio_init(M1B); gpio_set_dir(M1B, GPIO_OUT);
    gpio_init(M2A); gpio_set_dir(M2A, GPIO_OUT);
    gpio_init(M2B); gpio_set_dir(M2B, GPIO_OUT);

    motor_stop();

    printf("INTEG-T2: Line Following (1 IR Sensor, No PWM) Starting...\n");
}

// --------------------------------------------------------
// MAIN
// --------------------------------------------------------
int main()
{
    init_hardware();

    while (true)
    {
        uint16_t ir = read_line_sensor();

        // ----------------------------------------------------
        // Line-following logic:
        //   IR > threshold → sensor on dark line
        // ----------------------------------------------------
        if (ir > LINE_THRESHOLD)
        {
            // Line detected → turn left
            motor_turn_left();
            printf("[IR=%u] LEFT TURN\n", ir);
        }
        else
        {
            // No line → turn right
            motor_turn_right();
            printf("[IR=%u] RIGHT TURN\n", ir);
        }

        sleep_ms(60);
    }

    return 0;
}
