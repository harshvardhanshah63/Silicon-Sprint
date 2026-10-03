#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * TB6612FNG dual motor driver stage for NUCLEO-L476RG (Morpho pins only)
 *
 *   PWMA  PC8   CN10 pin 2   TIM3_CH3 PWM
 *   PWMB  PC9   CN10 pin 1   TIM3_CH4 PWM
 *   AIN1  PC10  CN7  pin 1
 *   AIN2  PC11  CN7  pin 2
 *   BIN1  PC12  CN7  pin 3
 *   BIN2  PD2   CN7  pin 4
 *   STBY  PC5   CN10 pin 6   low = driver in standby (all outputs off)
 */

typedef enum { MOTOR_A = 0, MOTOR_B = 1 } motor_id_t;

typedef enum {
    MOTOR_COAST = 0,   /* IN1 = IN2 = 0 : outputs off, motor free-wheels      */
    MOTOR_FWD,         /* IN1 = 1, IN2 = 0, PWM = duty (low phase = brake)    */
    MOTOR_REV,         /* IN1 = 0, IN2 = 1, PWM = duty (low phase = brake)    */
    MOTOR_BRAKE        /* IN1 = IN2 = 1 : motor terminals shorted (short brake) */
} motor_mode_t;

#define MOTOR_PWM_MAX   1000U      /* duty is given in permille (0..1000) */

void motor_init(void);                       /* driver stays in STANDBY after init */
void motor_enable(uint8_t on);               /* STBY high (1) or low (0)            */
void motor_set(motor_id_t m, motor_mode_t mode, uint16_t duty_permille);
void motor_all_coast(void);                  /* hard off: PWM low, IN low, STBY low.
                                                safe to call from an interrupt handler.
                                                Call motor_enable(1) to wake the driver. */

#endif