#include "stm32l4xx.h"
#include "motor.h"

/* 16 MHz timer clock / 800 = 20 kHz PWM. The TB6612FNG accepts up to 100 kHz,
 * so this can be changed freely: 1599 -> 10 kHz, 399 -> 40 kHz. */
#define PWM_ARR         799U

/* Output-compare mode values (OCxM field) */
#define OCM_FORCE_LOW   4U      /* output forced low (PWM pin off) */
#define OCM_PWM1        6U      /* normal PWM                      */

/* TIM3 CCMR2: CH3 -> OC3M bits 6:4, OC3PE bit 3
 *             CH4 -> OC4M bits 14:12, OC4PE bit 11 */
#define OC3M_SHIFT      4U
#define OC4M_SHIFT      12U

/* Direction and standby pins */
#define AIN1_PIN  10U   /* PC10 */
#define AIN2_PIN  11U   /* PC11 */
#define BIN1_PIN  12U   /* PC12 */
#define BIN2_PIN  2U    /* PD2  */
#define STBY_PIN  5U    /* PC5  */

static void gpio_mode(GPIO_TypeDef *p, uint32_t pin, uint32_t mode)
{
    p->MODER = (p->MODER & ~(3U << (pin * 2U))) | (mode << (pin * 2U));
}

static void gpio_af(GPIO_TypeDef *p, uint32_t pin, uint32_t af)
{
    uint32_t idx = pin >> 3U;
    uint32_t sh  = (pin & 7U) * 4U;
    p->AFR[idx] = (p->AFR[idx] & ~(0xFU << sh)) | (af << sh);
}

/* Set the two direction inputs of one bridge */
static void set_inputs(motor_id_t m, uint32_t in1, uint32_t in2)
{
    if (m == MOTOR_A) {
        GPIOC->BSRR = ((in1 ? (1U << AIN1_PIN) : (1U << (AIN1_PIN + 16U))) |
                       (in2 ? (1U << AIN2_PIN) : (1U << (AIN2_PIN + 16U))));
    } else {
        GPIOC->BSRR =  (in1 ? (1U << BIN1_PIN) : (1U << (BIN1_PIN + 16U)));
        GPIOD->BSRR =  (in2 ? (1U << BIN2_PIN) : (1U << (BIN2_PIN + 16U)));
    }
}

static void set_pwm_mode(motor_id_t m, uint32_t ocm)
{
    uint32_t shift = (m == MOTOR_A) ? OC3M_SHIFT : OC4M_SHIFT;
    uint32_t pe    = (m == MOTOR_A) ? TIM_CCMR2_OC3PE : TIM_CCMR2_OC4PE;
    uint32_t v     = TIM3->CCMR2;

    v &= ~((7U << shift) | pe);
    v |= (ocm << shift) | pe;
    TIM3->CCMR2 = v;
}

static void set_ccr(motor_id_t m, uint32_t ccr)
{
    if (m == MOTOR_A) { TIM3->CCR3 = ccr; }
    else              { TIM3->CCR4 = ccr; }
}

void motor_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    (void)RCC->APB1ENR1;

    /* STBY first, so the driver is in standby (ODR resets to 0 = low) */
    gpio_mode(GPIOC, STBY_PIN, 1U);

    /* Direction pins: push-pull outputs, start low */
    gpio_mode(GPIOC, AIN1_PIN, 1U);
    gpio_mode(GPIOC, AIN2_PIN, 1U);
    gpio_mode(GPIOC, BIN1_PIN, 1U);
    gpio_mode(GPIOD, BIN2_PIN, 1U);

    /* TIM3 timer: PWM outputs forced low so nothing moves at start-up */
    TIM3->PSC   = 0U;
    TIM3->ARR   = PWM_ARR;
    TIM3->CCR3  = 0U;
    TIM3->CCR4  = 0U;
    TIM3->CCMR2 = (OCM_FORCE_LOW << OC3M_SHIFT) | TIM_CCMR2_OC3PE |
                  (OCM_FORCE_LOW << OC4M_SHIFT) | TIM_CCMR2_OC4PE;
    TIM3->CCER  = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;

    /* Only now hand PC8/PC9 to the timer */
    gpio_mode(GPIOC, 8U, 2U);
    gpio_mode(GPIOC, 9U, 2U);
    gpio_af(GPIOC, 8U, 2U);
    gpio_af(GPIOC, 9U, 2U);
}

void motor_enable(uint8_t on)
{
    GPIOC->BSRR = on ? (1U << STBY_PIN) : (1U << (STBY_PIN + 16U));
}

/* Hard off, safe inside an ISR: PWM low, inputs low, driver into standby */
void motor_all_coast(void)
{
    TIM3->CCMR2 = (OCM_FORCE_LOW << OC3M_SHIFT) | TIM_CCMR2_OC3PE |
                  (OCM_FORCE_LOW << OC4M_SHIFT) | TIM_CCMR2_OC4PE;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
    GPIOC->BSRR = (1U << (STBY_PIN + 16U)) |
                  (1U << (AIN1_PIN + 16U)) | (1U << (AIN2_PIN + 16U)) |
                  (1U << (BIN1_PIN + 16U));
    GPIOD->BSRR = (1U << (BIN2_PIN + 16U));
}

/* Not for use inside an interrupt handler (it restores the PRIMASK it found) */
void motor_set(motor_id_t m, motor_mode_t mode, uint16_t duty_permille)
{
    if (duty_permille > MOTOR_PWM_MAX) { duty_permille = MOTOR_PWM_MAX; }
    if ((mode == MOTOR_FWD || mode == MOTOR_REV) && duty_permille == 0U) {
        mode = MOTOR_COAST;                 /* 0 % means off */
    }

    uint32_t ccr = ((uint32_t)duty_permille * (PWM_ARR + 1U)) / MOTOR_PWM_MAX;

    /* CCMR2 is shared by both channels and is read-modify-written below, so
     * keep the emergency-stop interrupt out between the read and the write. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    switch (mode) {
    case MOTOR_FWD:
        set_inputs(m, 1U, 0U);
        set_ccr(m, ccr);
        set_pwm_mode(m, OCM_PWM1);
        break;
    case MOTOR_REV:
        set_inputs(m, 0U, 1U);
        set_ccr(m, ccr);
        set_pwm_mode(m, OCM_PWM1);
        break;
    case MOTOR_BRAKE:
        set_pwm_mode(m, OCM_FORCE_LOW);
        set_ccr(m, 0U);
        set_inputs(m, 1U, 1U);
        break;
    case MOTOR_COAST:
    default:
        set_pwm_mode(m, OCM_FORCE_LOW);
        set_ccr(m, 0U);
        set_inputs(m, 0U, 0U);
        break;
    }

    __set_PRIMASK(primask);
}