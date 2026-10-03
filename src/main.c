/*
 * MINI-ECU : bare-metal embedded drive control unit
 * Board    : ST NUCLEO-L476RG (STM32L476RG)        Driver : TB6612FNG
 * Clock    : MSI raised to 16 MHz (HCLK = PCLK1 = PCLK2 = 16 MHz)
 *
 * Morpho pin map
 *   PA0  CN7-28   potentiometer wiper         ADC1_IN5
 *   PA1  CN7-30   touch sensor START output   input, pull-down, active HIGH
 *   PB0  CN7-34   emergency stop button       EXTI0, pull-up, active LOW to GND
 *   PB12 CN10-16  orange LED
 *   PB13 CN10-30  green LED
 *   PB14 CN10-28  blue LED
 *   PB15 CN10-26  red LED
 *   PA2  CN10-35  USART2 TX  \ ST-Link virtual COM port (same USB cable)
 *   PA3  CN10-37  USART2 RX  /
 *   Motor driver (see motor.h): PC8 PWMA, PC9 PWMB, PC10 AIN1, PC11 AIN2,
 *                               PC12 BIN1, PD2 BIN2, PC5 STBY
 *
 * Both motors are driven together (one vehicle, left + right wheel).
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "motor.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */
#define F_CPU_HZ            16000000U
#define UART_BAUD           115200U

#define ADC_LOW             60U       /* below this the pot reads 0 %    */
#define ADC_HIGH            4035U     /* above this the pot reads 100 %  */

#define TOUCH_PIN           1U        /* PA1 */
#define TOUCH_ACTIVE_LEVEL  1U        /* TTP223-style module: HIGH = touched */
#define DEBOUNCE_MS         30U

#define REVERSE_DEADTIME_MS 150U
#define SPEED_UPDATE_MS     10U

#define ESTOP_PIN           0U        /* PB0 -> EXTI0 */

#define LED_ORANGE_PIN      12U
#define LED_GREEN_PIN       13U
#define LED_BLUE_PIN        14U
#define LED_RED_PIN         15U
#define LED_ORANGE          (1U << LED_ORANGE_PIN)
#define LED_GREEN           (1U << LED_GREEN_PIN)
#define LED_BLUE            (1U << LED_BLUE_PIN)
#define LED_RED             (1U << LED_RED_PIN)
#define LED_ALL             (LED_ORANGE | LED_GREEN | LED_BLUE | LED_RED)

/* ------------------------------------------------------------------ */
/* Types and shared state                                              */
/* ------------------------------------------------------------------ */
typedef enum { ST_IDLE = 0, ST_READY, ST_RUNNING, ST_ESTOP } state_t;
typedef enum { DIR_NONE = 0, DIR_FWD, DIR_REV } dir_t;

static volatile state_t  g_state  = ST_IDLE;
static volatile uint8_t  g_estop  = 0U;      /* set in the EXTI ISR          */
static volatile dir_t    g_dir    = DIR_NONE;
static volatile uint32_t g_pwm_pm = 0U;      /* output duty in permille      */
static volatile uint32_t g_ms     = 0U;

static uint32_t g_adc_acc    = 0U;           /* IIR accumulator (x16)        */
static uint32_t g_adc_filt   = 0U;           /* filtered 12-bit value        */
static bool     g_adc_seeded = false;

/* ------------------------------------------------------------------ */
/* GPIO helpers (modes: 0 in, 1 out, 2 AF, 3 analog)                   */
/* ------------------------------------------------------------------ */
static void gpio_mode(GPIO_TypeDef *p, uint32_t pin, uint32_t mode)
{
    p->MODER = (p->MODER & ~(3U << (pin * 2U))) | (mode << (pin * 2U));
}

static void gpio_pull(GPIO_TypeDef *p, uint32_t pin, uint32_t pull)
{   /* 0 none, 1 pull-up, 2 pull-down */
    p->PUPDR = (p->PUPDR & ~(3U << (pin * 2U))) | (pull << (pin * 2U));
}

static void gpio_af(GPIO_TypeDef *p, uint32_t pin, uint32_t af)
{
    uint32_t idx = pin >> 3U;
    uint32_t sh  = (pin & 7U) * 4U;
    p->AFR[idx] = (p->AFR[idx] & ~(0xFU << sh)) | (af << sh);
}

/* ------------------------------------------------------------------ */
/* Clock and time base                                                 */
/* ------------------------------------------------------------------ */
static void clock_init(void)
{
    RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE) | RCC_CR_MSIRANGE_8 | RCC_CR_MSIRGSEL;
    while (!(RCC->CR & RCC_CR_MSIRDY)) { }
}

void SysTick_Handler(void)
{
    g_ms++;
}

static void systick_init(void)
{
    SysTick->LOAD = (F_CPU_HZ / 1000U) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms;
    while ((g_ms - start) < ms) { }
}

/* ------------------------------------------------------------------ */
/* UART (USART2 -> ST-Link virtual COM port, polled)                   */
/* ------------------------------------------------------------------ */
static void uart_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;

    gpio_mode(GPIOA, 2U, 2U);
    gpio_mode(GPIOA, 3U, 2U);
    gpio_af(GPIOA, 2U, 7U);
    gpio_af(GPIOA, 3U, 7U);

    USART2->BRR = (F_CPU_HZ + UART_BAUD / 2U) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

static void uart_putc(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE)) { }
    USART2->TDR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) { uart_putc(*s++); }
}

static void uart_putu(uint32_t v)
{
    char b[11];
    int i = 0;
    if (v == 0U) { b[i++] = '0'; }
    while (v) { b[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (i--) { uart_putc(b[i]); }
}

/* ------------------------------------------------------------------ */
/* LEDs: one atomic BSRR write per state (set bits win over reset bits) */
/* ------------------------------------------------------------------ */
static void leds_show(state_t s)
{
    uint32_t on = 0U;
    switch (s) {
    case ST_IDLE:    on = LED_ORANGE; break;
    case ST_READY:   on = LED_GREEN;  break;
    case ST_RUNNING: on = LED_BLUE;   break;
    case ST_ESTOP:   on = LED_RED;    break;
    }
    GPIOB->BSRR = (LED_ALL << 16U) | on;
}

static void leds_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    gpio_mode(GPIOB, LED_ORANGE_PIN, 1U);
    gpio_mode(GPIOB, LED_GREEN_PIN,  1U);
    gpio_mode(GPIOB, LED_BLUE_PIN,   1U);
    gpio_mode(GPIOB, LED_RED_PIN,    1U);
    GPIOB->BSRR = (LED_ALL << 16U);
}

/* ------------------------------------------------------------------ */
/* Both-motor helpers                                                  */
/* ------------------------------------------------------------------ */
static void drive_both(motor_mode_t mode, uint32_t duty_permille)
{
    motor_set(MOTOR_A, mode, (uint16_t)duty_permille);
    motor_set(MOTOR_B, mode, (uint16_t)duty_permille);
}

static void coast_both(void)
{
    drive_both(MOTOR_COAST, 0U);
}

static motor_mode_t mode_for(dir_t d)
{
    return (d == DIR_REV) ? MOTOR_REV : MOTOR_FWD;
}

/* ------------------------------------------------------------------ */
/* ADC (ADC1 channel 5 = PA0, continuous, light IIR filter)            */
/* ------------------------------------------------------------------ */
static void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_ADCEN;
    (void)RCC->AHB2ENR;

    gpio_mode(GPIOA, 0U, 3U);               /* analog */
    gpio_pull(GPIOA, 0U, 0U);
    GPIOA->ASCR |= (1U << 0U);              /* L47x: connect analog switch to ADC */

    /* ADC clock = HCLK / 1 (synchronous) */
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~ADC_CCR_CKMODE) | ADC_CCR_CKMODE_0;

    /* Leave deep power-down, start the voltage regulator */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    delay_ms(2U);

    /* Single-ended calibration */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    /* 12-bit, right aligned, continuous, overwrite on overrun */
    ADC1->CFGR  = ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
    ADC1->SMPR1 = (6U << ADC_SMPR1_SMP5_Pos);   /* channel 5: 247.5 cycles */
    ADC1->SQR1  = (5U << ADC_SQR1_SQ1_Pos);     /* 1 conversion, channel 5 */

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }

    ADC1->CR |= ADC_CR_ADSTART;
}

static void adc_update(void)
{
    if (ADC1->ISR & ADC_ISR_OVR) {
        ADC1->ISR = ADC_ISR_OVR;
    }
    if (ADC1->ISR & ADC_ISR_EOC) {
        uint32_t v = ADC1->DR;              /* reading DR clears EOC */
        if (!g_adc_seeded) {
            g_adc_acc = v << 4U;
            g_adc_seeded = true;
        } else {
            g_adc_acc = g_adc_acc - (g_adc_acc >> 4U) + v;
        }
        g_adc_filt = g_adc_acc >> 4U;
    }
}

/* Linear ADC -> duty map in permille, with dead zones at both ends */
static uint32_t pot_to_permille(uint32_t adc)
{
    if (adc <= ADC_LOW)  { return 0U; }
    if (adc >= ADC_HIGH) { return MOTOR_PWM_MAX; }
    return ((adc - ADC_LOW) * MOTOR_PWM_MAX) / (ADC_HIGH - ADC_LOW);
}

static uint32_t permille_to_percent(uint32_t pm)
{
    return (pm + 5U) / 10U;
}

/* ------------------------------------------------------------------ */
/* Emergency stop: EXTI0 on PB0, falling edge                          */
/* ------------------------------------------------------------------ */
static bool estop_pin_pressed(void)
{
    return (GPIOB->IDR & (1U << ESTOP_PIN)) == 0U;
}

static void estop_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    gpio_mode(GPIOB, ESTOP_PIN, 0U);
    gpio_pull(GPIOB, ESTOP_PIN, 1U);        /* pull-up, button to GND */

    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xFU << 0U)) | (0x1U << 0U); /* PB0 */
    EXTI->FTSR1 |= (1U << ESTOP_PIN);
    EXTI->RTSR1 &= ~(1U << ESTOP_PIN);
    EXTI->PR1    = (1U << ESTOP_PIN);
    EXTI->IMR1  |= (1U << ESTOP_PIN);

    NVIC_SetPriority(EXTI0_IRQn, 0U);       /* highest priority */
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << ESTOP_PIN)) {
        EXTI->PR1 = (1U << ESTOP_PIN);      /* clear pending flag */
        motor_all_coast();                  /* PWM low, inputs low, STBY low */
        g_pwm_pm = 0U;
        g_dir    = DIR_NONE;
        g_estop  = 1U;
        g_state  = ST_ESTOP;
        leds_show(ST_ESTOP);
    }
}

/* ------------------------------------------------------------------ */
/* State transitions (main context). Each one runs with interrupts off  */
/* and re-checks g_estop, so nothing can override an emergency stop.    */
/* ------------------------------------------------------------------ */
static bool enable_system(void)             /* IDLE -> READY */
{
    bool ok = false;
    __disable_irq();
    if (!g_estop && g_state == ST_IDLE) {
        motor_enable(1U);                   /* wake the TB6612 (STBY high) */
        g_state = ST_READY;
        leds_show(ST_READY);
        ok = true;
    }
    __enable_irq();
    return ok;
}

static bool try_run(uint32_t pm, dir_t dir) /* READY/RUNNING -> RUNNING */
{
    bool ok = false;
    __disable_irq();
    if (!g_estop) {
        drive_both(mode_for(dir), pm);
        g_dir    = dir;
        g_pwm_pm = pm;
        g_state  = ST_RUNNING;
        leds_show(ST_RUNNING);
        ok = true;
    }
    __enable_irq();
    return ok;
}

static void stop_to_ready(void)             /* RUNNING -> READY */
{
    __disable_irq();
    if (!g_estop) {
        coast_both();
        g_pwm_pm = 0U;
        g_dir    = DIR_NONE;
        g_state  = ST_READY;
        leds_show(ST_READY);
    }
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* Touch START (debounced rising edge)                                 */
/* ------------------------------------------------------------------ */
static void touch_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    gpio_mode(GPIOA, TOUCH_PIN, 0U);
    gpio_pull(GPIOA, TOUCH_PIN, 2U);        /* pull-down */
}

static bool touch_event(void)
{
    static uint8_t  last_raw = 0U;
    static uint8_t  stable   = 0U;
    static uint32_t t_change = 0U;

    uint8_t raw = (((GPIOA->IDR >> TOUCH_PIN) & 1U) == TOUCH_ACTIVE_LEVEL) ? 1U : 0U;
    if (raw != last_raw) {
        last_raw = raw;
        t_change = g_ms;
    }
    if (((g_ms - t_change) >= DEBOUNCE_MS) && (raw != stable)) {
        stable = raw;
        if (stable) { return true; }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* UART status report                                                  */
/* ------------------------------------------------------------------ */
static void print_status(void)
{
    state_t s = g_state;

    uart_puts("\r\n--------------------------------\r\n");
    uart_puts("     MINI ECU STATUS\r\n");
    uart_puts("--------------------------------\r\n");

    uart_puts("STATE      : ");
    switch (s) {
    case ST_IDLE:    uart_puts("IDLE");       break;
    case ST_READY:   uart_puts("READY");      break;
    case ST_RUNNING: uart_puts("RUNNING");    break;
    case ST_ESTOP:   uart_puts("EMERGENCY");  break;
    }
    uart_puts("\r\nSYSTEM     : ");
    uart_puts((s == ST_READY || s == ST_RUNNING) ? "ENABLED" : "DISABLED");
    uart_puts("\r\nMOTOR      : ");
    uart_puts((s == ST_RUNNING) ? "ON" : "OFF");
    uart_puts("\r\nDIRECTION  : ");
    if (s == ST_RUNNING) {
        uart_puts((g_dir == DIR_REV) ? "REVERSE" : "FORWARD");
    } else {
        uart_puts("-");
    }
    uart_puts("\r\nPWM        : ");
    uart_putu(permille_to_percent((s == ST_RUNNING) ? g_pwm_pm : 0U));
    uart_puts("%\r\nPOT SETTING: ");
    uart_putu(permille_to_percent(pot_to_permille(g_adc_filt)));
    uart_puts("%\r\nADC VALUE  : ");
    uart_putu(g_adc_filt);
    uart_puts("\r\nEMERGENCY  : ");
    uart_puts(g_estop ? "YES" : "NO");
    uart_puts("\r\n--------------------------------\r\n");
}

/* ------------------------------------------------------------------ */
/* Command handlers                                                    */
/* ------------------------------------------------------------------ */
static void cmd_move(dir_t dir)
{
    if (g_estop) {
        uart_puts("ERR: emergency stop active, send RESET\r\n");
        return;
    }
    if (g_state == ST_IDLE) {
        uart_puts("ERR: system not enabled, touch START first\r\n");
        return;
    }

    uint32_t pm = pot_to_permille(g_adc_filt);
    if (pm == 0U) {
        if (g_state == ST_RUNNING) { stop_to_ready(); }
        uart_puts("PWM is 0%, motor stays OFF\r\n");
        return;
    }

    if (g_state == ST_RUNNING && g_dir != dir) {
        coast_both();                       /* dead time before reversing */
        delay_ms(REVERSE_DEADTIME_MS);
    }
    uart_puts(try_run(pm, dir) ? "OK: moving\r\n"
                               : "ERR: emergency stop active\r\n");
}

static void cmd_stop(void)
{
    if (g_estop) {
        uart_puts("ERR: emergency stop active, send RESET\r\n");
        return;
    }
    if (g_state == ST_IDLE) {
        uart_puts("ERR: system not enabled\r\n");
        return;
    }
    stop_to_ready();
    uart_puts("OK: stopped, READY\r\n");
}

static void cmd_reset(void)
{
    if (!g_estop) {
        uart_puts("No emergency stop active\r\n");
        return;
    }
    if (estop_pin_pressed()) {
        uart_puts("ERR: release the E-STOP button first\r\n");
        return;
    }
    __disable_irq();
    EXTI->PR1 = (1U << ESTOP_PIN);          /* drop any bounce edge */
    motor_all_coast();                      /* stays in standby until START */
    g_pwm_pm = 0U;
    g_dir    = DIR_NONE;
    g_estop  = 0U;
    g_state  = ST_IDLE;
    leds_show(ST_IDLE);
    __enable_irq();
    uart_puts("E-STOP cleared, system IDLE. Touch START to enable.\r\n");
}

static void handle_command(const char *c)
{
    if      (strcmp(c, "STATUS") == 0) { print_status(); }
    else if (strcmp(c, "RESET")  == 0) { cmd_reset(); }
    else if (strcmp(c, "F")      == 0) { cmd_move(DIR_FWD); }
    else if (strcmp(c, "R")      == 0) { cmd_move(DIR_REV); }
    else if (strcmp(c, "S")      == 0) { cmd_stop(); }
    else { uart_puts("Unknown command. Use F, R, S, STATUS, RESET\r\n"); }
}

/* Non-blocking line reader: commands end with Enter (CR or LF) */
static void uart_poll(void)
{
    static char    buf[12];
    static uint8_t len = 0U;

    uint32_t isr = USART2->ISR;
    if (isr & USART_ISR_ORE) {
        USART2->ICR = USART_ICR_ORECF;
    }
    if (!(isr & USART_ISR_RXNE)) { return; }
    char c = (char)USART2->RDR;

    if (c == '\r' || c == '\n') {
        if (len > 0U) {
            buf[len] = '\0';
            uart_puts("\r\n");
            handle_command(buf);
            len = 0U;
            uart_puts("> ");
        }
    } else if (c == 0x08 || c == 0x7F) {    /* backspace */
        if (len > 0U) { len--; uart_puts("\b \b"); }
    } else if (c >= ' ' && len < (sizeof(buf) - 1U)) {
        if (c >= 'a' && c <= 'z') { c = (char)(c - 32); }
        buf[len++] = c;
        uart_putc(c);                       /* echo */
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */
int main(void)
{
    clock_init();
    systick_init();
    leds_init();
    leds_show(ST_IDLE);         /* orange ON while initialising              */
    motor_init();               /* PWM forced low, inputs low, STBY low      */
    uart_init();
    touch_init();
    adc_init();
    estop_init();

    /* Power-up is IDLE. If E-STOP is already held, start in EMERGENCY. */
    if (estop_pin_pressed()) {
        __disable_irq();
        motor_all_coast();
        g_estop = 1U;
        g_state = ST_ESTOP;
        leds_show(ST_ESTOP);
        __enable_irq();
    }

    uart_puts("\r\nMINI ECU ready. Commands: F, R, S, STATUS, RESET (end with Enter)\r\n> ");

    uint32_t last_update = 0U;

    while (1) {
        adc_update();
        uart_poll();

        if (touch_event() && g_state == ST_IDLE && !g_estop) {
            if (enable_system()) {
                uart_puts("\r\nSystem ENABLED (READY)\r\n> ");
            }
        }

        /* While running, follow the pot live; pot at zero returns to READY */
        if (g_state == ST_RUNNING && (g_ms - last_update) >= SPEED_UPDATE_MS) {
            last_update = g_ms;
            uint32_t pm = pot_to_permille(g_adc_filt);
            if (pm == 0U) {
                stop_to_ready();
            } else if (pm != g_pwm_pm) {
                __disable_irq();
                if (!g_estop && g_state == ST_RUNNING) {
                    drive_both(mode_for(g_dir), pm);
                    g_pwm_pm = pm;
                }
                __enable_irq();
            }
        }
    }
}