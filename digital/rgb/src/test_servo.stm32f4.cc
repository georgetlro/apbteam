
#include "ucoolib/arch/arch.hh"
#include "ucoolib/hal/gpio/gpio.hh"
#include "ucoolib/utils/delay.hh"
#include "ucoolib/common.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>

// 66mhz => 15ms
#define PWM_PERIOD      (43290)

// ~2ms
// #define SERVO_MAX       (5772)
// 2.3ms
// #define SERVO_MAX       (6637)
// 1.70ms
#define SERVO_MAX       (4906)
// 1.55ms
// #define SERVO_MAX       (4473)

// ~1ms
// #define SERVO_MIN       (2867)
// 0.8ms
// #define SERVO_MIN       (2308)
// 0.6ms
#define SERVO_MIN       (1732)

int
main(int argc, const char **argv)
{
    ucoo::arch_init (argc, argv);
    uint32_t servo_pos = SERVO_MIN;
    uint32_t delay = 2;

     /* Enable timer clock. */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

    timer_reset(TIM2);

    timer_set_mode(TIM2,
            TIM_CR1_CKD_CK_INT,
            TIM_CR1_CMS_EDGE,
            TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM2, 20);
    timer_set_period(TIM2, PWM_PERIOD);
    timer_set_repetition_counter(TIM2, 0);
    timer_continuous_mode(TIM2);

    /* Set timer channel to output */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO1);

    timer_disable_oc_output(TIM2, TIM_OC2);
    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC2);

    timer_enable_counter(TIM2);

    timer_set_oc_value(TIM2, TIM_OC2, servo_pos);
    ucoo::delay_ms(275);
    while (1)
    {
        servo_pos += 10;
        delay = 1;
        if (servo_pos >= SERVO_MAX)
        {
            servo_pos = SERVO_MIN;
            delay = 275;
        }
        timer_set_oc_value(TIM2, TIM_OC2, servo_pos);
        ucoo::delay_ms(delay);

    }
    return 0;
}

