#include "ucoolib/arch/arch.hh"
#include "ucoolib/hal/gpio/gpio.hh"
#include "ucoolib/utils/delay.hh"
#include "ucoolib/common.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>

/**
 * We need a 50 Hz period (1000 / 20ms = 50), thus devide 100000 by 50 = 20000 (us).
 */
#define PWM_PERIOD      (20000)

/**
 * Max. pos. at 2050 us (4.30ms).
 */
#define SERVO_MAX       (5000)

/**
 * Min. pos. at 950  us (0.90ms).
 */
#define SERVO_MIN       (900)

/**
 * Middle pos. at 1580 us (1.58ms).
 */
#define SERVO_NULL      (1580)

int
main(int argc, const char **argv)
{
    ucoo::arch_init (argc, argv);

     /* Enable timer clock. */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

    timer_reset(TIM2);

    timer_set_mode(TIM2,
            TIM_CR1_CKD_CK_INT,
            TIM_CR1_CMS_EDGE,
            TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM2, 20 - 1);
    timer_set_period(TIM2, PWM_PERIOD);
    timer_set_repetition_counter(TIM2, 0);
    timer_continuous_mode(TIM2);

    /* Set timer channel to output */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO1);

    timer_disable_oc_output(TIM2, TIM_OC2);
    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    // timer_set_oc_value(TIM2, TIM_OC2, 0);
    timer_enable_oc_output(TIM2, TIM_OC2);

    // timer_set_oc_value(TIM2, TIM_OC2, SERVO_NULL);

    timer_enable_counter(TIM2);

    while (1)
    {
        timer_set_oc_value(TIM2, TIM_OC2, SERVO_MIN);
        ucoo::delay(1);
        timer_set_oc_value(TIM2, TIM_OC2, SERVO_MAX);
        ucoo::delay(1);
    }
    return 0;
}

