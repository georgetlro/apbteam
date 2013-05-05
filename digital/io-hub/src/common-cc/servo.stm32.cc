// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Maxime Hadjinlian
//
// APBTeam:
//        Web: http://apbteam.org/
//      Email: team AT apbteam DOT org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
// }}}
#include "servo.hh"

#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>

Servo::Servo ()
{
}

void
Servo::enable ()
{
    rcc_peripheral_enable_clock (&RCC_APB2ENR, RCC_APB2ENR_TIM8EN);
    timer_reset(TIM8);

    timer_set_mode(TIM8,
            TIM_CR1_CKD_CK_INT,
            TIM_CR1_CMS_EDGE,
            TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM8, 20);
    // 66Mhz which seems to be a pretty good fit for servos
    timer_set_period(TIM8, 43290);
    timer_set_repetition_counter(TIM8, 0);
    timer_continuous_mode(TIM8);

    // RGB servo 1
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_af(GPIOC, GPIO_AF3, GPIO7);

    // Cherry servo 2
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_af(GPIOC, GPIO_AF3, GPIO8);

    // RGB servo 1
    timer_disable_oc_output(TIM8, TIM_OC2);
    timer_set_oc_mode(TIM8, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM8, TIM_OC2);

    // Cherry servo 2
    timer_disable_oc_output(TIM8, TIM_OC3);
    timer_set_oc_mode(TIM8, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM8, TIM_OC3);

    timer_enable_counter(TIM8);

    // Set null position, so the servo doesn't move.
    set_position(SERVO_RGB, 0);
    set_position(SERVO_CHERRY, 0);
}

void
Servo::set_position(int servo, int pos)
{
    if (servo == SERVO_RGB)
    {
        timer_set_oc_value(TIM8, TIM_OC2, pos);
    }
    else if (servo == SERVO_CHERRY)
    {
        timer_set_oc_value(TIM8, TIM_OC3, pos);
    }
}

