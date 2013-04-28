// rgb - Drives a sensors on an STM32F4 card. {{{
//
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


#include "ucoolib/arch/arch.hh"
#include "ucoolib/hal/gpio/gpio.hh"
#include "ucoolib/utils/delay.hh"
#include "ucoolib/common.hh"

#ifndef TARGET_host
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#endif

#include "rgb.hh"

RGB::RGB (ucoo::Gpio *EN_, ucoo::Gpio *S2_, ucoo::Gpio *S3_)
{
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN | RCC_AHB1ENR_IOPDEN);
    int i;

    EN = EN_;
    S2 = S2_;
    S3 = S3_;
    value_ready = false;
    for (i = 0; i<4;i++)
    {
        results[i] = 0;
    }
    measure_cnt = 0;
    cur_color = WHITE;

    sensor_setup(1);
    setup_input_capture();
}

void RGB::sensor_setup(int enable)
{
    // enable = 0 => enable the sensor, 1 disable
    EN->output();
    EN->set(enable);
}

void RGB::setup_input_capture()
{
    // Enable clock for Timer 2.
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

    // The button is mapped to TIM2_CH1
    // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    // gpio_set_af(GPIOA, GPIO_AF1, GPIO5);

    // Enable interrupts for TIM2.
    nvic_enable_irq(NVIC_TIM2_IRQ);

    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
                   TIM_CR1_CMS_EDGE,   // Edge synchronization
                   TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler(TIM2, 72-1);  // Counter unit = 1 us.
    timer_set_period(TIM2, 0xFFFF);
    timer_set_repetition_counter(TIM2, 0);
    timer_continuous_mode(TIM2);

    // Configure channel 1
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM2, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity(TIM2, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_prescaler(TIM2, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_enable(TIM2, TIM_IC1);
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    timer_enable_counter(TIM2);
}

void RGB::start ()
{
    min_color = 999;
    color = WHITE;
    sensor_setup(0);
}

void RGB::setup_color(int new_color)
{
    S2->output();
    S3->output();

    if (new_color == RED)
    {
        S2->set(0);
        S3->set(0);
    }
    else if (new_color == GREEN)
    {
        S2->set(1);
        S3->set(1);
    }
    else if (new_color == BLUE)
    {
        S2->set(0);
        S3->set(1);
    }
    else if (new_color == WHITE)
    {
        S2->set(1);
        S3->set(0);
    }
}

#ifdef TARGET_host
void RGB::tim2_isr(void)
{
    static u32 last_value = 0;
    static u32 value = 0;

    if (timer_get_flag(TIM2, TIM_SR_CC1IF) != 0) {
        // Interupt received
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        if (cur_color == UNKNOWN){
            // Disabling the sensor should stop the interupt also.
            sensor_setup(1);
        }
        // We want to throw a measure before changing color
        // since we do a "loop" of all the colors, we skip one out of two
        // measures.
        if (measure_cnt == 0)
        {
            // Leave this measure alone
            results[cur_color] = 0;
            measure_cnt++;
        }
        else if (measure_cnt%2 == 1)
        {
            // First measure that counts.
            last_value = TIM2_CCR1;
            measure_cnt++;
        }
        else if (measure_cnt%2 == 0)
        {
            // Get a results
            value = (TIM2_CCR1 - last_value);
            results[cur_color] += value;
            measure_cnt++;
        }
        // Means we have done 3 measure
        if (measure_cnt == 41)
        {
            // Grab the last results and move on to the next color
            results[cur_color] /= 20;

            cur_color++;
            setup_color(cur_color);
            measure_cnt = 0;
        }
    }
}
#endif

bool RGB::poke ()
{
    int i;
    if (EN->get() == 0)
    {
        value_ready = true;
        for(i = 0; i < 4; i++)
        {
            if (i >= 1 && results[i] < min_color)
            {
                color = i;
                min_color = results[i];
            }
            results[i] = 0;
        }
    }
    return value_ready;
}

int RGB::get ()
{
    if (true == value_ready)
    {
        return color;
    }
    else
    {
        return NOT_READY;
    }
}

