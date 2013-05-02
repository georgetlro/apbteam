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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "rgb.hh"

RGB::RGB (ucoo::Gpio *EN_): S2(GPIOE, 8), S3(GPIOE, 11)
{
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
    int i;

    EN = EN_;
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
    // Enable clock for Timer 1.
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

    // Enable interrupts for TIM1.
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_set_mode(TIM1,
                   TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
                   TIM_CR1_CMS_EDGE,   // Edge synchronization
                   TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler(TIM1, 72-1);  // Counter unit = 1 us.
    timer_set_period(TIM1, 0xFFFF);
    timer_set_repetition_counter(TIM1, 0);
    timer_continuous_mode(TIM1);

    // Configure channel 1
    timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM1, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity(TIM1, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_prescaler(TIM1, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_enable(TIM1, TIM_IC1);
    timer_clear_flag(TIM1, TIM_SR_CC1IF);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    timer_enable_counter(TIM1);
}

void RGB::start ()
{
    min_color = 999;
    color = WHITE;
    sensor_setup(0);
}

void RGB::setup_color(int new_color)
{
    S2.output();
    S3.output();

    if (new_color == RED)
    {
        S2.set(0);
        S3.set(0);
    }
    else if (new_color == GREEN)
    {
        S2.set(1);
        S3.set(1);
    }
    else if (new_color == BLUE)
    {
        S2.set(0);
        S3.set(1);
    }
    else if (new_color == WHITE)
    {
        S2.set(1);
        S3.set(0);
    }
}

void RGB::tim1_isr(void)
{
    static u32 last_value = 0;
    static u32 value = 0;

    if (timer_get_flag(TIM1, TIM_SR_CC1IF) != 0) {
        // Interupt received
        timer_clear_flag(TIM1, TIM_SR_CC1IF);
        if (cur_color == UNKNOWN){
            // Disabling the sensor should stop the interupt also.
            sensor_setup(1);
            // Disable the IC
            // timer_ic_disable(TIM1, TIM_IC1);
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
        if (measure_cnt == 3)
        {
            // Grab the last results and move on to the next color
            results[cur_color] /= 3;

            cur_color++;
            setup_color(cur_color);
            measure_cnt = 0;
        }
    }
}

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

