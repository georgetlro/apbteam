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
#include "rgb.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

static Rgb *rgb_instance;
static uint32_t last_color_value = 0;

// The interupt fonction will call one in our class.
extern "C" {
    void tim1_cc_isr () { Rgb::ic_isr (); }
}

Rgb::Rgb (int sensor_): S2(GPIOE, 8), S3(GPIOE, 11)
{
    // Get correct GPIO depending on which sensor we have.
    // This is the CANNON GPIO EN
    int gpio_en = 15;
    switch (sensor_)
    {
        case CHERRY_FAR:
            gpio_en = 10;
            break;
        case CHERRY_NEAR:
            gpio_en = 13;
            break;
    }
    ucoo::Gpio EN (GPIOE, gpio_en);
    EN.output ();
    S2.output ();
    S3.output ();

    sensor = sensor_;
    if (sensor == CANNON)
        // We want to use the BLUE filter only when using the cannon sensor
        setup_color (BLUE);

    disable_sensor ();
    measure_cnt = 0;
    color_value[4] = { 0 };
    // Keep a pointer to the class
    rgb_instance = this;
}

void
Rgb::enable (void)
{
    // Enable clock for Timer 1.
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

    /* Output GPIO */
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO9);

    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_reset(TIM1);
    timer_set_mode(TIM1,
            TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
            TIM_CR1_CMS_EDGE,   // Edge synchronization
            TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler(TIM1, 75);
    timer_set_period(TIM1, 0xFFFF);
    timer_set_repetition_counter(TIM1, 0);
    timer_continuous_mode(TIM1);

    /* Configure Input Capture Channel 1 */
    timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM1, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity(TIM1, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_prescaler(TIM1, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_enable(TIM1, TIM_IC1);
    timer_clear_flag(TIM1, TIM_SR_CC1IF);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    timer_enable_counter(TIM1);
}

void
Rgb::setup_color (int color)
{
    switch (color)
    {
        case RED:
            S2.set (0);
            S3.set (0);
            break;
        case BLUE:
            S2.set (0);
            S3.set (1);
            break;
        case WHITE:
            S2.set (1);
            S3.set (0);
            break;
        case GREEN:
            S2.set (1);
            S3.set (1);
            break;
    }
}

int
Rgb::calibrate_sensor (void)
{
    int offset = 0;

    // Do a measure to get current light situation
    do_measure ();

    // We only take into account the WHITE (or CLEAR since it's without
    // any filter) value.
    offset = BASIC_GREY - color_value[0];

    return offset;
}

int
Rgb::get_color (int offset)
{
    if (sensor == CANNON)
    {
        return get_cannon_color (offset);
    }
    else
    {
        return get_cherry_color ();
    }
    return 0;
}

void
Rgb::enable_sensor (void)
{
    EN.set (0);
}

void
Rgb::disable_sensor (void)
{
    EN.set (1);
}

void
Rgb::do_measure (void)
{
    enable_sensor ();
    // The interupt will disable the sensor
    while ( EN.get() == 0)
        ;
    // Grab the results in... color_value
}

int
Rgb::get_cannon_color (int offset)
{
    uint32_t curr_color_value = 0;
    cur_color = WHITE;
    color_value[0] = 0;

    do_measure ();

    curr_color_value = color_value[0] + offset;
    if (curr_color_value > 120 && curr_color_value < 165)
    {
        // we don't know if it's blue or red, and we don't care
        // It's colored and must be thrown out !
        return BLUE;
    }
    else if (curr_color_value > 40 && curr_color_value < 100)
    {
        return WHITE;
    }
    return NOTHING;
}

int
Rgb::get_cherry_color (void)
{
    int min_color = 999, results = NOTHING, color;
    cur_color = WHITE;
    setup_color (cur_color);

    do_measure ();

    // We don't care for white level
    for(color = WHITE; color != UNKNOWN; color++)
    {
        if (color == BLUE) {
            color_value[color] *= 0.8;
        }else if (color == RED){
            color_value[color] *= 1.2;
        }

        if (color_value[color] < min_color)
        {
            results = color;
            min_color = color_value[color];
        }
        color_value[color] = 0;
    }

    return results;
}

void
Rgb::ic_isr (void)
{
    Rgb &rgb = *rgb_instance;

    if (timer_get_flag(TIM1, TIM_SR_CC1IF) == 0) {
        return;
    }
    timer_clear_flag(TIM1, TIM_SR_CC1IF);

    // Throw away the first measurement
    if (rgb.measure_cnt == 0)
    {
        rgb.color_value[rgb.cur_color] = 0;
        rgb.measure_cnt++;
    }
    else if (rgb.measure_cnt == 1)
    {
        last_color_value = TIM1_CCR1;
        rgb.measure_cnt++;
    }
    else if (rgb.measure_cnt == 2)
    {
        if (TIM1_CCR1 > last_color_value)
        {
            rgb.color_value[rgb.cur_color] = (TIM1_CCR1 - last_color_value);
        }
        else
        {
            // Handle overflow
            // Our timer is 16bit, therefore the 65535 (2**16 -1)
            rgb.color_value[rgb.cur_color] = (65535 - last_color_value) + TIM1_CCR1;
        }
        if (rgb.sensor == CANNON || (rgb.sensor != CANNON && rgb.cur_color == UNKNOWN))
        {
            // There will no more interrupt since the sensor is shut.
            rgb.disable_sensor ();
        }
        else
        {
            // We need the 4 color measured
            ++rgb.cur_color;
            rgb.setup_color (rgb.cur_color);
        }
        rgb.measure_cnt = 0;
    }
}
