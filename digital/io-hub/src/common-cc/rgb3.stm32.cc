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
#include "rgb2.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

static Rgb *rgb_instance;

// The interupt fonction will call one in our class.
extern "C" {
    void tim1_cc_isr () { Rgb::ic_isr (); }
}

Rgb::Rgb (ucoo::Gpio *EN)
    : type (0)
{
    EN_ = EN;
    rgb_instance = this;
    color_value[WHITE] = 0;
    color_value[RED] = 0;
    color_value[BLUE] = 0;
    color_value[GREEN] = 0;
}

void
Rgb::enable ()
{
    // Disable the sensor to be sure
    disable_sensor ();

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
    ucoo::Gpio S2(GPIOE, 8);
    ucoo::Gpio S3(GPIOE, 11);
    S2.output ();
    S3.output ();

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

uint32_t
Rgb::calibrate_sensor ()
{
    // Do a measure to get current light situation
    do_measure ();

    // We only take into account the WHITE (or CLEAR since it's without
    // any filter) value.
    return BASIC_GREY - color_value[0];
}

void
Rgb::enable_sensor ()
{
    EN_->set (0);
}

void
Rgb::disable_sensor ()
{
    EN_->set (1);
}

void
Rgb::do_measure ()
{
    enable_sensor ();
    // The interupt will disable the sensor
    while ( EN_->get() == 0)
        ;
    // Grab the results in... color_value
}

uint32_t
Rgb::get_cannon_color (uint32_t offset)
{
    type = CANNON;
    setup_color (BLUE);

    uint32_t curr_color_value = 0;
    color_value[WHITE] = 0;

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

uint32_t
Rgb::get_candle_color ()
{
    type = CANDLE;
    uint32_t min_color = 999, results = NOTHING, color;
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

int
Rgb::rgb_to_hue (int r, int g, int b)
{
    uint8_t rgb_min, rgb_max, hue, sat, val;

    // find minimal between r, g and b
    if (g <= b){
        if (r <= g){
            rgb_min = r;
        }else{
            rgb_min = g;
        }
    }else if (r <= b){
        rgb_min = r;
    }else{
        rgb_min = b;
    }

    // find maximal between r, g and b
    if (g >= b){
        if (r >= g){
            rgb_max = r;
        }else{
            rgb_max = g;
        }
    }else if (r >= b){
        rgb_max = r;
    }else{
        rgb_max = b;
    }

    val = rgb_max;
    if (val == 0)
        return 0;

    sat = 255 * (rgb_max - rgb_min) / val;
    if (sat == 0)
        return 0;

    /* Compute hue */
    if (rgb_max == r) {
        hue = 0 + 43*(g - b)/(rgb_max - rgb_min);
    } else if (rgb_max == g) {
        hue = 85 + 43*(b - r)/(rgb_max - rgb_min);
    } else /* rgb_max == b */ {
        hue = 171 + 43*(r - g)/(rgb_max - rgb_min);
    }

    return hue;
}

void
Rgb::ic_isr ()
{
    Rgb &rgb = *rgb_instance;

    if (timer_get_flag(TIM1, TIM_SR_CC1IF) == 0) {
        return;
    }
    timer_clear_flag(TIM1, TIM_SR_CC1IF);

    // Throw away the first measurement
    if (measure_cnt == 0)
    {
        color_value[cur_color] = 0;
        measure_cnt++;
    }
    else if (measure_cnt == 1)
    {
        last_color_value = TIM1_CCR1;
        measure_cnt++;
    }
    else if (measure_cnt == 2)
    {
        if (TIM1_CCR1 > last_color_value)
        {
            color_value[cur_color] = (TIM1_CCR1 - last_color_value);
        }
        else
        {
            // Handle overflow
            // Our timer is 16bit, therefore the 65535 (2**16 -1)
            color_value[cur_color] = (65535 - last_color_value) + TIM1_CCR1;
        }
        if (rgb.type == CANNON || (rgb.type != CANNON && cur_color == UNKNOWN))
        {
            // There will no more interrupt since the sensor is shut.
            rgb.disable_sensor ();
        }
        else
        {
            // We need the 4 color measured
            ++cur_color;
            rgb.setup_color (cur_color);
        }
        measure_cnt = 0;
    }
}
