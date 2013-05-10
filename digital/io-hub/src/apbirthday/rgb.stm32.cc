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
#include "robot.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// The interupt fonction will call one in our class.
extern "C"
{
    void tim1_cc_isr () { robot->rgb.ic_isr (); }
}

Rgb::Rgb (ucoo::Gpio *en_cannon,
          ucoo::Gpio *en_candle_far,
          ucoo::Gpio *en_candle_near)
{
    g[CANNON] = en_cannon;
    g[CANNON]->output ();
    g[CANDLE_NEAR] = en_candle_near;
    g[CANDLE_NEAR]->output ();
    g[CANDLE_FAR] = en_candle_far;
    g[CANDLE_FAR]->output ();
    router_timer_ = 0;
}

void
Rgb::enable ()
{
    // Disable the sensor to be sure
    int c;
    for (c = (int) CANNON; c < (int) RGB_TYPE_NB; c++)
    {
        type_ = (enum Rgb::rgb_type) c;
        disable_sensor ();
    }

    // Enable clock for Timer 1.
    rcc_peripheral_enable_clock (&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

    /* Output GPIO */
    gpio_mode_setup (GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af (GPIOE, GPIO_AF1, GPIO9);

    nvic_enable_irq (NVIC_TIM1_CC_IRQ);

    timer_reset (TIM1);
    timer_set_mode (TIM1,
                    TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
                    TIM_CR1_CMS_EDGE,   // Edge synchronization
                    TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler (TIM1, 75);
    timer_set_period (TIM1, 0xFFFF);
    timer_set_repetition_counter (TIM1, 0);
    timer_continuous_mode (TIM1);

    /* Configure Input Capture Channel 1 */
    timer_ic_set_input (TIM1, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter (TIM1, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity (TIM1, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_prescaler (TIM1, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_enable (TIM1, TIM_IC1);
    timer_clear_flag (TIM1, TIM_SR_CC1IF);
    timer_disable_irq (TIM1, TIM_DIER_CC1IE);

    timer_enable_counter (TIM1);
}

void
Rgb::setup_color (int color)
{
    ucoo::Gpio S2 (GPIOE, 8);
    ucoo::Gpio S3 (GPIOE, 11);
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

void
Rgb::calibrate_cannon_sensor ()
{
    // Do a measure to get current light situation
    do_measure (CANNON, false);
    cannon_ref_grey_ = color_value_[BLUE];
}

void
Rgb::enable_sensor ()
{
    g[type_]->set (0);
    timer_clear_flag (TIM1, TIM_SR_CC1IF);
    timer_enable_irq (TIM1, TIM_DIER_CC1IE);
}

void
Rgb::disable_sensor ()
{
    g[type_]->set (1);
    timer_disable_irq (TIM1, TIM_DIER_CC1IE);
}

void
Rgb::do_measure (rgb_type type, bool router)
{
    type_ = type;
    router_ = router;
    measure_cnt_ = 0;
    if (router)
        cur_color_ = BLUE;
    else
        cur_color_ = WHITE;
    setup_color (cur_color_);
    seen_color_ = seen_white_ = false;
    enable_sensor ();
    // The interupt will disable the sensor
    while (!router && g[type]->get() == 0)
        ;
    // Grab the results in... color_value_
}

void
Rgb::start_cannon_color ()
{
    do_measure (CANNON, true);
}

void
Rgb::stop_cannon_color ()
{
    disable_sensor ();
}

void
Rgb::update ()
{
    if (router_timer_)
    {
        --router_timer_;
        if (router_timer_ == 0)
        {
            robot->hardware.cherry_bad_out.set (false);
        }
    }
}

enum Rgb::color
Rgb::get_candle_far_color ()
{
    do_measure (CANDLE_FAR, false);
    return Rgb::get_candle_color ();
}

enum Rgb::color
Rgb::get_candle_near_color ()
{
    do_measure (CANDLE_NEAR, false);
    return Rgb::get_candle_color ();
}

enum Rgb::color
Rgb::get_candle_color ()
{
    color_value_[BLUE] *= 0.8;
    color_value_[RED] *= 1.2;
    if (color_value_[BLUE] < color_value_[RED])
        return BLUE;
    else
        return RED;
}

void
Rgb::get_colors (rgb_type type, uint16_t &w, uint16_t &r, uint16_t &g,
                 uint16_t &b)
{
    do_measure (type, false);
    w = color_value_[WHITE];
    r = color_value_[RED];
    g = color_value_[GREEN];
    b = color_value_[BLUE];
}

void
Rgb::ic_isr ()
{
    uint16_t cc = TIM1_CCR1;

    // Throw away the first measurement
    if (measure_cnt_ == 0)
    {
        measure_cnt_++;
    }
    else if (measure_cnt_ == 1)
    {
        last_color_value_ = cc;
        measure_cnt_++;
    }
    else if (measure_cnt_ == 2)
    {
        color_value_[cur_color_] = (cc - last_color_value_) & 0xffff;

        if (router_)
        {
            uint16_t v = (color_value_[BLUE] * 100) / cannon_ref_grey_;

            if (v > 0 && v < 30)
            {
                seen_white_ = true;
            }
            else if (v > 30 && v < 90)
            {
                // we don't know if it's blue or red, and we don't care
                // It's colored and must be thrown out !
                seen_color_ = true;
            }
            // Check if we are seeing GREY again
            else if (v > 90)
            {
                // If we have seen color
                if (seen_color_ && !seen_white_)
                {
                    robot->hardware.cherry_bad_out.set (true);
                    router_timer_ = 18;
                }
                seen_color_ = seen_white_ = false;
            }
        }
        else
        {
            cur_color_++;
            if (cur_color_ == UNKNOWN)
                disable_sensor ();
            else
                setup_color (cur_color_);
        }

        measure_cnt_ = 0;
    }
}
