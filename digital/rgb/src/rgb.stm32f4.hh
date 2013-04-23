#pragma once

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
#include "ucoolib/hal/usb/usb.hh"
#include "ucoolib/utils/delay.hh"
#include "ucoolib/common.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <cstdio>

static enum colors{
    WHITE = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    UNKNOWN = 4
} color;
const char * clr[] = { "WHITE", "RED", "GREEN", "BLUE", "UNK"};

static int measure_cnt = 0;
static int cur_color = WHITE;
static float results[4] = {0, 0, 0, 0};
int setup_measure ();
void tim2_isr(void);
void sensor_setup(int enable);
void setup_color(int new_color);
void setup_input_capture();


