#ifndef rgb2_hh
#define rgb2_hh
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

#include "ucoolib/hal/gpio/gpio.hh"

/* This value was determined during tests using a BLUE filter */
/* This will only be used for the cannon method */
#define BASIC_GREY 190

class Rgb
{
    public:
        Rgb (ucoo::Gpio *EN);
        /* Enable the RGB timer, and setup the Input Capture block */
        void enable ();
        /* setup a filter to measure one color, this is for all RGB */
        void setup_color (int color);
        uint32_t calibrate_sensor ();
        uint32_t get_cannon_color (uint32_t offset);
        uint32_t get_candle_color ();
        static void ic_isr ();
        static int measure_cnt;
        static int cur_color;
        static uint32_t last_color_value;
        static uint32_t color_value[4];

    private:
        void enable_sensor ();
        void disable_sensor ();
        void do_measure ();
        int rgb_to_hue (int r, int g, int b);

        /* class attributes */
        ucoo::Gpio *EN_;
        int type;

        enum color
        {
            WHITE,
            RED,
            GREEN,
            BLUE,
            UNKNOWN,
            NOTHING
        };
        enum rgb_type
        {
            CANNON,
            CANDLE
        };
};

#endif // rgb_hh
