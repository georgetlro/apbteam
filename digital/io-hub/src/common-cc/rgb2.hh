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
        Rgb ();
        /* Enable all the RGB timer, and setup the Input Capture block */
        static void enable (void);
        /* Setup the filter on the sensor */
        static void setup_color (int color);
        static int calibrate_sensor (void);
        /* Get a color results from a sensor */
        static int get_color (int sensor, int offset);
        /* Input Capture interrupt function */
        static void ic_isr ();

        /* GPIO Manipulation */
        static void enable_sensor (int sensor);
        static void disable_sensor (int sensor);
        /* Do one measurement */
        static void do_measure (int sensor);
        /* There is different type to grab a color for each RGB */
        static int get_cannon_color (int offset);
        static int get_cherry_color (int sensor);
        static int rgb_to_hue (int r, int g, int b);

        enum color
        {
            WHITE,
            RED,
            GREEN,
            BLUE,
            UNKNOWN,
            NOTHING
        };
        enum sensor
        {
            CANNON,
            CHERRY_FAR,
            CHERRY_NEAR,
            NONE
        };
};

#endif // rgb_hh
