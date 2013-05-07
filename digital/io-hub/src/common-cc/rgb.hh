#ifndef rgb_hh
#define rgb_hh
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
        Rgb (int sensor_);
        /* Enable all the RGB timer, and setup the Input Capture block */
        static void enable (void);
        /* Setup the filter on the sensor */
        void setup_color (int color);
        int calibrate_sensor (void);
        /* Get a color results from a sensor */
        int get_color (int offset);
        /* Input Capture interrupt function */
        static void ic_isr ();

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
    private:
        ucoo::Gpio EN;
        ucoo::Gpio S2;
        ucoo::Gpio S3;
        /* Keep the sensor type */
        int sensor;
        /* Store results for each color here */
        int color_value[4];
        /* Measure counter */
        int measure_cnt;
        /* Current color being measured */
        int cur_color;

        /* GPIO Manipulation */
        void enable_sensor (void);
        void disable_sensor (void);
        /* Do one measurement */
        void do_measure (void);
        /* There is different type to grab a color for each RGB */
        int get_cannon_color (int offset);
        int get_cherry_color (void);
};

#endif // rgb_hh
