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

class RGB
{
    public:
        // Need to handle a group of RGB here, each function take an
        // index to specify which RGB to do stuff on.
        RGB (ucoo::Gpio *EN_);
        ucoo::Gpio *EN;

        void start ();
        int get ();
        // Ask us if the value is ready or not.
        bool poke ();
        enum colors{
            WHITE = 0,
            RED = 1,
            GREEN = 2,
            BLUE = 3,
            UNKNOWN = 4,
            NOT_READY = 5,
            NB = 6
        };
    private:
        ucoo::Gpio S2;
        ucoo::Gpio S3;
        bool value_ready;
        int min_color, color;
        float results[4];
        int cur_color;
        int measure_cnt;
        void sensor_setup (int enable);
        void setup_color (int new_color);
        void setup_input_capture ();
        #ifndef TARGET_host
            void tim1_isr ();
        #endif
};
