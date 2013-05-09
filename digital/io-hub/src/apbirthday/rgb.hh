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
        Rgb (ucoo::Gpio *en_cannon,
             ucoo::Gpio *en_candle_far,
             ucoo::Gpio *en_candle_near);

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
            CANDLE_NEAR,
            CANDLE_FAR,
            RGB_TYPE_NB
        };

        /* Enable the RGB timer, and setup the Input Capture block */
        void enable ();
        void update ();
        void calibrate_cannon_sensor ();
        void start_cannon_color ();
        void stop_cannon_color ();
        enum color get_candle_color ();
        enum color get_candle_near_color ();
        enum color get_candle_far_color ();
        void get_colors (rgb_type type, uint16_t &w, uint16_t &r, uint16_t &g,
                         uint16_t &b);

        void ic_isr ();

    private:
        /* setup a filter to measure one color, this is for all RGB */
        void setup_color (int color);
        void enable_sensor ();
        void disable_sensor ();
        void do_measure (rgb_type type, bool router);
        int rgb_to_hue (int r, int g, int b);

        ucoo::Gpio* g[RGB_TYPE_NB];
        enum rgb_type type_;
        uint32_t cannon_offset_;
        bool router_;
        bool seen_color_, seen_white_;
        int measure_cnt_;
        int cur_color_;
        int router_timer_;
        uint32_t last_color_value_;
        uint32_t color_value_[4];
};

#endif // rgb_hh
