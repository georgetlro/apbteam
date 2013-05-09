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
#include "rgb.hh"
#include "robot.hh"

Rgb::Rgb (ucoo::Gpio *en_cannon,
          ucoo::Gpio *en_candle_far,
          ucoo::Gpio *en_candle_near)
{
}

void
Rgb::enable ()
{
}

void
Rgb::update ()
{
}

void
Rgb::calibrate_cannon_sensor ()
{
}

void
Rgb::start_cannon_color ()
{
}

void
Rgb::stop_cannon_color ()
{
}

enum Rgb::color
Rgb::get_candle_far_color ()
{
    return robot->rgb.get_candle_color ();
}

enum Rgb::color
Rgb::get_candle_near_color ()
{
    return robot->rgb.get_candle_color ();
}

enum Rgb::color
Rgb::get_candle_color ()
{
    return WHITE;
}

void
Rgb::get_colors (rgb_type type, uint16_t &w, uint16_t &r, uint16_t &g,
                 uint16_t &b)
{
    w = r = g = b = 0;
}

