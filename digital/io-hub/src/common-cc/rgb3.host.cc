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
#include "rgb3.hh"

Rgb::Rgb (ucoo::Gpio *EN)
{
}

void
Rgb::enable ()
{
}

void
Rgb::setup_color (int color)
{
}

uint32_t
Rgb::calibrate_sensor ()
{
    return 0;
}

void
Rgb::enable_sensor ()
{
}

void
Rgb::disable_sensor ()
{
}

void
Rgb::do_measure ()
{
}

uint32_t
Rgb::get_cannon_color (uint32_t offset)
{
    return UNKNOWN;
}

uint32_t
Rgb::get_candle_color ()
{
    return UNKNOWN;
}

int
Rgb::rgb_to_hue (int r, int g, int b){
    return 0;
}

void
Rgb::ic_isr ()
{
}
