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

Rgb::Rgb ()
{
}

void
Rgb::enable (void)
{
}

void
Rgb::setup_color (int color)
{
}

int
Rgb::calibrate_sensor (void)
{
    return 0;
}

int
Rgb::get_color (int sensor, int offset)
{
    if (sensor == CANNON)
    {
        return get_cannon_color (offset);
    }
    else
    {
        return get_cherry_color (sensor);
    }
    return 0;
}

void
Rgb::enable_sensor (int sensor)
{
}

void
Rgb::disable_sensor (int sensor)
{
}

void
Rgb::do_measure (int sensor)
{
}

int
Rgb::get_cannon_color (int offset)
{
    return UNKNOWN;
}

int
Rgb::get_cherry_color (int sensor)
{
    return UNKNOWN;
}

int
Rgb::rgb_to_hue (int r, int g, int b){
    return 0;
}

void
Rgb::ic_isr (void)
{
}
