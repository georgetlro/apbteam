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

int
Rgb::rgb_to_hue (int r, int g, int b)
{
    uint8_t min = 255, max = 0, hue;

    // find minimal between r, g and b
    if (r < min)
        min = r;
    if (g < min)
        min = g;
    if (b < min)
        min = b;

    // find maximal between r, g and b
    if (r > max)
        max = r;
    if (g > max)
        max = g;
    if (b > max)
        max = b;

    if (min == max)
        return 0;

    /* Compute hue */
    if (max == r)
        hue = 0 + 43 * (g - b) / (max - min);
    else if (max == g)
        hue = 85 + 43 * (b - r) / (max - min);
    else /* max == b */
        hue = 171 + 43 * (r - g) / (max - min);
    return hue;
}
