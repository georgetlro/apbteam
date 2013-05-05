#ifndef playground_2013_hh
#define playground_2013_hh
// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Nicolas Schodet
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
#include "playground.hh"

/// Largest radius of the cake.
static const int pg_cake_radius = 500;

/// Cake center X and Y.
static const vect_t pg_cake_pos = { pg_width / 2, pg_length };

/// Distance from the cake to blow candles.
static const int pg_cake_distance = 40;

/// Plate width / 2.
static const int pg_plate_size_border = 170 / 2;

/// Distance from the border to open gifts.
static const int pg_gifts_distance = 80;

/// Width of a gift.
static const int pg_gift_width = 176;

#endif // playground_2013_hh
