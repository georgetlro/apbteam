// path_2013.cc
// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Olivier Lanneluc
//
// APBTeam:
//      Web: http://apbteam.org/
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

#include "path_2013.hh"
#include "bot.hh"
#ifdef HOST
#include "debug.host.hh"
#endif

Path_2013::Path_2013() : Path( (pg_border_distance + pg_plate_size_border*2 + BOT_SIZE_RADIUS/2),
                               (pg_border_distance),
                               (pg_width - pg_border_distance - pg_plate_size_border*2 - BOT_SIZE_RADIUS/2),
                               (pg_length - pg_border_distance))
{
    host_debug("Path_2013 constructor\n");
    obstacles = obstacles_2013;
    navpoints = navpoints_2013;
    navweights = navweights_2013;
    astar_nodes = astar_nodes_2013;
}

void Path_2013::reset()
{
    /* Reset the mobile obstacles */
    Path::reset();

    /* Add the cake obstacle */
    add_obstacle( pg_cake_pos,
                  pg_cake_radius,
                  PATH_2013_CAKE_NAVPOINTS_NB * 2 /* only half the navpoints are on the playground */,
                  PATH_2013_CAKE_NAVPOINTS_LAYERS,
                  0 /* no extra clearance radius */,
                  true /* it is allowed to target the center */);
}
