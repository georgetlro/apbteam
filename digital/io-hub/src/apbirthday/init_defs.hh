#ifndef init_defs_hh
#define init_defs_hh
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

/// Parameters to push the first wall.
#define INIT_FIRST_WALL_PUSH \
    Asserv::FORWARD, pg_x (650), pg_y (pg_length - BOT_FRONT_CONTACT_DIST), \
    pg_a_deg (90 + BOT_BACK_CONTACT_ANGLE_ERROR_DEG)
/// Parameters to go away from the first wall.
#define INIT_FIRST_WALL_AWAY (-(600 - BOT_FRONT_CONTACT_DIST))
/// Parameter to face the second wall.
#define INIT_SECOND_WALL_ANGLE pg_a_deg (180)
/// Parameters to push the second wall.
#define INIT_SECOND_WALL_PUSH \
    Asserv::FORWARD, pg_x (BOT_FRONT_CONTACT_DIST), -1, -1
/// Start match next to the wall.
#define INIT_START_SECOND_WALL 1
/// Parameters to go away from the second wall.
#define INIT_SECOND_WALL_AWAY (-(200 - BOT_FRONT_CONTACT_DIST))

#endif // init_defs_hh
