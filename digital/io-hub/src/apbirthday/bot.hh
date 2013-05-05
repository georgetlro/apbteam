#ifndef bot_hh
#define bot_hh
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

/// Robot specific definitions.

/// Scaling factor, millimeter per step.
#ifdef HOST
# define BOT_SCALE 0.0395840674352314
#else
# define BOT_SCALE 0.0317975134344
#endif

/// Distance from the robot axis to the front.
#define BOT_SIZE_FRONT 102
/// Distance from the robot axis to the back.
#define BOT_SIZE_BACK 108
/// Distance from the robot axis to the side.
#define BOT_SIZE_SIDE 140
/// Maximum distance from the robot base center to one of its edge.
#define BOT_SIZE_RADIUS 177

/// Distance between the front contact point and the robot center.
#define BOT_FRONT_CONTACT_DIST BOT_SIZE_FRONT
/// Angle error at the front contact point.
#define BOT_BACK_CONTACT_ANGLE_ERROR_DEG 0

/// Speed used for initialisation.
#ifdef HOST
# define BOT_SPEED_INIT 0x20, 0x20, 0x20, 0x20
#else
# define BOT_SPEED_INIT 0x10, 0x10, 0x10, 0x10
#endif
/// Normal cruise speed.
#define BOT_SPEED_NORMAL 0x50, 0x60, 0x20, 0x20
/// Speed used for plate loading.
#define BOT_SPEED_PLATE 0x20, 0x20, 0x20, 0x20
/// Speed used for gifts.
#define BOT_SPEED_GIFTS 0x28, 0x30, 0x20, 0x20

/// Normal pneumatic pressure.
#define BOT_NORMAL_PRESSURE 0x0c00

/// X position of the gift arm when extended.
static const int bot_gift_arm_x = -100;

#endif // bot_hh
