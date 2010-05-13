/* top.c */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
 *
 * Copyright (C) 2009 Nicolas Haller
 *
 * APBTeam:
 *        Web: http://apbteam.org/
 *      Email: team AT apbteam DOT org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * }}} */
#include "common.h"
#include "defs.h"
#include "top.h"
#include "food.h"
#include "asserv.h"
#include "loader.h"
#include "move.h"
#include "playground.h"

void
top_init (void)
{
}

uint8_t
top_collect (uint8_t force)
{
    static uint8_t max = 3;
    if (loader_elements < max || force)
      {
	position_t robot_position;
	asserv_get_position (&robot_position);
	uint8_t food = food_best (robot_position);
	if (food == 0xff)
	    return 0;
	vect_t food_v;
	food_pos (food, &food_v);
	move_start_noangle (food_v, 0);
	max--;
	if (max == 0) max = 1;
	return 1;
      }
    else
      {
	move_start_noangle (PG_VECT (2625, 253), 0);
	return 0;
      }
}

