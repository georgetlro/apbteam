#ifndef aquajim_h
#define aquajim_h
/* aquajim.h */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
 *
 * Copyright (C) 2009 Dufour Jérémy
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

/**
 * Some defines for the eurobot 2009 bot of APBTeam: Aqua Jim.
 */

/**
 * Duration of a match in milliseconds.
 */
#define MATCH_DURATION_MS 90000

/**
 * filterbridge servo
 */

#define SERVO_FINGER_ID 0
#define SERVO_DOOR_ID 1
#define SERVO_FINGER_IDLE 0
#define SERVO_FINGER_PUSHING 1
#define SERVO_DOOR_OPEN 0
#define SERVO_DOOR_CLOSE 1

/**
 * How to compute a angle for giboulee?
 * One degree is 65536 / 360
 */
#define BOT_ANGLE_DEGREE (65536 / 360)

/**
 * The size of the bot.
 */
#define BOT_LENGTH 300
#define BOT_WIDTH 310

#endif /* aquajim_h */
