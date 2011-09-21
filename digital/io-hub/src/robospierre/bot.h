#ifndef bot_h
#define bot_h
/* bot.h */
/* robospierre - Eurobot 2011 AI. {{{
 *
 * Copyright (C) 2011 Nicolas Schodet
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

/** Robot specific defines. */

/** Scaling factor, millimeter per step. */
#ifdef HOST
# define BOT_SCALE 0.0395840674352314
#else
# define BOT_SCALE 0.0317975134344
#endif

/** Distance from the robot axis to the front. */
#define BOT_SIZE_FRONT 150
/** Distance from the robot axis to the back. */
#define BOT_SIZE_BACK 150
/** Distance from the robot axis to the side. */
#define BOT_SIZE_SIDE 190

/** Radius of an element. */
#define BOT_ELEMENT_RADIUS 100

/** Distance between the front contact point and the robot center. */
#define BOT_FRONT_CONTACT_DIST_MM 150
/** Angle error at the front contact point. */
#define BOT_FRONT_CONTACT_ANGLE_ERROR_DEG 0

/** Distance from robot center to front pawn detection threshold. */
#define BOT_PAWN_FRONT_DETECTION_THRESHOLD_MM 190
/** Distance from robot center to back pawn detection threshold. */
#define BOT_PAWN_BACK_DETECTION_THRESHOLD_MM -190
/** Distance from robot center to an element near enough to be taken. */
#define BOT_PAWN_TAKING_DISTANCE_MM 150

/** Distance from border to position in front of a green element. */
#define BOT_GREEN_ELEMENT_PLACE_DISTANCE_MM 600
/** Distance from border to go to capture a green element. */
#define BOT_GREEN_ELEMENT_DISTANCE_MM \
    (BOT_ELEMENT_RADIUS + BOT_SIZE_FRONT + 1)

/** Speed used for initialisation. */
#ifdef HOST
# define BOT_SPEED_INIT 0x20, 0x20, 0x20, 0x20
#else
# define BOT_SPEED_INIT 0x10, 0x10, 0x10, 0x10
#endif
/** Normal cruise speed. */
#define BOT_SPEED_NORMAL 0x50, 0x60, 0x20, 0x20

#ifdef HOST

# define BOT_CLAMP_SLOT_FRONT_BOTTOM_ELEVATION_STEP 0
# define BOT_CLAMP_SLOT_FRONT_MIDDLE_ELEVATION_STEP (0x3b0b / 2)
# define BOT_CLAMP_SLOT_FRONT_TOP_ELEVATION_STEP 0x3b0b
# define BOT_CLAMP_SLOT_BACK_BOTTOM_ELEVATION_STEP 0
# define BOT_CLAMP_SLOT_BACK_MIDDLE_ELEVATION_STEP (0x3b0b / 2)
# define BOT_CLAMP_SLOT_BACK_TOP_ELEVATION_STEP 0x3b0b
# define BOT_CLAMP_SLOT_SIDE_ELEVATION_STEP 0x3b0b
# define BOT_CLAMP_BAY_FRONT_LEAVE_ELEVATION_STEP (0x3b0b / 2 + 1000)
# define BOT_CLAMP_BAY_BACK_LEAVE_ELEVATION_STEP (0x3b0b / 2 + 1000)
# define BOT_CLAMP_BAY_SIDE_ENTER_LEAVE_ELEVATION_STEP (0x3b0b / 2)
#define BOT_CLAMP_INIT_ELEVATION_SWITCH_STEP \
    BOT_CLAMP_SLOT_FRONT_TOP_ELEVATION_STEP

# define BOT_CLAMP_SLOT_FRONT_BOTTOM_ROTATION_STEP 0
# define BOT_CLAMP_SLOT_FRONT_MIDDLE_ROTATION_STEP 0
# define BOT_CLAMP_SLOT_FRONT_TOP_ROTATION_STEP 0
# define BOT_CLAMP_SLOT_BACK_BOTTOM_ROTATION_STEP 0x233e
# define BOT_CLAMP_SLOT_BACK_MIDDLE_ROTATION_STEP 0x233e
# define BOT_CLAMP_SLOT_BACK_TOP_ROTATION_STEP 0x233e

# define BOT_CLAMP_BAY_FRONT_ROTATION_STEP \
    BOT_CLAMP_SLOT_FRONT_MIDDLE_ROTATION_STEP
# define BOT_CLAMP_BAY_BACK_ROTATION_STEP \
    BOT_CLAMP_SLOT_BACK_MIDDLE_ROTATION_STEP
# define BOT_CLAMP_BAY_SIDE_ROTATION_STEP \
    (BOT_CLAMP_BAY_BACK_ROTATION_STEP / 2)
# define BOT_CLAMP_BAY_SIDE_MARGIN_ROTATION_STEP 1000

#define BOT_CLAMP_CLOSED_FRONT_ROTATION_OFFSET 0
#define BOT_CLAMP_CLOSED_BACK_ROTATION_OFFSET 0
#define BOT_CLAMP_CLOSED_SIDE_ROTATION_OFFSET 0

#else /* !HOST */

# define BOT_CLAMP_SLOT_FRONT_BOTTOM_ELEVATION_STEP 0
# define BOT_CLAMP_SLOT_FRONT_MIDDLE_ELEVATION_STEP (0x1da7 - 250)
# define BOT_CLAMP_SLOT_FRONT_TOP_ELEVATION_STEP 0x35e2
# define BOT_CLAMP_SLOT_BACK_BOTTOM_ELEVATION_STEP 0x0169
# define BOT_CLAMP_SLOT_BACK_MIDDLE_ELEVATION_STEP (0x1f03 - 250)
# define BOT_CLAMP_SLOT_BACK_TOP_ELEVATION_STEP 0x3610
# define BOT_CLAMP_SLOT_SIDE_ELEVATION_STEP (0x3596 + 3 * 0x72)
# define BOT_CLAMP_BAY_FRONT_LEAVE_ELEVATION_STEP 0x1da7
# define BOT_CLAMP_BAY_BACK_LEAVE_ELEVATION_STEP 0x1f03
# define BOT_CLAMP_BAY_SIDE_ENTER_LEAVE_ELEVATION_STEP ((0x1da7 + 0x1f03) / 2)
#define BOT_CLAMP_INIT_ELEVATION_SWITCH_STEP 0x363f

# define BOT_CLAMP_SLOT_FRONT_BOTTOM_ROTATION_STEP 92
# define BOT_CLAMP_SLOT_FRONT_MIDDLE_ROTATION_STEP 92
# define BOT_CLAMP_SLOT_FRONT_TOP_ROTATION_STEP 92
# define BOT_CLAMP_SLOT_BACK_BOTTOM_ROTATION_STEP 0x2433
# define BOT_CLAMP_SLOT_BACK_MIDDLE_ROTATION_STEP 0x2433
# define BOT_CLAMP_SLOT_BACK_TOP_ROTATION_STEP 0x2433

# define BOT_CLAMP_BAY_FRONT_ROTATION_STEP \
    BOT_CLAMP_SLOT_FRONT_MIDDLE_ROTATION_STEP
# define BOT_CLAMP_BAY_BACK_ROTATION_STEP \
    BOT_CLAMP_SLOT_BACK_MIDDLE_ROTATION_STEP
# define BOT_CLAMP_BAY_SIDE_ROTATION_STEP (0x1183 + 120)
# define BOT_CLAMP_BAY_SIDE_MARGIN_ROTATION_STEP 1000

#define BOT_CLAMP_CLOSED_FRONT_ROTATION_OFFSET -129
#define BOT_CLAMP_CLOSED_BACK_ROTATION_OFFSET -60
#define BOT_CLAMP_CLOSED_SIDE_ROTATION_OFFSET -120

#endif /* !HOST */

#define BOT_CLAMP_INIT_ELEVATION_STEP \
    (BOT_CLAMP_SLOT_FRONT_MIDDLE_ELEVATION_STEP + 10 * 0x72)

#define BOT_CLAMP_CLOSED_ROTATION_OFFSET(pos) \
    (CLAMP_IS_SLOT_IN_FRONT_BAY (pos) \
     ? BOT_CLAMP_CLOSED_FRONT_ROTATION_OFFSET \
     : (CLAMP_IS_SLOT_IN_BACK_BAY (pos) \
	? BOT_CLAMP_CLOSED_BACK_ROTATION_OFFSET \
	: BOT_CLAMP_CLOSED_SIDE_ROTATION_OFFSET))

#define BOT_CLAMP_INIT_ELEVATION_SPEED 0x10
#define BOT_CLAMP_INIT_ROTATION_SPEED -0x04
#define BOT_CLAMP_ELEVATION_SPEED 0x60
#define BOT_CLAMP_ROTATION_SPEED 0x60
#define BOT_CLAMP_ROTATION_OFFSET_SPEED 1

#define BOT_PWM_CLAMP 2
#define BOT_PWM_DOOR_FRONT_BOTTOM 0
#define BOT_PWM_DOOR_FRONT_TOP 1
#define BOT_PWM_DOOR_BACK_BOTTOM 3
#define BOT_PWM_DOOR_BACK_TOP 4

#define BOT_PWM_CLAMP_OPEN_TIME 150
#define BOT_PWM_CLAMP_OPEN 0x1ff, 150, 0
#define BOT_PWM_CLAMP_CLOSE_TIME 150
#define BOT_PWM_CLAMP_CLOSE -0x1ff, 150, 0

#define BOT_PWM_DOOR_OPEN_TIME 12
#define BOT_PWM_DOOR_OPEN(slot) \
    0x1ff, (((slot) == CLAMP_SLOT_FRONT_BOTTOM \
	     || (slot) == CLAMP_SLOT_BACK_BOTTOM) ? 80 : 62), 0x55
#define BOT_PWM_DOOR_CLOSE_TIME 100
#define BOT_PWM_DOOR_CLOSE(slot) \
    -0x1ff, (((slot) == CLAMP_SLOT_FRONT_BOTTOM \
	     || (slot) == CLAMP_SLOT_BACK_BOTTOM) ? 80 : 62), \
    (((slot) == CLAMP_SLOT_FRONT_BOTTOM \
      || (slot) == CLAMP_SLOT_BACK_BOTTOM) ? -0x100 : -0x200)

#define BOT_PWM_CLAMP_INIT 0x1ff, 150, 0
#define BOT_PWM_DOOR_INIT 0x100, 160, 0
#define BOT_PWM_DOOR_INIT_START 0x55
#define BOT_PWM_CLAMP_DOOR_INIT 250

#endif /* bot_h */
