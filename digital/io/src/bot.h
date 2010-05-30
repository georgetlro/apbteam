#ifndef bot_h
#define bot_h
/* bot.h */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
 *
 * Copyright (C) 2010 Nicolas Schodet
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
 * Specific defines about the robot dimensions and competion rules.
 */

/**
 * Duration of a match in milliseconds, with margin.
 */
#define MATCH_DURATION_MS (90000 - 2500)

/**
 * The scaling factor, millimeter per step.
 */
#ifdef HOST
# define BOT_SCALE 0.0395840674352314
#else
# define BOT_SCALE 0.0415178942124
#endif

/** Distance from the robot axis to the front. */
#define BOT_SIZE_FRONT 120
/** Distance from the robot axis to the front clamp. */
#define BOT_SIZE_FRONT_CLAMP (120 + 100)
/** Distance from the robot axis to the back. */
#define BOT_SIZE_BACK (280 - 120)
/** Distance from the robot axis to the side. */
#define BOT_SIZE_SIDE (310 / 2)
/** Distance from the robot axis to farthest corner, this is the radius of the
 * circle needed when rotating. */
#define BOT_SIZE_RADIUS 196

/** Start position. */
#define BOT_INIT_XYA \
    PG_X (PG_START_ZONE_WIDTH - BOT_SIZE_RADIUS), \
    PG_Y (PG_LENGTH - PG_START_ZONE_LENGTH + BOT_SIZE_RADIUS), \
    PG_A_DEG (-76)

/**
 * Distance required to be away from a border to be able to turn freely.
 * In millimeter.
 */
#define BOT_MIN_DISTANCE_TURN_FREE 300

/** Fast parameters. */
#define BOT_MOVE_FAST 0x20, 0x20, 0x10, 0x10

/** Medium parameters. */
#define BOT_MOVE_MEDIUM 0x30, 0x30, 0x18, 0x18

/** Slow parameters. */
#define BOT_MOVE_SLOW 0x20, 0x20, 0x10, 0x10

/** Very slow parameters. */
#define BOT_MOVE_SNAYLE 0x8, 0x8, 0x8, 0x8

/** Elevator stroke in steps. */
#define BOT_ELEVATOR_STROKE_STEP 7089

/** Elevator unloading up position in steps. */
#define BOT_ELEVATOR_UNLOAD_STEP 5600

/** Elevator resting up position in steps. */
#define BOT_ELEVATOR_REST_STEP 4111

/** Elevator position under which a blocking is considered as due to a black
 * corn in steps. */
#define BOT_ELEVATOR_BLACK_THRESHOLD_STEP 3000

/** Elevator down position in steps. */
#define BOT_ELEVATOR_DOWN_STEP 160

/** Elevator work speed. */
#define BOT_ELEVATOR_SPEED 0x60

/** Elevator find zero speed. */
#define BOT_ELEVATOR_ZERO_SPEED 0x10

/** Clamp stroke in steps. */
#define BOT_CLAMP_STROKE_STEP 2813

/** Clamp open position in steps. */
#define BOT_CLAMP_OPEN_STEP 60

/** Clamp width in steps. */
#define BOT_CLAMP_WIDTH_STEP 4604

/** Distance under which the clamp is considered empty. */
#define BOT_CLAMP_EMPTY_STEP 639

/** Clamp work speed. */
#define BOT_CLAMP_SPEED 0x60

/** Clamp find zero speed. */
#define BOT_CLAMP_ZERO_SPEED 0x10

/** Clamp open loop PWM. */
#define BOT_CLAMP_PWM 0x100

/** Gate work speed. */
#define BOT_GATE_SPEED 0x46

/** Gate stroke in steps. */
#define BOT_GATE_STROKE_STEP -0x1d6b

/**
 * Definition of the colors.
 */
enum team_color_e
{
    RED_TEAM = 0,
    BLUE_TEAM = 1
};

/**
 * Our color.
 */
extern enum team_color_e bot_color;

#endif /* bot_h */
