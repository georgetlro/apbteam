/* clamp.c */
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
#include "common.h"
#include "clamp.h"

#include "mimot.h"
#include "pwm.h"
#include "contact.h"
#include "bot.h"
#include "element.h"
#include "playground.h"

#define FSM_NAME AI
#include "fsm.h"
#include "fsm_queue.h"

#include "logistic.h"
#include "pawn_sensor.h"

/*
 * There is two FSM in this file.
 *
 * The clamp FSM handles high level clamp behaviour, new elements, drop, and
 * gives orders to the clamp move FSM.
 *
 * The clamp move FSM only handle moving the clamp without load or moving an
 * element from a slot to another one.
 */

FSM_STATES (
	    /* Initial state. */
	    CLAMP_START,
	    /* Initialisation sequence: opening everything. */
	    CLAMP_INIT_OPENING,
	    /* Initialisation sequence: going to the middle level. */
	    CLAMP_INIT_GOING_MIDDLE,
	    /* Initialisation sequence: finding front right edge. */
	    CLAMP_INIT_FINDING_ROTATION_EDGE,
	    /* Initialisation sequence: finding top switch. */
	    CLAMP_INIT_FINDING_TOP,
	    /* Initialisation sequence: going to rest position. */
	    CLAMP_INIT_GOING_REST,
	    /* Clamp ready, waiting in rest position. */
	    CLAMP_INIT_READY,

	    /* Returning to idle position. */
	    CLAMP_GOING_IDLE,
	    /* Waiting external events, clamp at middle level. */
	    CLAMP_IDLE,
	    /* Taking an element at bottom slots. */
	    CLAMP_TAKING_DOOR_CLOSING,
	    /* Moving elements around. */
	    CLAMP_MOVING_ELEMENT,
	    /* Droping a tower. */
	    CLAMP_DROPING_DOOR_OPENING,
	    /* Droping a tower, waiting for robot to advance. */
	    CLAMP_DROPING_WAITING_ROBOT,
	    /* Clamp locked in a bay. */
	    CLAMP_LOCKED,
	    /* Clamp blocked. */
	    CLAMP_BLOCKED,

	    /* Waiting movement order. */
	    CLAMP_MOVE_IDLE,
	    /* Moving to a final or intermediary position. */
	    CLAMP_MOVE_ROUTING,
	    /* Moving to source slot. */
	    CLAMP_MOVE_SRC_ROUTING,
	    /* Closing the clamp once arrived at source. */
	    CLAMP_MOVE_SRC_CLAMP_CLOSING,
	    /* Opening door once clamp closed. */
	    CLAMP_MOVE_SRC_DOOR_OPENDING,
	    /* Moving to destination slot. */
	    CLAMP_MOVE_DST_ROUTING,
	    /* Closing door once arrived at destination. */
	    CLAMP_MOVE_DST_DOOR_CLOSING,
	    /* Opening the clamp once door closed. */
	    CLAMP_MOVE_DST_CLAMP_OPENING)

FSM_EVENTS (
	    /* New element inside bottom slot. */
	    clamp_new_element,
	    /* Order to prepare tower. */
	    clamp_prepare,
	    /* Sent when an element has just been taken (door closed). */
	    clamp_taken,
	    /* Sent when clamp is working. */
	    clamp_working,
	    /* Sent when clamp return to idle state. */
	    clamp_done,
	    /* Sent when clamp is blocked. */
	    clamp_blocked,
	    /* Order to drop elements. */
	    clamp_drop,
	    /* Sent once drop is done, but robot should advance to completely
	     * free the dropped tower. */
	    clamp_drop_waiting,
	    /* Received when top FSM made the robot advance after a drop. */
	    clamp_drop_clear,
	    /* Order to move the clamp. */
	    clamp_move,
	    /* Clamp movement success. */
	    clamp_move_success,
	    /* Clamp movement failure. */
	    clamp_move_failure,
	    /* Elevation and elevation motor success. */
	    clamp_elevation_rotation_success,
	    /* Elevation or elevation motor failure. */
	    clamp_elevation_or_rotation_failure,
	    /* Elevation motor success. */
	    clamp_elevation_success,
	    /* Elevation motor failure. */
	    clamp_elevation_failure,
	    /* Rotation motor success. */
	    clamp_rotation_success,
	    /* Rotation motor failure. */
	    clamp_rotation_failure)

FSM_START_WITH (CLAMP_START)
FSM_START_WITH (CLAMP_MOVE_IDLE)

/** Clamp context. */
struct clamp_t
{
    /** True if clamp is working. */
    uint8_t working;
    /** Current position. */
    uint8_t pos_current;
    /** Requested position. */
    uint8_t pos_request;
    /** Element moving destination. */
    uint8_t moving_to;
    /** Position of a new element. */
    uint8_t pos_new;
    /** New element type. */
    uint8_t new_element_type;
    /** Drop direction, drop on the other side. */
    uint8_t drop_direction;
    /** True if clamp is open. */
    uint8_t open;
    /** True if clamp position is controled. */
    uint8_t controled;
    /** Contact state at start of move, for head check. */
    uint8_t contact_head_before_move;
};

/** Global context. */
struct clamp_t clamp_global;
#define ctx clamp_global

/** Clamp positions. */
static const uint16_t clamp_pos[][2] = {
      { BOT_CLAMP_SLOT_FRONT_BOTTOM_ELEVATION_STEP,
	BOT_CLAMP_SLOT_FRONT_BOTTOM_ROTATION_STEP },
      { BOT_CLAMP_SLOT_FRONT_MIDDLE_ELEVATION_STEP,
	BOT_CLAMP_SLOT_FRONT_MIDDLE_ROTATION_STEP },
      { BOT_CLAMP_SLOT_FRONT_TOP_ELEVATION_STEP,
	BOT_CLAMP_SLOT_FRONT_TOP_ROTATION_STEP },
      { BOT_CLAMP_SLOT_BACK_BOTTOM_ELEVATION_STEP,
	BOT_CLAMP_SLOT_BACK_BOTTOM_ROTATION_STEP },
      { BOT_CLAMP_SLOT_BACK_MIDDLE_ELEVATION_STEP,
	BOT_CLAMP_SLOT_BACK_MIDDLE_ROTATION_STEP },
      { BOT_CLAMP_SLOT_BACK_TOP_ELEVATION_STEP,
	BOT_CLAMP_SLOT_BACK_TOP_ROTATION_STEP },
      { BOT_CLAMP_SLOT_SIDE_ELEVATION_STEP,
	BOT_CLAMP_BAY_SIDE_ROTATION_STEP },
      { BOT_CLAMP_BAY_FRONT_LEAVE_ELEVATION_STEP,
	BOT_CLAMP_BAY_FRONT_ROTATION_STEP },
      { BOT_CLAMP_BAY_FRONT_LEAVE_ELEVATION_STEP,
	BOT_CLAMP_BAY_SIDE_ROTATION_STEP },
      { BOT_CLAMP_BAY_BACK_LEAVE_ELEVATION_STEP,
	BOT_CLAMP_BAY_BACK_ROTATION_STEP },
      { BOT_CLAMP_BAY_BACK_LEAVE_ELEVATION_STEP,
	BOT_CLAMP_BAY_SIDE_ROTATION_STEP },
      { BOT_CLAMP_BAY_SIDE_ENTER_LEAVE_ELEVATION_STEP,
	BOT_CLAMP_BAY_SIDE_ROTATION_STEP },
};

/** Slot doors. */
static const uint8_t clamp_slot_door[] = {
    BOT_PWM_DOOR_FRONT_BOTTOM,
    0xff,
    BOT_PWM_DOOR_FRONT_TOP,
    BOT_PWM_DOOR_BACK_BOTTOM,
    0xff,
    BOT_PWM_DOOR_BACK_TOP,
    0xff
};

static void
clamp_openclose (uint8_t open);

static void
clamp_route (void);

static void
clamp_head_check_prepare (uint8_t from);

void
clamp_init (void)
{
    ctx.open = 1;
}

uint8_t
clamp_working (void)
{
    return ctx.working;
}

void
clamp_move (uint8_t pos)
{
    if (pos != ctx.pos_current)
      {
	ctx.pos_request = pos;
	ctx.moving_to = CLAMP_POS_NB;
	FSM_HANDLE (AI, clamp_move);
      }
    else
	fsm_queue_post_event (FSM_EVENT (AI, clamp_move_success));
}

void
clamp_move_element (uint8_t from, uint8_t to)
{
    assert (from != to);
    ctx.pos_request = from;
    ctx.moving_to = to;
    clamp_head_check_prepare (from);
    FSM_HANDLE (AI, clamp_move);
}

void
clamp_new_element (uint8_t pos, uint8_t element_type)
{
    assert (pos == CLAMP_SLOT_FRONT_BOTTOM || pos == CLAMP_SLOT_BACK_BOTTOM);
    ctx.pos_new = pos;
    ctx.new_element_type = element_type;
    FSM_HANDLE (AI, clamp_new_element);
}

void
clamp_prepare (uint8_t prepare)
{
    logistic_global.prepare = prepare;
    FSM_HANDLE (AI, clamp_prepare);
}

uint8_t
clamp_drop (uint8_t drop_direction)
{
    if (FSM_CAN_HANDLE (AI, clamp_drop))
      {
	ctx.drop_direction = drop_direction;
	FSM_HANDLE (AI, clamp_drop);
	return 1;
      }
    else
	return 0;
}

void
clamp_drop_clear (void)
{
    FSM_HANDLE (AI, clamp_drop_clear);
}

void
clamp_door (uint8_t pos, uint8_t open)
{
    if (pos == 0xff)
	clamp_openclose (open);
    else if (clamp_slot_door[pos] != 0xff)
      {
	if (open)
	    pwm_set_timed (clamp_slot_door[pos], BOT_PWM_DOOR_OPEN (pos));
	else
	    pwm_set_timed (clamp_slot_door[pos], BOT_PWM_DOOR_CLOSE (pos));
      }
}

uint8_t
clamp_handle_event (void)
{
    if (FSM_CAN_HANDLE (AI, clamp_new_element))
      {
	uint8_t element_type;
	element_type = pawn_sensor_get (DIRECTION_FORWARD);
	if (element_type)
	  {
	    clamp_new_element (CLAMP_SLOT_FRONT_BOTTOM, element_type);
	    return 1;
	  }
	element_type = pawn_sensor_get (DIRECTION_BACKWARD);
	if (element_type)
	  {
	    clamp_new_element (CLAMP_SLOT_BACK_BOTTOM, element_type);
	    return 1;
	  }
      }
    /* Handle special hardware offset. */
    uint16_t rotation_position = mimot_get_motor1_position ();
    if ((ctx.pos_current == CLAMP_BAY_FRONT_LEAVING
	 && rotation_position > (BOT_CLAMP_BAY_SIDE_ROTATION_STEP +
				 BOT_CLAMP_BAY_FRONT_ROTATION_STEP) / 2)
	|| (ctx.pos_current == CLAMP_BAY_BACK_LEAVING
	    && rotation_position < (BOT_CLAMP_BAY_BACK_ROTATION_STEP +
				    BOT_CLAMP_BAY_SIDE_ROTATION_STEP) / 2))
      {
	/* Go directly to next point. */
	clamp_route ();
      }
    return 0;
}

/** Open or close clamp and adjust rotation. */
static void
clamp_openclose (uint8_t open)
{
    if (open)
	pwm_set_timed (BOT_PWM_CLAMP, BOT_PWM_CLAMP_OPEN);
    else
	pwm_set_timed (BOT_PWM_CLAMP, BOT_PWM_CLAMP_CLOSE);
    if (ctx.controled)
      {
	int16_t offset = open ? 0
	    : BOT_CLAMP_CLOSED_ROTATION_OFFSET (ctx.pos_current);
	mimot_move_motor1_absolute (clamp_pos[ctx.pos_current][1] + offset,
				    BOT_CLAMP_ROTATION_OFFSET_SPEED);
      }
    ctx.open = open;
}

/** Find next position and start motors. */
static void
clamp_route (void)
{
    uint8_t pos_new;
    uint8_t pos_current = ctx.pos_current;
    uint8_t pos_request = ctx.pos_request;
    /* Compute new position. */
    if (CLAMP_IS_SLOT_IN_FRONT_BAY (pos_current))
      {
	if (!CLAMP_IS_SLOT_IN_FRONT_BAY (pos_request))
	    pos_new = CLAMP_BAY_FRONT_LEAVE;
	else
	    pos_new = pos_request;
      }
    else if (CLAMP_IS_SLOT_IN_BACK_BAY (pos_current))
      {
	if (!CLAMP_IS_SLOT_IN_BACK_BAY (pos_request))
	    pos_new = CLAMP_BAY_BACK_LEAVE;
	else
	    pos_new = pos_request;
      }
    else if (pos_current == CLAMP_SLOT_SIDE)
      {
	pos_new = CLAMP_BAY_SIDE_ENTER_LEAVE;
      }
    else if (pos_current == CLAMP_BAY_FRONT_LEAVE)
      {
	if (CLAMP_IS_SLOT_IN_FRONT_BAY (pos_request))
	    pos_new = pos_request;
	else
	    pos_new = CLAMP_BAY_FRONT_LEAVING;
      }
    else if (pos_current == CLAMP_BAY_BACK_LEAVE)
      {
	if (CLAMP_IS_SLOT_IN_BACK_BAY (pos_request))
	    pos_new = pos_request;
	else
	    pos_new = CLAMP_BAY_BACK_LEAVING;
      }
    else if (pos_current == CLAMP_BAY_FRONT_LEAVING)
      {
	if (pos_request == CLAMP_SLOT_SIDE
	    || pos_request == CLAMP_BAY_SIDE_ENTER_LEAVE)
	    pos_new = CLAMP_BAY_SIDE_ENTER_LEAVE;
	else
	    pos_new = CLAMP_SLOT_BACK_MIDDLE;
      }
    else if (pos_current == CLAMP_BAY_BACK_LEAVING)
      {
	if (pos_request == CLAMP_SLOT_SIDE
	    || pos_request == CLAMP_BAY_SIDE_ENTER_LEAVE)
	    pos_new = CLAMP_BAY_SIDE_ENTER_LEAVE;
	else
	    pos_new = CLAMP_SLOT_FRONT_MIDDLE;
      }
    else
      {
	assert (pos_current == CLAMP_BAY_SIDE_ENTER_LEAVE);
	if (pos_request == CLAMP_SLOT_SIDE)
	    pos_new = pos_request;
	else if (CLAMP_IS_SLOT_IN_FRONT_BAY (pos_request))
	    pos_new = CLAMP_SLOT_FRONT_MIDDLE;
	else
	    pos_new = CLAMP_SLOT_BACK_MIDDLE;
      }
    /* Run motors. */
    mimot_move_motor0_absolute (clamp_pos[pos_new][0],
				BOT_CLAMP_ELEVATION_SPEED);
    int16_t offset = ctx.open ? 0
	: BOT_CLAMP_CLOSED_ROTATION_OFFSET (pos_new);
    mimot_move_motor1_absolute (clamp_pos[pos_new][1] + offset,
				BOT_CLAMP_ROTATION_SPEED);
    ctx.controled = 1;
    /* Remember new position. */
    ctx.pos_current = pos_new;
}

static void
clamp_taken_pawn (uint8_t element_type)
{
    position_t robot_pos;
    asserv_get_position (&robot_pos);
    element_taken (element_nearest_element_id (robot_pos), element_type);
}

/* When lifting an element, we can discover it is actually a head.  In this
 * case, change destination. */
static void
clamp_head_check (void)
{
    uint8_t from = logistic_global.moving_from;
    if (!ctx.contact_head_before_move && logistic_global.slots[from] == ELEMENT_PAWN)
      {
	/* Look head contact. */
	uint8_t contact_head = 0;
	if (from == CLAMP_SLOT_FRONT_BOTTOM)
	    contact_head = !IO_GET (CONTACT_FRONT_TOP);
	else if (from == CLAMP_SLOT_BACK_BOTTOM)
	    contact_head = !IO_GET (CONTACT_BACK_TOP);
	/* Change? */
	if (contact_head)
	  {
	    logistic_element_change (from, ELEMENT_KING);
	    clamp_taken_pawn (ELEMENT_HEAD);
	    if (logistic_global.moving_from != from)
		/* Cancel move. */
		ctx.pos_request = ctx.moving_to = from;
	    else
		/* Change move. */
		ctx.pos_request = ctx.moving_to = logistic_global.moving_to;
	  }
      }
}

/** Prepare head check, if contact is set yet, this is not a new head. */
static void
clamp_head_check_prepare (uint8_t from)
{
    ctx.contact_head_before_move = 0;
    if (CLAMP_IS_SLOT_IN_FRONT_BAY (from))
	ctx.contact_head_before_move = !IO_GET (CONTACT_FRONT_TOP);
    else if (CLAMP_IS_SLOT_IN_BACK_BAY (from))
	ctx.contact_head_before_move = !IO_GET (CONTACT_BACK_TOP);
}

/* When clamp moved to bottom slot, we can discover it is actually a tower.
 * In this case, stop movement. */
static uint8_t
clamp_tower_check (void)
{
    uint8_t from = logistic_global.moving_from;
    if ((from == CLAMP_SLOT_FRONT_BOTTOM || from == CLAMP_SLOT_BACK_BOTTOM)
	&& ctx.pos_current == from
	&& logistic_global.slots[from] == ELEMENT_PAWN)
      {
	/* Check for green zone. */
	position_t robot_pos;
	asserv_get_position (&robot_pos);
	if (robot_pos.v.x < 450 || robot_pos.v.x > PG_WIDTH - 450)
	    return 0;
	/* Look tower contact. */
	uint8_t contact_tower;
	if (from == CLAMP_SLOT_FRONT_BOTTOM)
	    contact_tower = !IO_GET (CONTACT_FRONT_MIDDLE);
	else
	    contact_tower = !IO_GET (CONTACT_BACK_MIDDLE);
	/* Change? */
	if (contact_tower)
	  {
	    logistic_element_change (from, ELEMENT_TOWER);
	    clamp_taken_pawn (ELEMENT_TOWER);
	    return 1;
	  }
      }
    return 0;
}

static void
clamp_blocked (void)
{
    /* Free everything. */
    clamp_openclose (1);
    uint16_t rotation_position = mimot_get_motor1_position ();
    uint16_t elevation_position = mimot_get_motor0_position ();
    if (rotation_position < BOT_CLAMP_BAY_SIDE_ROTATION_STEP
	- BOT_CLAMP_BAY_SIDE_MARGIN_ROTATION_STEP)
      {
	clamp_door (CLAMP_SLOT_FRONT_BOTTOM, 1);
	if (elevation_position
	    > (BOT_CLAMP_SLOT_FRONT_MIDDLE_ELEVATION_STEP
	       + BOT_CLAMP_SLOT_FRONT_TOP_ELEVATION_STEP) / 2)
	  {
	    clamp_door (CLAMP_SLOT_FRONT_TOP, 1);
	    logistic_dump (DIRECTION_FORWARD, 1);
	  }
	else
	    logistic_dump (DIRECTION_FORWARD, 0);
      }
    else if (rotation_position > BOT_CLAMP_BAY_SIDE_ROTATION_STEP
	+ BOT_CLAMP_BAY_SIDE_MARGIN_ROTATION_STEP)
      {
	clamp_door (CLAMP_SLOT_BACK_BOTTOM, 1);
	if (elevation_position
	    > (BOT_CLAMP_SLOT_BACK_MIDDLE_ELEVATION_STEP
	       + BOT_CLAMP_SLOT_BACK_TOP_ELEVATION_STEP) / 2)
	  {
	    clamp_door (CLAMP_SLOT_BACK_TOP, 1);
	    logistic_dump (DIRECTION_BACKWARD, 1);
	  }
	else
	    logistic_dump (DIRECTION_BACKWARD, 0);
      }
    mimot_motor0_free ();
    mimot_motor1_free ();
    /* Signal problem. */
    fsm_queue_post_event (FSM_EVENT (AI, clamp_move_failure));
}

#define CLAMP_DECISION_MOVE_ELEMENT 0
#define CLAMP_DECISION_MOVE_TO_IDLE 1
#define CLAMP_DECISION_CLAMP_LOCKED 2
#define CLAMP_DECISION_DONE 3

static uint8_t
clamp_decision (uint8_t unblock)
{
    if (!unblock && logistic_global.moving_from != CLAMP_SLOT_NB)
      {
	clamp_move_element (logistic_global.moving_from,
			    logistic_global.moving_to);
	return CLAMP_DECISION_MOVE_ELEMENT;
      }
    else if (logistic_global.prepare != 3
	     && (unblock
		 || logistic_global.clamp_pos_idle != ctx.pos_current))
      {
	if (logistic_path_clear (ctx.pos_current,
				 logistic_global.clamp_pos_idle))
	  {
	    clamp_move (logistic_global.clamp_pos_idle);
	    return CLAMP_DECISION_MOVE_TO_IDLE;
	  }
	else
	  {
	    ctx.working = 0;
	    fsm_queue_post_event (FSM_EVENT (AI, clamp_done));
	    return CLAMP_DECISION_CLAMP_LOCKED;
	  }
      }
    else
      {
	ctx.working = 0;
	fsm_queue_post_event (FSM_EVENT (AI, clamp_done));
	return CLAMP_DECISION_DONE;
      }
}

/* CLAMP FSM */

FSM_TRANS (CLAMP_START, init_actuators, CLAMP_INIT_OPENING)
{
    pwm_set_timed (BOT_PWM_DOOR_FRONT_BOTTOM, BOT_PWM_DOOR_INIT);
    pwm_set_timed (BOT_PWM_DOOR_FRONT_TOP, BOT_PWM_DOOR_INIT);
    pwm_set_timed (BOT_PWM_DOOR_BACK_BOTTOM, BOT_PWM_DOOR_INIT);
    pwm_set_timed (BOT_PWM_DOOR_BACK_TOP, BOT_PWM_DOOR_INIT);
    pwm_set_timed (BOT_PWM_CLAMP, BOT_PWM_CLAMP_INIT);
    return FSM_NEXT (CLAMP_START, init_actuators);
}

FSM_TRANS_TIMEOUT (CLAMP_INIT_OPENING, BOT_PWM_CLAMP_DOOR_INIT,
		   CLAMP_INIT_GOING_MIDDLE)
{
    mimot_move_motor0_absolute (mimot_get_motor0_position () +
				BOT_CLAMP_INIT_ELEVATION_STEP,
				BOT_CLAMP_INIT_ELEVATION_SPEED);
    return FSM_NEXT_TIMEOUT (CLAMP_INIT_OPENING);
}

FSM_TRANS (CLAMP_INIT_GOING_MIDDLE, clamp_elevation_success,
	   CLAMP_INIT_FINDING_ROTATION_EDGE)
{
    mimot_motor1_find_zero (BOT_CLAMP_INIT_ROTATION_SPEED, 0, 0);
    return FSM_NEXT (CLAMP_INIT_GOING_MIDDLE, clamp_elevation_success);
}

FSM_TRANS (CLAMP_INIT_FINDING_ROTATION_EDGE, clamp_rotation_success,
	   CLAMP_INIT_FINDING_TOP)
{
    mimot_motor0_find_zero (BOT_CLAMP_INIT_ELEVATION_SPEED, 1,
			    BOT_CLAMP_INIT_ELEVATION_SWITCH_STEP);
    return FSM_NEXT (CLAMP_INIT_FINDING_ROTATION_EDGE,
		     clamp_rotation_success);
}

FSM_TRANS (CLAMP_INIT_FINDING_TOP, clamp_elevation_success,
	   rest, CLAMP_INIT_GOING_REST,
	   demo, CLAMP_GOING_IDLE)
{
    if (IO_GET (CONTACT_STRAT))
      {
	clamp_move (CLAMP_BAY_SIDE_ENTER_LEAVE);
	return FSM_NEXT (CLAMP_INIT_FINDING_TOP, clamp_elevation_success,
			 rest);
      }
    else
      {
	clamp_move (logistic_global.clamp_pos_idle);
	return FSM_NEXT (CLAMP_INIT_FINDING_TOP, clamp_elevation_success,
			 demo);
      }
}

FSM_TRANS (CLAMP_INIT_GOING_REST, clamp_move_success, CLAMP_INIT_READY)
{
    mimot_motor0_free ();
    mimot_motor1_free ();
    return FSM_NEXT (CLAMP_INIT_GOING_REST, clamp_move_success);
}

FSM_TRANS (CLAMP_INIT_READY, init_start_round, CLAMP_IDLE)
{
    pwm_set (BOT_PWM_DOOR_FRONT_BOTTOM, BOT_PWM_DOOR_INIT_START);
    pwm_set (BOT_PWM_DOOR_FRONT_TOP, BOT_PWM_DOOR_INIT_START);
    pwm_set (BOT_PWM_DOOR_BACK_BOTTOM, BOT_PWM_DOOR_INIT_START);
    pwm_set (BOT_PWM_DOOR_BACK_TOP, BOT_PWM_DOOR_INIT_START);
    //clamp_move (logistic_global.clamp_pos_idle);
    return FSM_NEXT (CLAMP_INIT_READY, init_start_round);
}

FSM_TRANS (CLAMP_GOING_IDLE, clamp_move_success, CLAMP_IDLE)
{
    ctx.working = 0;
    fsm_queue_post_event (FSM_EVENT (AI, clamp_done));
    return FSM_NEXT (CLAMP_GOING_IDLE, clamp_move_success);
}

FSM_TRANS (CLAMP_GOING_IDLE, clamp_move_failure, CLAMP_BLOCKED)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_blocked));
    return FSM_NEXT (CLAMP_GOING_IDLE, clamp_move_failure);
}

FSM_TRANS (CLAMP_IDLE, clamp_new_element, CLAMP_TAKING_DOOR_CLOSING)
{
    ctx.working = 1;
    fsm_queue_post_event (FSM_EVENT (AI, clamp_working));
    pwm_set_timed (clamp_slot_door[ctx.pos_new],
		   BOT_PWM_DOOR_CLOSE (ctx.pos_new));
    return FSM_NEXT (CLAMP_IDLE, clamp_new_element);
}

FSM_TRANS (CLAMP_IDLE, clamp_prepare,
	   move_element, CLAMP_MOVING_ELEMENT,
	   move_to_idle, CLAMP_GOING_IDLE,
	   clamp_locked, CLAMP_LOCKED,
	   done, CLAMP_IDLE)
{
    logistic_decision ();
    switch (clamp_decision (0))
      {
      default:
      case CLAMP_DECISION_MOVE_ELEMENT:
	return FSM_NEXT (CLAMP_IDLE, clamp_prepare, move_element);
      case CLAMP_DECISION_MOVE_TO_IDLE:
	return FSM_NEXT (CLAMP_IDLE, clamp_prepare, move_to_idle);
      case CLAMP_DECISION_CLAMP_LOCKED:
	return FSM_NEXT (CLAMP_IDLE, clamp_prepare, clamp_locked);
      case CLAMP_DECISION_DONE:
	return FSM_NEXT (CLAMP_IDLE, clamp_prepare, done);
      }
}

FSM_TRANS (CLAMP_IDLE, clamp_drop, CLAMP_DROPING_DOOR_OPENING)
{
    /* If going forward, drop at back. */
    uint8_t bay = ctx.drop_direction == DIRECTION_FORWARD
	? CLAMP_SLOT_BACK_BOTTOM : CLAMP_SLOT_FRONT_BOTTOM;
    pwm_set_timed (clamp_slot_door[bay + 0], BOT_PWM_DOOR_OPEN (bay + 0));
    pwm_set_timed (clamp_slot_door[bay + 2], BOT_PWM_DOOR_OPEN (bay + 2));
    return FSM_NEXT (CLAMP_IDLE, clamp_drop);
}

FSM_TRANS_TIMEOUT (CLAMP_TAKING_DOOR_CLOSING, BOT_PWM_DOOR_CLOSE_TIME,
		   move_element, CLAMP_MOVING_ELEMENT,
		   move_to_idle, CLAMP_GOING_IDLE,
		   clamp_locked, CLAMP_LOCKED,
		   done, CLAMP_IDLE)
{
    logistic_element_new (ctx.pos_new, ctx.new_element_type);
    clamp_taken_pawn (ctx.new_element_type);
    switch (clamp_decision (0))
      {
      default:
      case CLAMP_DECISION_MOVE_ELEMENT:
	return FSM_NEXT_TIMEOUT (CLAMP_TAKING_DOOR_CLOSING, move_element);
      case CLAMP_DECISION_MOVE_TO_IDLE:
	return FSM_NEXT_TIMEOUT (CLAMP_TAKING_DOOR_CLOSING, move_to_idle);
      case CLAMP_DECISION_CLAMP_LOCKED:
	return FSM_NEXT_TIMEOUT (CLAMP_TAKING_DOOR_CLOSING, clamp_locked);
      case CLAMP_DECISION_DONE:
	return FSM_NEXT_TIMEOUT (CLAMP_TAKING_DOOR_CLOSING, done);
      }
}

FSM_TRANS (CLAMP_MOVING_ELEMENT, clamp_move_success,
	   move_element, CLAMP_MOVING_ELEMENT,
	   move_to_idle, CLAMP_GOING_IDLE,
	   clamp_locked, CLAMP_LOCKED,
	   done, CLAMP_IDLE)
{
    logistic_element_move_done ();
    switch (clamp_decision (0))
      {
      default:
      case CLAMP_DECISION_MOVE_ELEMENT:
	return FSM_NEXT (CLAMP_MOVING_ELEMENT, clamp_move_success,
			 move_element);
      case CLAMP_DECISION_MOVE_TO_IDLE:
	return FSM_NEXT (CLAMP_MOVING_ELEMENT, clamp_move_success,
			 move_to_idle);
      case CLAMP_DECISION_CLAMP_LOCKED:
	return FSM_NEXT (CLAMP_MOVING_ELEMENT, clamp_move_success,
			 clamp_locked);
      case CLAMP_DECISION_DONE:
	return FSM_NEXT (CLAMP_MOVING_ELEMENT, clamp_move_success,
			 done);
      }
}

FSM_TRANS (CLAMP_MOVING_ELEMENT, clamp_move_failure, CLAMP_BLOCKED)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_blocked));
    return FSM_NEXT (CLAMP_MOVING_ELEMENT, clamp_move_failure);
}

FSM_TRANS_TIMEOUT (CLAMP_DROPING_DOOR_OPENING, BOT_PWM_CLAMP_OPEN_TIME,
		   CLAMP_DROPING_WAITING_ROBOT)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_drop_waiting));
    return FSM_NEXT_TIMEOUT (CLAMP_DROPING_DOOR_OPENING);
}

FSM_TRANS (CLAMP_DROPING_WAITING_ROBOT, clamp_drop_clear,
	   move_element, CLAMP_MOVING_ELEMENT,
	   move_to_idle, CLAMP_GOING_IDLE,
	   clamp_locked, CLAMP_LOCKED,
	   done, CLAMP_IDLE)
{
    logistic_drop (ctx.drop_direction);
    switch (clamp_decision (0))
      {
      default:
      case CLAMP_DECISION_MOVE_ELEMENT:
	return FSM_NEXT (CLAMP_DROPING_WAITING_ROBOT, clamp_drop_clear,
			 move_element);
      case CLAMP_DECISION_MOVE_TO_IDLE:
	return FSM_NEXT (CLAMP_DROPING_WAITING_ROBOT, clamp_drop_clear,
			 move_to_idle);
      case CLAMP_DECISION_CLAMP_LOCKED:
	return FSM_NEXT (CLAMP_DROPING_WAITING_ROBOT, clamp_drop_clear,
			 clamp_locked);
      case CLAMP_DECISION_DONE:
	return FSM_NEXT (CLAMP_DROPING_WAITING_ROBOT, clamp_drop_clear,
			 done);
      }
}

FSM_TRANS (CLAMP_LOCKED, clamp_new_element, CLAMP_LOCKED)
{
    pwm_set_timed (clamp_slot_door[ctx.pos_new],
		   BOT_PWM_DOOR_CLOSE (ctx.pos_new));
    logistic_element_new (ctx.pos_new, ctx.new_element_type);
    clamp_taken_pawn (ctx.new_element_type);
    return FSM_NEXT (CLAMP_LOCKED, clamp_new_element);
}

FSM_TRANS (CLAMP_LOCKED, clamp_drop, CLAMP_DROPING_DOOR_OPENING)
{
    /* If going forward, drop at back. */
    uint8_t bay = ctx.drop_direction == DIRECTION_FORWARD
	? CLAMP_SLOT_BACK_BOTTOM : CLAMP_SLOT_FRONT_BOTTOM;
    pwm_set_timed (clamp_slot_door[bay + 0], BOT_PWM_DOOR_OPEN (bay + 0));
    pwm_set_timed (clamp_slot_door[bay + 2], BOT_PWM_DOOR_OPEN (bay + 2));
    return FSM_NEXT (CLAMP_LOCKED, clamp_drop);
}

FSM_TRANS (CLAMP_BLOCKED, clamp_prepare,
	   move_to_idle, CLAMP_GOING_IDLE,
	   clamp_locked, CLAMP_LOCKED,
	   done, CLAMP_IDLE)
{
    switch (clamp_decision (1))
      {
      default:
      case CLAMP_DECISION_MOVE_TO_IDLE:
	return FSM_NEXT (CLAMP_BLOCKED, clamp_prepare, move_to_idle);
      case CLAMP_DECISION_CLAMP_LOCKED:
	return FSM_NEXT (CLAMP_BLOCKED, clamp_prepare, clamp_locked);
      case CLAMP_DECISION_DONE:
	return FSM_NEXT (CLAMP_BLOCKED, clamp_prepare, done);
      }
}

/* CLAMP_MOVE FSM */

FSM_TRANS (CLAMP_MOVE_IDLE, clamp_move,
	   move, CLAMP_MOVE_ROUTING,
	   move_element, CLAMP_MOVE_SRC_ROUTING,
	   move_element_here, CLAMP_MOVE_SRC_CLAMP_CLOSING)
{
    if (ctx.moving_to == CLAMP_POS_NB)
      {
	clamp_route ();
	return FSM_NEXT (CLAMP_MOVE_IDLE, clamp_move, move);
      }
    else
      {
	if (ctx.pos_current != ctx.pos_request)
	  {
	    clamp_route ();
	    return FSM_NEXT (CLAMP_MOVE_IDLE, clamp_move, move_element);
	  }
	else
	  {
	    clamp_openclose (0);
	    return FSM_NEXT (CLAMP_MOVE_IDLE, clamp_move, move_element_here);
	  }
      }
}

FSM_TRANS (CLAMP_MOVE_ROUTING, clamp_elevation_rotation_success,
	   done, CLAMP_MOVE_IDLE,
	   next, CLAMP_MOVE_ROUTING)
{
    if (ctx.pos_current == ctx.pos_request)
      {
	fsm_queue_post_event (FSM_EVENT (AI, clamp_move_success));
	return FSM_NEXT (CLAMP_MOVE_ROUTING, clamp_elevation_rotation_success,
			 done);
      }
    else
      {
	clamp_route ();
	return FSM_NEXT (CLAMP_MOVE_ROUTING, clamp_elevation_rotation_success,
			 next);
      }
}

FSM_TRANS (CLAMP_MOVE_ROUTING, clamp_elevation_or_rotation_failure,
	   CLAMP_MOVE_IDLE)
{
    clamp_blocked ();
    return FSM_NEXT (CLAMP_MOVE_ROUTING, clamp_elevation_or_rotation_failure);
}

FSM_TRANS (CLAMP_MOVE_SRC_ROUTING, clamp_elevation_rotation_success,
	   cancel, CLAMP_MOVE_IDLE,
	   done, CLAMP_MOVE_SRC_CLAMP_CLOSING,
	   next, CLAMP_MOVE_SRC_ROUTING)
{
    if (clamp_tower_check ())
      {
	fsm_queue_post_event (FSM_EVENT (AI, clamp_move_success));
	return FSM_NEXT (CLAMP_MOVE_SRC_ROUTING, clamp_elevation_rotation_success,
			 cancel);
      }
    else if (ctx.pos_current == ctx.pos_request)
      {
	clamp_openclose (0);
	return FSM_NEXT (CLAMP_MOVE_SRC_ROUTING,
			 clamp_elevation_rotation_success, done);
      }
    else
      {
	clamp_route ();
	return FSM_NEXT (CLAMP_MOVE_SRC_ROUTING,
			 clamp_elevation_rotation_success, next);
      }
}

FSM_TRANS (CLAMP_MOVE_SRC_ROUTING, clamp_elevation_or_rotation_failure,
	   CLAMP_MOVE_IDLE)
{
    clamp_blocked ();
    return FSM_NEXT (CLAMP_MOVE_SRC_ROUTING,
		     clamp_elevation_or_rotation_failure);
}

FSM_TRANS_TIMEOUT (CLAMP_MOVE_SRC_CLAMP_CLOSING, BOT_PWM_CLAMP_CLOSE_TIME,
		   open_door, CLAMP_MOVE_SRC_DOOR_OPENDING,
		   move, CLAMP_MOVE_DST_ROUTING)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_taken));
    if (clamp_slot_door[ctx.pos_current] != 0xff)
      {
	pwm_set_timed (clamp_slot_door[ctx.pos_current],
		       BOT_PWM_DOOR_OPEN (ctx.pos_current));
	return FSM_NEXT_TIMEOUT (CLAMP_MOVE_SRC_CLAMP_CLOSING, open_door);
      }
    else
      {
	ctx.pos_request = ctx.moving_to;
	clamp_route ();
	return FSM_NEXT_TIMEOUT (CLAMP_MOVE_SRC_CLAMP_CLOSING, move);
      }
}

FSM_TRANS_TIMEOUT (CLAMP_MOVE_SRC_DOOR_OPENDING, BOT_PWM_DOOR_OPEN_TIME,
		   CLAMP_MOVE_DST_ROUTING)
{
    ctx.pos_request = ctx.moving_to;
    clamp_route ();
    return FSM_NEXT_TIMEOUT (CLAMP_MOVE_SRC_DOOR_OPENDING);
}

FSM_TRANS (CLAMP_MOVE_DST_ROUTING, clamp_elevation_rotation_success,
	   done_close_door, CLAMP_MOVE_DST_DOOR_CLOSING,
	   done_open_clamp, CLAMP_MOVE_DST_CLAMP_OPENING,
	   next, CLAMP_MOVE_DST_ROUTING)
{
    clamp_head_check ();
    if (ctx.pos_current == ctx.pos_request)
      {
	if (clamp_slot_door[ctx.pos_current] != 0xff)
	  {
	    pwm_set_timed (clamp_slot_door[ctx.pos_current],
			   BOT_PWM_DOOR_CLOSE (ctx.pos_current));
	    return FSM_NEXT (CLAMP_MOVE_DST_ROUTING,
			     clamp_elevation_rotation_success,
			     done_close_door);
	  }
	else
	  {
	    clamp_openclose (1);
	    return FSM_NEXT (CLAMP_MOVE_DST_ROUTING,
			     clamp_elevation_rotation_success,
			     done_open_clamp);
	  }
      }
    else
      {
	clamp_route ();
	return FSM_NEXT (CLAMP_MOVE_DST_ROUTING,
			 clamp_elevation_rotation_success, next);
      }
}

FSM_TRANS (CLAMP_MOVE_DST_ROUTING, clamp_elevation_or_rotation_failure,
	   CLAMP_MOVE_IDLE)
{
    clamp_blocked ();
    return FSM_NEXT (CLAMP_MOVE_DST_ROUTING,
		     clamp_elevation_or_rotation_failure);
}

FSM_TRANS_TIMEOUT (CLAMP_MOVE_DST_DOOR_CLOSING, BOT_PWM_DOOR_CLOSE_TIME,
		   CLAMP_MOVE_DST_CLAMP_OPENING)
{
    clamp_openclose (1);
    return FSM_NEXT_TIMEOUT (CLAMP_MOVE_DST_DOOR_CLOSING);
}

FSM_TRANS_TIMEOUT (CLAMP_MOVE_DST_CLAMP_OPENING, BOT_PWM_CLAMP_OPEN_TIME,
		   CLAMP_MOVE_IDLE)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_move_success));
    return FSM_NEXT_TIMEOUT (CLAMP_MOVE_DST_CLAMP_OPENING);
}

