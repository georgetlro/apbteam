/* top.c */
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
#include "io.h"

#include "playground_2011.h"
#include "asserv.h"

#define FSM_NAME AI
#include "fsm.h"

#include "logistic.h"
#include "move.h"
#include "chrono.h"
#include "pawn_sensor.h"
#include "contact.h"

/*
 * Here is the top FSM.  This FSM is suppose to give life to the robot with an
 * impression of intelligence... Well...
 */

#define TOP_PAWN_TIME 45000l

FSM_INIT

FSM_STATES (
	    /* Initial state. */
	    TOP_START,
	    /* Going out of start area. */
	    TOP_GOING_OUT1,
	    /* Problem going out, wait before retry. */
	    TOP_GOING_OUT1_BLOCK_WAIT,
	    /* Going out, first pawn emplacement. */
	    TOP_GOING_OUT2,
	    /* Problem going out, wait before retry. */
	    TOP_GOING_OUT2_BLOCK_WAIT,

	    TOP_GOING_TO_DROP,
	    TOP_GOING_TO_ELEMENT,
	    /* Waiting clamp has finished its work. */
	    TOP_WAITING_CLAMP,
	    /* Unblocking: waiting a little bit. */
	    TOP_UNBLOCKING_SHAKE_WAIT,
	    /* Unblocking: shaking. */
	    TOP_UNBLOCKING_SHAKE,
	    /* Waiting construction is ready to drop. */
	    TOP_WAITING_READY,
	    /* Dropping, opening the doors. */
	    TOP_DROP_DROPPING,
	    /* Dropping, clearing so that doors can be closed. */
	    TOP_DROP_CLEARING)

FSM_EVENTS (
	    /* Bumpers have seen something. */
	    top_bumper)

FSM_START_WITH (TOP_START)

/** Top context. */
struct top_t
{
    /** Target element. */
    uint8_t target_element_id;
    /** Chaos counter. */
    uint8_t chaos;
    /** Broken clamp. */
    uint8_t broken;
    /** Saved, direction when picking element. */
    uint8_t go_to_element_direction;
    /** Return to our green zone again. */
    uint8_t green_again;
};

/** Global context. */
struct top_t top_global;
#define ctx top_global

FSM_TRANS (TOP_START, init_start_round, TOP_GOING_OUT1)
{
    element_init ();
    ctx.broken = 1;
    ctx.green_again = 3;
    asserv_goto (PG_X (PG_GREEN_WIDTH_MM + 100),
		 PG_Y (PG_LENGTH - 200), 0);
    return FSM_NEXT (TOP_START, init_start_round);
}

FSM_TRANS (TOP_GOING_OUT1, robot_move_success, TOP_GOING_OUT2)
{
    asserv_goto (PG_X (1500 - 2 * 350), PG_Y (PG_LENGTH - 350), 0);
    return FSM_NEXT (TOP_GOING_OUT1, robot_move_success);
}

FSM_TRANS (TOP_GOING_OUT1, robot_move_failure, TOP_GOING_OUT1_BLOCK_WAIT)
{
    return FSM_NEXT (TOP_GOING_OUT1, robot_move_failure);
}

FSM_TRANS_TIMEOUT (TOP_GOING_OUT1_BLOCK_WAIT, 250, TOP_GOING_OUT1)
{
    asserv_goto (PG_X (PG_GREEN_WIDTH_MM + 100),
		 PG_Y (PG_LENGTH - 200 - (++ctx.chaos % 4) * 10), 0);
    return FSM_NEXT_TIMEOUT (TOP_GOING_OUT1_BLOCK_WAIT);
}

static uint8_t
top_prepare_level (void)
{
    if (ctx.broken)
	return 3;
    else if (logistic_global.need_prepare
	     || chrono_remaining_time () < TOP_PAWN_TIME)
	return 2;
    else
	return 1;
}

static void
top_go_this_element (vect_t pos, int16_t shorten)
{
    ctx.go_to_element_direction = logistic_global.collect_direction;
    uint8_t backward = logistic_global.collect_direction == DIRECTION_FORWARD
	? 0 : ASSERV_BACKWARD;
    move_start_noangle (pos, backward, shorten);
}

static uint8_t
top_go_element (void)
{
    position_t robot_pos;
    asserv_get_position (&robot_pos);
    ctx.target_element_id = element_best (robot_pos);
    element_t e = element_get (ctx.target_element_id);
    if (e.attr & ELEMENT_GREEN)
      {
	if (!ctx.broken)
	    logistic_global.prepare = 0;
	else
	    logistic_global.prepare = top_prepare_level ();
	pawn_sensor_bumper_enable (0);
      }
    else
      {
	logistic_global.prepare = top_prepare_level ();
	pawn_sensor_bumper_enable (1);
      }
    vect_t element_pos = element_get_pos (ctx.target_element_id);
    top_go_this_element (element_pos, 0);
    return 1;
}

static uint8_t
top_go_drop (void)
{
    position_t robot_pos;
    asserv_get_position (&robot_pos);
    ctx.target_element_id = element_unload_best (robot_pos);
    position_t drop_pos;
    drop_pos.v = element_get_pos (ctx.target_element_id);
    logistic_global.prepare = top_prepare_level ();
    pawn_sensor_bumper_enable (0);
    uint8_t backward = logistic_global.collect_direction == DIRECTION_FORWARD
	? 0 : ASSERV_BACKWARD;
    /* Go above or below the drop point. */
    if (drop_pos.v.y < 350)
      {
	/* Protected zone. */
	drop_pos.v.x = PG_WIDTH / 2;
	drop_pos.v.y = 350;
	drop_pos.a = PG_A_DEG (45);
      }
    else if (drop_pos.v.y > PG_LENGTH / 2)
      {
	drop_pos.v.y -= 350 / 2;
	drop_pos.a = PG_A_DEG (-90);
      }
    else
      {
	drop_pos.v.y += 350 / 2;
	drop_pos.a = PG_A_DEG (90);
      }
    if (logistic_global.collect_direction == DIRECTION_BACKWARD)
	drop_pos.a += POSITION_A_DEG (180);
    /* Go! */
    move_start (drop_pos, backward);
    return 0;
}

static uint8_t
top_decision (void)
{
    /* If we can make a tower. */
    if (logistic_global.construct_possible == 1
	|| ((ctx.broken || chrono_remaining_time () < TOP_PAWN_TIME)
	    && logistic_global.construct_possible == 2))
	return top_go_drop ();
    if (logistic_global.need_prepare)
      {
	clamp_prepare (top_prepare_level ());
	return top_go_drop ();
      }
    else
	return top_go_element ();
}

FSM_TRANS (TOP_GOING_OUT2, robot_move_success,
	   clamp_working, TOP_WAITING_CLAMP,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    if (clamp_working ())
	return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_success, clamp_working);
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_GOING_OUT2, robot_move_success, drop);
      case 1: return FSM_NEXT (TOP_GOING_OUT2, robot_move_success, element);
      }
}

FSM_TRANS (TOP_GOING_OUT2, robot_move_failure, TOP_GOING_OUT2_BLOCK_WAIT)
{
    return FSM_NEXT (TOP_GOING_OUT2, robot_move_failure);
}

FSM_TRANS_TIMEOUT (TOP_GOING_OUT2_BLOCK_WAIT, 250,
		   clamp_working, TOP_WAITING_CLAMP,
		   drop, TOP_GOING_TO_DROP,
		   element, TOP_GOING_TO_ELEMENT)
{
    if (clamp_working ())
	return FSM_NEXT_TIMEOUT (TOP_GOING_OUT2_BLOCK_WAIT, clamp_working);
    switch (top_decision ())
      {
      default: return FSM_NEXT_TIMEOUT (TOP_GOING_OUT2_BLOCK_WAIT, drop);
      case 1: return FSM_NEXT_TIMEOUT (TOP_GOING_OUT2_BLOCK_WAIT, element);
      }
}

FSM_TRANS (TOP_GOING_TO_DROP, move_success,
	   ready, TOP_DROP_DROPPING,
	   wait_clamp, TOP_WAITING_READY)
{
    if (logistic_global.ready)
      {
	if (clamp_drop (logistic_global.collect_direction))
	    return FSM_NEXT (TOP_GOING_TO_DROP, move_success, ready);
	else
	    return FSM_NEXT (TOP_GOING_TO_DROP, move_success, wait_clamp);
      }
    else
      {
	clamp_prepare (top_prepare_level ());
	return FSM_NEXT (TOP_GOING_TO_DROP, move_success, wait_clamp);
      }
}

FSM_TRANS (TOP_GOING_TO_DROP, move_failure,
	   clamp_working, TOP_WAITING_CLAMP,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    if (ctx.target_element_id != 0xff)
	element_failure (ctx.target_element_id);
    if (clamp_working ())
	return FSM_NEXT (TOP_GOING_TO_DROP, move_failure, clamp_working);
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_GOING_TO_DROP, move_failure, drop);
      case 1: return FSM_NEXT (TOP_GOING_TO_DROP, move_failure, element);
      }
}

FSM_TRANS (TOP_GOING_TO_DROP, clamp_working, TOP_WAITING_CLAMP)
{
    move_stop ();
    return FSM_NEXT (TOP_GOING_TO_DROP, clamp_working);
}

FSM_TRANS (TOP_GOING_TO_ELEMENT, move_success,
	   clamp_working, TOP_WAITING_CLAMP,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    if (ctx.target_element_id != 0xff)
	element_failure (ctx.target_element_id); /* Do not take this one again. */
    if (clamp_working ())
	return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_success, clamp_working);
    if (ctx.green_again)
      {
	ctx.green_again--;
	if (ctx.green_again == 0)
	    element_no_more_green ();
      }
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_success, drop);
      case 1: return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_success, element);
      }
}

FSM_TRANS (TOP_GOING_TO_ELEMENT, move_failure,
	   clamp_working, TOP_WAITING_CLAMP,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    if (ctx.target_element_id != 0xff)
	element_failure (ctx.target_element_id);
    if (clamp_working ())
	return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_failure, clamp_working);
    if (ctx.green_again)
      {
	ctx.green_again--;
	if (ctx.green_again == 0)
	    element_no_more_green ();
      }
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_failure, drop);
      case 1: return FSM_NEXT (TOP_GOING_TO_ELEMENT, move_failure, element);
      }
}

FSM_TRANS (TOP_GOING_TO_ELEMENT, clamp_working, TOP_WAITING_CLAMP)
{
    move_stop ();
    if (ctx.green_again)
      {
	ctx.green_again--;
	if (ctx.green_again == 0)
	    element_no_more_green ();
      }
    return FSM_NEXT (TOP_GOING_TO_ELEMENT, clamp_working);
}

FSM_TRANS (TOP_GOING_TO_ELEMENT, top_bumper, TOP_GOING_TO_ELEMENT)
{
    if (!ctx.broken)
	logistic_global.prepare = top_prepare_level ();
    move_stop ();
    ctx.target_element_id = 0xff;
    top_go_this_element (pawn_sensor_get_last_bumped (), BOT_ELEMENT_RADIUS - 50);
    return FSM_NEXT (TOP_GOING_TO_ELEMENT, top_bumper);
}

FSM_TRANS (TOP_WAITING_CLAMP, clamp_done,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_WAITING_CLAMP, clamp_done, drop);
      case 1: return FSM_NEXT (TOP_WAITING_CLAMP, clamp_done, element);
      }
}

FSM_TRANS (TOP_WAITING_CLAMP, clamp_blocked, TOP_UNBLOCKING_SHAKE_WAIT)
{
    return FSM_NEXT (TOP_WAITING_CLAMP, clamp_blocked);
}

FSM_TRANS (TOP_WAITING_CLAMP, clamp_taken, TOP_WAITING_CLAMP)
{
    position_t robot_pos;
    asserv_get_position (&robot_pos);
    if (robot_pos.v.x < 400 || robot_pos.v.x > PG_WIDTH - 400)
	asserv_move_linearly (ctx.go_to_element_direction
			      == DIRECTION_FORWARD ? -50 : 50);
    return FSM_NEXT (TOP_WAITING_CLAMP, clamp_taken);
}

FSM_TRANS_TIMEOUT (TOP_UNBLOCKING_SHAKE_WAIT, 250,
		   try_again, TOP_UNBLOCKING_SHAKE,
		   tryout, TOP_WAITING_CLAMP)
{
    if (ctx.chaos < 4)
      {
	int16_t dist = logistic_global.collect_direction
	    == DIRECTION_FORWARD ? 100 : -100;
	asserv_move_linearly (++ctx.chaos % 2 ? -dist : dist);
	return FSM_NEXT_TIMEOUT (TOP_UNBLOCKING_SHAKE_WAIT, try_again);
      }
    else
      {
	ctx.broken = 1;
	clamp_prepare (3);
	return FSM_NEXT_TIMEOUT (TOP_UNBLOCKING_SHAKE_WAIT, tryout);
      }
}

FSM_TRANS (TOP_UNBLOCKING_SHAKE, robot_move_success, TOP_WAITING_CLAMP)
{
    clamp_prepare (0);
    return FSM_NEXT (TOP_UNBLOCKING_SHAKE, robot_move_success);
}

FSM_TRANS (TOP_UNBLOCKING_SHAKE, robot_move_failure,
	   TOP_UNBLOCKING_SHAKE_WAIT)
{
    return FSM_NEXT (TOP_UNBLOCKING_SHAKE, robot_move_failure);
}

FSM_TRANS (TOP_WAITING_READY, clamp_done,
	   drop, TOP_DROP_DROPPING,
	   not_ready, TOP_WAITING_READY)
{
    if (clamp_drop (logistic_global.collect_direction))
	return FSM_NEXT (TOP_WAITING_READY, clamp_done, drop);
    else
	return FSM_NEXT (TOP_WAITING_READY, clamp_done, not_ready);
}

FSM_TRANS (TOP_WAITING_READY, clamp_blocked, TOP_UNBLOCKING_SHAKE_WAIT)
{
    return FSM_NEXT (TOP_WAITING_CLAMP, clamp_blocked);
}

FSM_TRANS (TOP_DROP_DROPPING, clamp_drop_waiting, TOP_DROP_CLEARING)
{
    if (ctx.target_element_id != 0xff)
	element_down (ctx.target_element_id,
		      logistic_drop_element_type
		      (logistic_global.collect_direction));
    if (IO_GET (CONTACT_STRAT))
	element_i_like_green ();
    asserv_move_linearly (logistic_global.collect_direction
			  == DIRECTION_FORWARD ? 150 : -150);
    return FSM_NEXT (TOP_DROP_DROPPING, clamp_drop_waiting);
}

FSM_TRANS (TOP_DROP_CLEARING, robot_move_success,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    clamp_drop_clear ();
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_DROP_CLEARING, robot_move_success, drop);
      case 1: return FSM_NEXT (TOP_DROP_CLEARING, robot_move_success, element);
      }
}

FSM_TRANS (TOP_DROP_CLEARING, robot_move_failure,
	   drop, TOP_GOING_TO_DROP,
	   element, TOP_GOING_TO_ELEMENT)
{
    clamp_drop_clear ();
    switch (top_decision ())
      {
      default: return FSM_NEXT (TOP_DROP_CLEARING, robot_move_failure, drop);
      case 1: return FSM_NEXT (TOP_DROP_CLEARING, robot_move_failure, element);
      }
}

