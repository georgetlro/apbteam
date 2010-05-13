/* ai_loader_cb.c */
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
#include "common.h"
#include "fsm.h"
#include "ai_cb.h"
#include "asserv.h"
#include "mimot.h"
#include "bot.h"
#include "main.h"
#include "food.h"
#include "loader.h"

/*
 * LOADER_IDLE =start=>
 *  => LOADER_WAIT_JACK_IN
 */
fsm_branch_t
ai__LOADER_IDLE__start (void)
{
    return ai_next (LOADER_IDLE, start);
}

/*
 * LOADER_WAIT_JACK_IN =jack_inserted_into_bot=>
 *  => LOADER_WAIT_JACK_OUT
 */
fsm_branch_t
ai__LOADER_WAIT_JACK_IN__jack_inserted_into_bot (void)
{
    return ai_next (LOADER_WAIT_JACK_IN, jack_inserted_into_bot);
}

/*
 * LOADER_WAIT_JACK_OUT =jack_removed_from_bot=>
 *  => LOADER_INIT_ELEVATOR_ZERO
 *   find elevator zero
 *   close clamp
 *   find gate zero
 */
fsm_branch_t
ai__LOADER_WAIT_JACK_OUT__jack_removed_from_bot (void)
{
    asserv_motor0_zero_position (-BOT_ELEVATOR_ZERO_SPEED);
    mimot_motor0_clamp (BOT_CLAMP_ZERO_SPEED, 0);
    mimot_motor1_clamp (BOT_CLAMP_ZERO_SPEED, 0);
    asserv_motor1_zero_position (-BOT_GATE_SPEED);
    return ai_next (LOADER_WAIT_JACK_OUT, jack_removed_from_bot);
}

/*
 * LOADER_INIT_ELEVATOR_ZERO =elevator_succeed=>
 *  => LOADER_INIT_CLAMP_CLOSE
 */
fsm_branch_t
ai__LOADER_INIT_ELEVATOR_ZERO__elevator_succeed (void)
{
    return ai_next (LOADER_INIT_ELEVATOR_ZERO, elevator_succeed);
}

/*
 * LOADER_INIT_CLAMP_CLOSE =clamp_succeed=>
 *  => LOADER_INIT_CLAMP_ZERO
 *   move elevator up
 *   find clamp zero
 */
fsm_branch_t
ai__LOADER_INIT_CLAMP_CLOSE__clamp_succeed (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_REST_STEP,
				 BOT_ELEVATOR_ZERO_SPEED);
    mimot_motor0_zero_position (-BOT_CLAMP_ZERO_SPEED);
    mimot_motor1_zero_position (-BOT_CLAMP_ZERO_SPEED);
    return ai_next (LOADER_INIT_CLAMP_CLOSE, clamp_succeed);
}

/*
 * LOADER_INIT_CLAMP_ZERO =clamp_succeed=>
 *  => LOADER_INIT_GATE_ZERO
 */
fsm_branch_t
ai__LOADER_INIT_CLAMP_ZERO__clamp_succeed (void)
{
    return ai_next (LOADER_INIT_CLAMP_ZERO, clamp_succeed);
}

/*
 * LOADER_INIT_GATE_ZERO =gate_succeed=>
 *  => LOADER_INIT_ELEVATOR_UP
 */
fsm_branch_t
ai__LOADER_INIT_GATE_ZERO__gate_succeed (void)
{
    return ai_next (LOADER_INIT_GATE_ZERO, gate_succeed);
}

/*
 * LOADER_INIT_ELEVATOR_UP =elevator_succeed=>
 *  => LOADER_INIT_GATE_WAIT
 */
fsm_branch_t
ai__LOADER_INIT_ELEVATOR_UP__elevator_succeed (void)
{
    return ai_next (LOADER_INIT_ELEVATOR_UP, elevator_succeed);
}

/*
 * LOADER_INIT_ELEVATOR_UP =elevator_failed=>
 *  => LOADER_IDLE
 *   initialisation failure
 */
fsm_branch_t
ai__LOADER_INIT_ELEVATOR_UP__elevator_failed (void)
{
    /* Move so that the operator notice it. */
    asserv_move_motor0_absolute (BOT_ELEVATOR_STROKE_STEP / 3,
				 BOT_ELEVATOR_ZERO_SPEED);
    return ai_next (LOADER_INIT_ELEVATOR_UP, elevator_failed);
}

/*
 * LOADER_INIT_GATE_WAIT =state_timeout=>
 *  => LOADER_INIT_GATE_CLOSE
 *   close gate
 */
fsm_branch_t
ai__LOADER_INIT_GATE_WAIT__state_timeout (void)
{
    asserv_move_motor1_absolute (BOT_GATE_STROKE_STEP, BOT_GATE_SPEED);
    return ai_next (LOADER_INIT_GATE_WAIT, state_timeout);
}

/*
 * LOADER_INIT_GATE_CLOSE =gate_succeed=>
 *  => LOADER_UP
 */
fsm_branch_t
ai__LOADER_INIT_GATE_CLOSE__gate_succeed (void)
{
    return ai_next (LOADER_INIT_GATE_CLOSE, gate_succeed);
}

/*
 * LOADER_UP =loader_down=>
 *  => LOADER_DOWNING
 *   move down
 */
fsm_branch_t
ai__LOADER_UP__loader_down (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_DOWN_STEP, BOT_ELEVATOR_SPEED);
    return ai_next (LOADER_UP, loader_down);
}

/*
 * LOADER_DOWN =loader_up=>
 *  => LOADER_UPING
 *   move up
 */
fsm_branch_t
ai__LOADER_DOWN__loader_up (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_REST_STEP, BOT_ELEVATOR_SPEED);
    return ai_next (LOADER_DOWN, loader_up);
}

/*
 * LOADER_DOWN =loader_element=>
 *  => LOADER_LOAD_CLOSING
 *   clamp
 */
fsm_branch_t
ai__LOADER_DOWN__loader_element (void)
{
    mimot_motor0_clamp (BOT_CLAMP_SPEED, BOT_CLAMP_PWM);
    mimot_motor1_clamp (BOT_CLAMP_SPEED, BOT_CLAMP_PWM);
    return ai_next (LOADER_DOWN, loader_element);
}

/*
 * LOADER_UPING =elevator_succeed=>
 *  => LOADER_UP
 *   post loader_uped event
 */
fsm_branch_t
ai__LOADER_UPING__elevator_succeed (void)
{
    main_post_event (AI_EVENT_loader_uped);
    return ai_next (LOADER_UPING, elevator_succeed);
}

/*
 * LOADER_UPING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_UPING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_UPING, elevator_failed);
}

/*
 * LOADER_DOWNING =elevator_succeed=>
 *  => LOADER_DOWN
 *   release elevator motor
 *   post loader_downed event
 */
fsm_branch_t
ai__LOADER_DOWNING__elevator_succeed (void)
{
    asserv_motor0_free ();
    main_post_event (AI_EVENT_loader_downed);
    return ai_next (LOADER_DOWNING, elevator_succeed);
}

/*
 * LOADER_DOWNING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_DOWNING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_DOWNING, elevator_failed);
}

/*
 * LOADER_ERROR =loader_down=>
 *  => LOADER_ERROR_DOWNING
 *   move down
 */
fsm_branch_t
ai__LOADER_ERROR__loader_down (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_DOWN_STEP, BOT_ELEVATOR_SPEED);
    return ai_next (LOADER_ERROR, loader_down);
}

/*
 * LOADER_ERROR =loader_up=>
 *  => LOADER_ERROR_UPING
 *   move up
 */
fsm_branch_t
ai__LOADER_ERROR__loader_up (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_REST_STEP, BOT_ELEVATOR_SPEED);
    return ai_next (LOADER_ERROR, loader_up);
}

/*
 * LOADER_ERROR_DOWNING =elevator_succeed=>
 *  => LOADER_ERROR_DOWNING_OPEN
 *   release elevator motor
 *   open clamp
 */
fsm_branch_t
ai__LOADER_ERROR_DOWNING__elevator_succeed (void)
{
    asserv_motor0_free ();
    mimot_move_motor0_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    mimot_move_motor1_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    return ai_next (LOADER_ERROR_DOWNING, elevator_succeed);
}

/*
 * LOADER_ERROR_DOWNING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_ERROR_DOWNING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_ERROR_DOWNING, elevator_failed);
}

/*
 * LOADER_ERROR_DOWNING_OPEN =clamp_succeed=>
 *  => LOADER_DOWN
 *   post loader_downed event
 */
fsm_branch_t
ai__LOADER_ERROR_DOWNING_OPEN__clamp_succeed (void)
{
    main_post_event (AI_EVENT_loader_downed);
    return ai_next (LOADER_ERROR_DOWNING_OPEN, clamp_succeed);
}

/*
 * LOADER_ERROR_UPING =elevator_succeed=>
 *  => LOADER_ERROR_UPING_OPEN
 *   open clamp
 */
fsm_branch_t
ai__LOADER_ERROR_UPING__elevator_succeed (void)
{
    mimot_move_motor0_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    mimot_move_motor1_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    return ai_next (LOADER_ERROR_UPING, elevator_succeed);
}

/*
 * LOADER_ERROR_UPING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_ERROR_UPING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_ERROR_UPING, elevator_failed);
}

/*
 * LOADER_ERROR_UPING_OPEN =clamp_succeed=>
 *  => LOADER_UP
 *   post loader_uped event
 */
fsm_branch_t
ai__LOADER_ERROR_UPING_OPEN__clamp_succeed (void)
{
    main_post_event (AI_EVENT_loader_uped);
    return ai_next (LOADER_ERROR_UPING_OPEN, clamp_succeed);
}

/*
 * LOADER_LOAD_CLOSING =clamp_succeed=>
 * full => LOADER_LOAD_UPING
 *   move up
 *   count one element
 * empty => LOADER_LOAD_EMPTY_OPEN
 *   open clamp
 */
fsm_branch_t
ai__LOADER_LOAD_CLOSING__clamp_succeed (void)
{
    /* Measure load using clamp position. */
    uint16_t tickness = BOT_CLAMP_WIDTH_STEP
	- mimot_get_motor0_position ()
	- mimot_get_motor1_position ();
    if (tickness > BOT_CLAMP_EMPTY_STEP)
      {
	asserv_move_motor0_absolute (BOT_ELEVATOR_STROKE_STEP,
				     BOT_ELEVATOR_SPEED);
	position_t robot_position;
	asserv_get_position (&robot_position);
	food_taken (robot_position);
	loader_elements++;
	return ai_next_branch (LOADER_LOAD_CLOSING, clamp_succeed, full);
      }
    else
      {
	mimot_move_motor0_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
	mimot_move_motor1_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
	return ai_next_branch (LOADER_LOAD_CLOSING, clamp_succeed, empty);
      }
}

/*
 * LOADER_LOAD_UPING =elevator_unload_position=>
 *  => LOADER_LOAD_UNLOADING
 *   open clamp
 */
fsm_branch_t
ai__LOADER_LOAD_UPING__elevator_unload_position (void)
{
    mimot_move_motor0_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    mimot_move_motor1_absolute (BOT_CLAMP_OPEN_STEP, BOT_CLAMP_SPEED);
    return ai_next (LOADER_LOAD_UPING, elevator_unload_position);
}

/*
 * LOADER_LOAD_UPING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_LOAD_UPING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_LOAD_UPING, elevator_failed);
}

/*
 * LOADER_LOAD_UNLOADING =elevator_succeed=>
 * choucroute => LOADER_HACK_MATCH1
 * merguez => LOADER_LOAD_UNLOADING_OPEN
 */
fsm_branch_t
ai__LOADER_LOAD_UNLOADING__elevator_succeed (void)
{
    if (loader_choucroute)
	return ai_next_branch (LOADER_LOAD_UNLOADING, elevator_succeed, choucroute);
    else
	return ai_next_branch (LOADER_LOAD_UNLOADING, elevator_succeed, merguez);
}

/*
 * LOADER_LOAD_UNLOADING =elevator_failed=>
 *  => LOADER_ERROR
 *   post loader_errored event
 */
fsm_branch_t
ai__LOADER_LOAD_UNLOADING__elevator_failed (void)
{
    main_post_event (AI_EVENT_loader_errored);
    return ai_next (LOADER_LOAD_UNLOADING, elevator_failed);
}

/*
 * LOADER_LOAD_UNLOADING_OPEN =clamp_succeed=>
 *  => LOADER_DOWNING
 *   move down
 */
fsm_branch_t
ai__LOADER_LOAD_UNLOADING_OPEN__clamp_succeed (void)
{
    asserv_move_motor0_absolute (BOT_ELEVATOR_DOWN_STEP, BOT_ELEVATOR_SPEED);
    return ai_next (LOADER_LOAD_UNLOADING_OPEN, clamp_succeed);
}

/*
 * LOADER_LOAD_EMPTY_OPEN =clamp_succeed=>
 *  => LOADER_DOWN
 */
fsm_branch_t
ai__LOADER_LOAD_EMPTY_OPEN__clamp_succeed (void)
{
    return ai_next (LOADER_LOAD_EMPTY_OPEN, clamp_succeed);
}

/*
 * LOADER_HACK_MATCH1 =state_timeout=>
 *  => LOADER_LOAD_UNLOADING_OPEN
 */
fsm_branch_t
ai__LOADER_HACK_MATCH1__state_timeout (void)
{
    return ai_next (LOADER_HACK_MATCH1, state_timeout);
}

