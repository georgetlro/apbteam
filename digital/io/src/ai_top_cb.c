/* ai_top_cb.c */
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
 * }}}
 * Main FSM calling other FSM.
 */
#include "common.h"
#include "fsm.h"
#include "ai_cb.h"
#include "top.h"
#include "move.h"
#include "chrono.h"
#include "playground.h"
#include "asserv.h"
#include "loader.h"

/**
 * Time limit.
 */
#define TOP_TIME_LIMIT 25000

/*
 * IDLE =start=>
 *  => WAIT_INIT_TO_FINISH
 *   nothing to do.
 */
fsm_branch_t
ai__IDLE__start (void)
{
    return ai_next (IDLE, start);
}

/*
 * WAIT_INIT_TO_FINISH =init_match_is_started=>
 *  => FIRST_GO_BEGIN_OF_LINE_FAST
 *   the match start
 *   set fast speed
 *   go to first sequence begin of line
 */
fsm_branch_t
ai__WAIT_INIT_TO_FINISH__init_match_is_started (void)
{
    asserv_set_speed (BOT_MOVE_FAST);
    move_start_noangle (PG_VECT (375, 1503), 0);
    return ai_next (WAIT_INIT_TO_FINISH, init_match_is_started);
}

/*
 * FIRST_GO_BEGIN_OF_LINE_FAST =move_fsm_succeed=>
 *  => FIRST_GO_END_OF_LINE_FAST
 *   go to end of line
 *   loader down
 */
fsm_branch_t
ai__FIRST_GO_BEGIN_OF_LINE_FAST__move_fsm_succeed (void)
{
    move_start_noangle (PG_VECT (2625, 253), 0);
    loader_down ();
    return ai_next (FIRST_GO_BEGIN_OF_LINE_FAST, move_fsm_succeed);
}

/*
 * FIRST_GO_BEGIN_OF_LINE_FAST =move_fsm_failed=>
 *  => FIRST_GO_BEGIN_OF_LINE_FAST
 *   set slow speed
 *   retry
 */
fsm_branch_t
ai__FIRST_GO_BEGIN_OF_LINE_FAST__move_fsm_failed (void)
{
    asserv_set_speed (BOT_MOVE_SLOW);
    move_start_noangle (PG_VECT (375, 1503), 0);
    return ai_next (FIRST_GO_BEGIN_OF_LINE_FAST, move_fsm_failed);
}

/*
 * FIRST_GO_END_OF_LINE_FAST =in_field=>
 *  => FIRST_GO_END_OF_LINE_SLOW
 *   set slow speed
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_FAST__in_field (void)
{
    asserv_set_speed (BOT_MOVE_SLOW);
    return ai_next (FIRST_GO_END_OF_LINE_FAST, in_field);
}

/*
 * FIRST_GO_END_OF_LINE_FAST =move_fsm_succeed=>
 *  => UNLOAD_LOADER_UP
 *   set slow speed
 *   move loader up
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_FAST__move_fsm_succeed (void)
{
    asserv_set_speed (BOT_MOVE_SLOW);
    ai__FIRST_GO_END_OF_LINE_SLOW__move_fsm_succeed ();
    return ai_next (FIRST_GO_END_OF_LINE_FAST, move_fsm_succeed);
}

/*
 * FIRST_GO_END_OF_LINE_FAST =move_fsm_failed=>
 *  => FIRST_GO_END_OF_LINE_SLOW
 *   set slow speed
 *   retry
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_FAST__move_fsm_failed (void)
{
    asserv_set_speed (BOT_MOVE_SLOW);
    ai__FIRST_GO_END_OF_LINE_SLOW__move_fsm_failed ();
    return ai_next (FIRST_GO_END_OF_LINE_FAST, move_fsm_failed);
}

/*
 * FIRST_GO_END_OF_LINE_FAST =loader_errored=>
 *  => FIRST_GO_END_OF_LINE_UNBLOCKING_UP
 *   same as below
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_FAST__loader_errored (void)
{
    asserv_set_speed (BOT_MOVE_SLOW);
    ai__FIRST_GO_END_OF_LINE_SLOW__loader_errored ();
    return ai_next (FIRST_GO_END_OF_LINE_FAST, loader_errored);
}

/*
 * FIRST_GO_END_OF_LINE_SLOW =move_fsm_succeed=>
 *  => UNLOAD_LOADER_UP
 *   move loader up
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_SLOW__move_fsm_succeed (void)
{
    loader_up ();
    return ai_next (FIRST_GO_END_OF_LINE_SLOW, move_fsm_succeed);
}

/*
 * FIRST_GO_END_OF_LINE_SLOW =move_fsm_failed=>
 *  => FIRST_GO_END_OF_LINE_SLOW
 *   retry
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_SLOW__move_fsm_failed (void)
{
    move_start_noangle (PG_VECT (2625, 253), 0);
    return ai_next (FIRST_GO_END_OF_LINE_SLOW, move_fsm_failed);
}

/*
 * FIRST_GO_END_OF_LINE_SLOW =loader_errored=>
 *  => FIRST_GO_END_OF_LINE_UNBLOCKING_UP
 *   move backward
 *   loader up
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_SLOW__loader_errored (void)
{
    asserv_move_linearly (-90);
    loader_up ();
    return ai_next (FIRST_GO_END_OF_LINE_SLOW, loader_errored);
}

/*
 * FIRST_GO_END_OF_LINE_UNBLOCKING_UP =loader_uped=>
 *  => FIRST_GO_END_OF_LINE_UNBLOCKING
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_UNBLOCKING_UP__loader_uped (void)
{
    return ai_next (FIRST_GO_END_OF_LINE_UNBLOCKING_UP, loader_uped);
}

/*
 * FIRST_GO_END_OF_LINE_UNBLOCKING =bot_move_succeed=>
 *  => FIRST_GO_END_OF_LINE_SLOW
 *   move loader down
 *   retry
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_UNBLOCKING__bot_move_succeed (void)
{
    move_start_noangle (PG_VECT (2625, 253), 0);
    loader_down ();
    return ai_next (FIRST_GO_END_OF_LINE_UNBLOCKING, bot_move_succeed);
}

/*
 * FIRST_GO_END_OF_LINE_UNBLOCKING =bot_move_failed=>
 *  => FIRST_GO_END_OF_LINE_SLOW
 *   move loader down
 *   retry
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_UNBLOCKING__bot_move_failed (void)
{
    ai__FIRST_GO_END_OF_LINE_UNBLOCKING__bot_move_succeed ();
    return ai_next (FIRST_GO_END_OF_LINE_UNBLOCKING, bot_move_failed);
}

/*
 * FIRST_GO_END_OF_LINE_UNBLOCKING =loader_errored=>
 *  => FIRST_GO_END_OF_LINE_UNBLOCKING
 *   unblock again
 */
fsm_branch_t
ai__FIRST_GO_END_OF_LINE_UNBLOCKING__loader_errored (void)
{
    asserv_move_linearly (-90);
    return ai_next (FIRST_GO_END_OF_LINE_UNBLOCKING, loader_errored);
}

/*
 * UNLOAD =move_fsm_succeed=>
 *  => UNLOAD_LOADER_UP
 *   move loader up
 */
fsm_branch_t
ai__UNLOAD__move_fsm_succeed (void)
{
    loader_up ();
    return ai_next (UNLOAD, move_fsm_succeed);
}

/*
 * UNLOAD =move_fsm_failed=>
 *  => UNLOAD
 *   retry
 */
fsm_branch_t
ai__UNLOAD__move_fsm_failed (void)
{
    move_start_noangle (PG_VECT (2625, 253), 0);
    return ai_next (UNLOAD, move_fsm_failed);
}

/*
 * UNLOAD_LOADER_UP =loader_uped=>
 *  => UNLOAD_FACE_BIN
 *   turn toward bin
 */
fsm_branch_t
ai__UNLOAD_LOADER_UP__loader_uped (void)
{
    asserv_goto_angle (PG_A_DEG (90));
    return ai_next (UNLOAD_LOADER_UP, loader_uped);
}

/*
 * UNLOAD_FACE_BIN =bot_move_succeed=>
 *  => UNLOAD_BACK_BIN
 *   go backward to bin
 */
fsm_branch_t
ai__UNLOAD_FACE_BIN__bot_move_succeed (void)
{
    asserv_move_linearly (-(128 + 250 / 2 - BOT_SIZE_BACK - 50));
    return ai_next (UNLOAD_FACE_BIN, bot_move_succeed);
}

/*
 * UNLOAD_BACK_BIN =bot_move_succeed=>
 *  => UNLOAD_UNLOAD
 *   unload
 */
fsm_branch_t
ai__UNLOAD_BACK_BIN__bot_move_succeed (void)
{
    asserv_move_motor1_absolute (0, BOT_GATE_SPEED);
    return ai_next (UNLOAD_BACK_BIN, bot_move_succeed);
}

/*
 * UNLOAD_BACK_BIN =bot_move_failed=>
 *  => UNLOAD_UNLOAD
 *   unload
 */
fsm_branch_t
ai__UNLOAD_BACK_BIN__bot_move_failed (void)
{
    ai__UNLOAD_BACK_BIN__bot_move_succeed ();
    return ai_next (UNLOAD_BACK_BIN, bot_move_failed);
}

/*
 * UNLOAD_UNLOAD =state_timeout=>
 *  => HACK_MATCH1
 *   close gate
 *   move to hack position
 */
fsm_branch_t
ai__UNLOAD_UNLOAD__state_timeout (void)
{
    loader_elements = 0;
    asserv_move_motor1_absolute (BOT_GATE_STROKE_STEP, BOT_GATE_SPEED);
    position_t pos = PG_POSITION_DEG (2400, 378, 29);
    move_start (pos, 0);
    return ai_next (UNLOAD_UNLOAD, state_timeout);
}

/*
 * COLLECT =move_fsm_succeed=>
 * unload => UNLOAD
 * collect => COLLECT
 */
fsm_branch_t
ai__COLLECT__move_fsm_succeed (void)
{
    if (top_collect (0))
	return ai_next_branch (COLLECT, move_fsm_succeed, collect);
    else
	return ai_next_branch (COLLECT, move_fsm_succeed, unload);
}

/*
 * COLLECT =move_fsm_failed=>
 * unload => UNLOAD
 * collect => COLLECT
 */
fsm_branch_t
ai__COLLECT__move_fsm_failed (void)
{
    if (top_collect (0))
	return ai_next_branch (COLLECT, move_fsm_failed, collect);
    else
	return ai_next_branch (COLLECT, move_fsm_failed, unload);
}

/*
 * HACK_MATCH1 =move_fsm_succeed=>
 *  => COLLECT
 *   loader down
 *   choose best food to collect
 */
fsm_branch_t
ai__HACK_MATCH1__move_fsm_succeed (void)
{
    loader_down ();
    top_collect (1);
    loader_choucroute = 1;
    return ai_next (HACK_MATCH1, move_fsm_succeed);
}

