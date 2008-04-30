/* move_cb.c */
/*  {{{
 *
 * Copyright (C) 2008 Nélio Laranjeiro
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
#include "move_cb.h"

#include "path.h"
#include "asserv.h"
#include "playground.h"
#include "move.h"

#include "main.h"      /* main_post_event_for_top_fsm */
#include "modules/math/fixed/fixed.h"	/* fixed_* */

/**
 * The real radius of the obstacle.
 */
#define MOVE_REAL_OBSTACLE_RADIUS 150

/**
 * The sharp distance between the bot and the obstacle.
 */
#define MOVE_SHARP_DISTANCE 300

/**
 * The distance between the axis of the bot and the front sharp.
 */
#define MOVE_AXIS_FRONT_SHARP 150

/**
 * The standard distance of the obstacle.
 */
#define MOVE_OBSTACLE_DISTANCE \
    (MOVE_REAL_OBSTACLE_RADIUS + MOVE_SHARP_DISTANCE + MOVE_AXIS_FRONT_SHARP)

/**
 * The radius of the obstacle for the path module.
 * It corresponds to the real radius of the obstacle plus the distance you
 * want to add to avoid it.
 */
#define MOVE_OBSTACLE_RADIUS (MOVE_REAL_OBSTACLE_RADIUS + 250)

/**
 * The generic validity time (in term of number of cyles).
 */
#define MOVE_OBSTACLE_VALIDITY (5 * 225)

/**
 * Cycles count to ignore sharp event in the main loop.
 */
#define MOVE_MAIN_IGNORE_SHARP_EVENT (3 * 225)

/**
 * Easier function to get the next intermediate positon from the path module.
 * @param dst new destination position computed by the path module
 */
void
move_get_next_position (move_position_t *dst)
{
    /* Get the current position */
    asserv_position_t current_pos;
    asserv_get_position (&current_pos);
    /* Give the current position of the bot to the path module */
    path_endpoints (current_pos.x, current_pos.y,
		    move_data.final.x, move_data.final.y);
    /* Update the path module */
    path_update ();

    /* Retrieve next path coordinate */
    if (!path_get_next (&dst->x, &dst->y))
      {
	/* If it failed, try original destination */
	dst->x = move_data.final.x;
	dst->y = move_data.final.y;
      }
    main_sharp_ignore_event = MOVE_MAIN_IGNORE_SHARP_EVENT;
}

/**
 * Compute the obstacle position assuming it is right in front of us.
 * @param cur current position
 * @param obstacle the obstacle position computed
 */
void
move_compute_obstacle_position (asserv_position_t cur,
				move_position_t *obstacle)
{
    /* Convert the angle */
    uint32_t angle = cur.a;
    angle = angle << 8;
    /* X */
    obstacle->x = cur.x + fixed_mul_f824 (fixed_cos_f824 (angle),
					  MOVE_OBSTACLE_DISTANCE);
    /* Y */
    obstacle->y = cur.y + fixed_mul_f824 (fixed_sin_f824 (angle),
					  MOVE_OBSTACLE_DISTANCE);
}
/**
 * Unique function to compute the obstacle position from here.
 */
void
move_obstacle_here (void)
{
    /* Get the current position of the bot */
    asserv_position_t current;
    asserv_get_position (&current);
    /* Compute the obstacle position */
    move_compute_obstacle_position (current, &move_data.obstacle);
    /* Give it to the path module */
    path_obstacle (0, move_data.obstacle.x, move_data.obstacle.y,
		   MOVE_OBSTACLE_RADIUS, MOVE_OBSTACLE_VALIDITY);
}

/**
 * Unique function after moving backward to have unique code.
 */
void
move_after_moving_backward (void)
{
    /* Give the current position of the bot to the path module */
    move_get_next_position (&move_data.intermediate);
    /* Go to the next position */
    asserv_goto (move_data.intermediate.x, move_data.intermediate.y);
}

/*
 * IDLE =start=>
 *  => MOVING_TO_FINAL_POSITION
 *   ask the asserv to go to the final position
 */
fsm_branch_t
move__IDLE__start (void)
{
    /* Go to the destination position */
    asserv_goto (move_data.final.x, move_data.final.y);
    return move_next (IDLE, start);
}

/*
 * MOVING_TO_INTERMEDIATE_POSITION =bot_move_succeed=>
 * final_position => IDLE
 *   post an event for the top FSM to tell we have finished
 * position_intermediary => MOVING_TO_INTERMEDIATE_POSITION
 *   go to the next intermediate position computed by the path module
 */
fsm_branch_t
move__MOVING_TO_INTERMEDIATE_POSITION__bot_move_succeed (void)
{
    if ((move_data.final.x == move_data.intermediate.x) &&
	(move_data.final.y == move_data.intermediate.y))
      {
	/* Post an event for the top FSM to tell we have finished */
	main_post_event_for_top_fsm = TOP_EVENT_move_fsm_finished;
	return move_next_branch (MOVING_TO_INTERMEDIATE_POSITION, bot_move_succeed, final_position);
      }
    else
      {
	/* Get next position */
	move_get_next_position (&move_data.intermediate);
	/* Go to the next intermediate position */
	asserv_goto (move_data.intermediate.x, move_data.intermediate.y);
	return move_next_branch (MOVING_TO_INTERMEDIATE_POSITION, bot_move_succeed, position_intermediary);
      }
}

/*
 * MOVING_TO_INTERMEDIATE_POSITION =bot_move_obstacle=>
 *  => MOVING_TO_INTERMEDIATE_POSITION
 *   compute the obstacle position
 *   give the obstacle position to the path module
 *   go to the next intermediate position computed by the path module
 */
fsm_branch_t
move__MOVING_TO_INTERMEDIATE_POSITION__bot_move_obstacle (void)
{
    /* Compute obstacle position */
    move_obstacle_here ();
    /* Get next position */
    move_get_next_position (&move_data.intermediate);
    /* Go to the next intermediate position */
    asserv_goto (move_data.intermediate.x, move_data.intermediate.y);
    return move_next (MOVING_TO_INTERMEDIATE_POSITION, bot_move_obstacle);
}

/*
 * MOVING_TO_INTERMEDIATE_POSITION =bot_move_failed=>
 *  => MOVING_BACKWARD
 *   store the current position of the obstacle
 *   move backward to turn freely
 */
fsm_branch_t
move__MOVING_TO_INTERMEDIATE_POSITION__bot_move_failed (void)
{
    /* Compute obstacle position */
    move_obstacle_here ();
    /* Go backward */
    asserv_move_linearly (-PG_MOVE_DISTANCE); 
    return move_next (MOVING_TO_INTERMEDIATE_POSITION, bot_move_failed);
}

/*
 * MOVING_BACKWARD =bot_move_failed=>
 *  => MOVING_TO_INTERMEDIATE_POSITION
 *   do the same as when we succeed
 *   give the obstacle position to the path module
 *   give the current position of the bot to the path module
 *   get the intermediate position from path module
 */
fsm_branch_t
move__MOVING_BACKWARD__bot_move_failed (void)
{
    /* Call generic function */
    move_after_moving_backward ();
    return move_next (MOVING_BACKWARD, bot_move_failed);
}

/*
 * MOVING_BACKWARD =bot_move_succeed=>
 *  => MOVING_TO_INTERMEDIATE_POSITION
 *   give the obstacle position to the path module
 *   give the current position of the bot to the path module
 *   get the intermediate position from path module
 */
fsm_branch_t
move__MOVING_BACKWARD__bot_move_succeed (void)
{
    /* Call generic function */
    move_after_moving_backward ();
    return move_next (MOVING_BACKWARD, bot_move_succeed);
}

/*
 * MOVING_TO_FINAL_POSITION =bot_move_failed=>
 *  => MOVING_BACKWARD
 *   store the current position of the obstacle
 *   move backward to turn freely
 */
fsm_branch_t
move__MOVING_TO_FINAL_POSITION__bot_move_failed (void)
{
    /* Compute obstacle position */
    move_obstacle_here ();
    /* Move backward to turn freely */
    asserv_move_linearly (-PG_MOVE_DISTANCE);
    return move_next (MOVING_TO_FINAL_POSITION, bot_move_failed);
}

/*
 * MOVING_TO_FINAL_POSITION =bot_move_succeed=>
 *  => IDLE
 *   post an event for the top FSM to tell we have finished
 */
fsm_branch_t
move__MOVING_TO_FINAL_POSITION__bot_move_succeed (void)
{
    /* Post an event for the top FSM to tell we have finished */
    main_post_event_for_top_fsm = TOP_EVENT_move_fsm_finished;
    return move_next (MOVING_TO_FINAL_POSITION, bot_move_succeed);
}

/*
 * MOVING_TO_FINAL_POSITION =bot_move_obstacle=>
 *  => MOVING_TO_INTERMEDIATE_POSITION
 *   compute the obstacle position
 *   give the obstacle position to the path module
 *   give the current position of the bot to the path module
 *   get the intermediate position from path module
 */
fsm_branch_t
move__MOVING_TO_FINAL_POSITION__bot_move_obstacle (void)
{
    /* Compute obstacle position */
    move_obstacle_here ();
    /* Get next position */
    move_get_next_position (&move_data.intermediate);
    /* Go to the next intermediate position */
    asserv_goto (move_data.intermediate.x, move_data.intermediate.y);
    return move_next (MOVING_TO_FINAL_POSITION, bot_move_obstacle);
}

