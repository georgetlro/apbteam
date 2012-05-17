/* bottom_clamp.c */
/* guybrush - Eurobot 2012 AI. {{{
 *
 * Copyright (C) 2012 Julien Balleyguier
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
#include "output.h"

#include "mimot.h"
#include "contact.h"
#include "bot.h"
#include "main.h"
#include "bottom_clamp.h"
#include "pressure.h"


#define FSM_NAME AI
#include "fsm.h"
#include "fsm_queue.h"

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

    /* ---------------------Initialisation sequence---------------------- */
    /*Opening all the clamps. */
    CLAMP_INIT_OPEN_ALL_CLAMPS,
    /*Closing all the clamps (except bottom clamp 2). */
    CLAMP_INIT_CLOSE_ALL_CLAMPS,
    /*Finding 0 position. */
    CLAMP_INIT_FIND_0,
    /* Hide the clamp in order not to go over the robot. */
    CLAMP_INIT_HIDE_CLAMP,
    /* Clamp ready, waiting in rest position. */
    CLAMP_INIT_READY,
    /* Returning to idle position. */
    CLAMP_GOING_IDLE,

    /*--------------------------IDLE---------------------------------------*/
    /*Wait before going idle*/
    CLAMP_WAIT_BEFORE_IDLE,

    /* Waiting external events. */
    CLAMP_IDLE,
    /*---------------------------Cleaning area sequence---------------------*/
    CLAMP_READY_TO_CLEAN,
    CLAMP_CATCH_COIN,
    CLAMP_READY_TO_LOAD, 
    /*---------------------------Taking coin sequence-----------------------*/
    /*Taking a coin. */
    CLAMP_TAKE_COIN,
    /*Turning half way. */
    CLAMP_TURN_HALF_WAY,
    /* Droping the cd. */
    CLAMP_DROP_CD,

    /*--------------------------Approaching Tree sequence------------------*/
    /*Hide bottom clamp to unfold the upper set. */
    CLAMP_BOTTOM_CLAMP_HIDE_POS,
    /*unfold the upper set. */
    CLAMP_UNFOLD_UPPER_SET,
    /*Putting the bottom clamp back in position. */
    CLAMP_BOTTOM_CLAMP_READY,
    /*All the clamps are now open and the robot is ready to empty the tree. */
    CLAMP_READY_TO_EMPTY_TREE,
    /*Close all clamps to empty the tree. */
    CLAMP_CLOSE_ALL_CLAMPS,
    /*Recentrage of the CD clamps. */
    CLAMP_REARRANGE_CD,
    /*Hiding bottom clamp to fold back the upper set. */
    CLAMP_BOTTOM_CLAMP_HIDE_POS2,
    /*Release the asserve not to block the clamp when going back up. */
    CLAMP_RELEASE_ASSERV,
    /*fold back the upper set. */
    CLAMP_FOLD_UPPER_SET,
    /*Open upper clamps to release objects. */
    CLAMP_OPEN_UPPER_CLAMPS,
    /*Make the coin "fall" into the clamp (the clamp has to be 90° from the floor*/
    CLAMP_READY_TO_RECALE,
    CLAMP_READY_TO_RECALE_2,
    CLAMP_TEMPO_RECALE,
    CLAMP_END_RECALE,

    /*-------------------------Stop tree approach extra states--------------*/
    /*Letting the bottom clamp to finish it's move before starting the stop tree procedure. */
    CLAMP_FINISH_BOTTOM_CLAMP_READY,
    /*Putting the bottom clamp back in position after a stop tree approach signal. */
    CLAMP_BOTTOM_CLAMP_BACK,

    /*------------------------ Clamp blocked------------------------------. */
    CLAMP_UPPER_BLOCKED,
    CLAMP_UPPER_BLOCKED_WAIT,
    CLAMP_UPPER_BLOCKED_UP,
    CLAMP_BLOCKED,
    CLAMP_OPEN_BOTTOM_CLAMPS,
    CLAMP_TURN_BACKWARD,
    CLAMP_WAIT,
    CLAMP_TURN_FORWARD,
    CLAMP_SHITTY_STATE
    )

FSM_EVENTS (
    /* O detected*/
    0_found,
    /* coin detected in the clamp */
    coin_detected,
    /*signal sent to the top fsm when taking a coin*/
    taking_coin,
    /*time to drop the cd. Sent by main*/
    time_to_drop_coin,
    /* Lower clamp rotation motor success. */
    lower_clamp_rotation_success,
    /* Lower clamp rotation motor failure. */
    lower_clamp_rotation_failure,
    /*upper set is totally open*/
    upper_set_up,
    /*upper set is totally close*/
    upper_set_down,
    /*Clean the area*/
    clean_start,
    /*Clos the bottom clamp*/
    clean_catch,
    /* Load the coin*/
    clean_load,
    /* Tree detected, get ready to empty it. */
    tree_detected,
    /* stop the tree-emptying cycle*/
    stop_tree_approach,
    /* signal sent when the robot is ready to empty the tree. the approach has been finished*/
    clamps_ready,
    /* the robot is ready to empty the tree, and the top fsm is asking him to do so*/
    empty_tree,
    /* The robot has step back. Event given by the main.c programm(after an empty tree, or a clamp_block event. */
    robot_is_back,
    /* The clamp is blocked. We can try to unblock it*/
    clamp_blocked,
    /* Try to unblock clamp. */
    clamp_unblock,
    /* We tryed to unblock the clamp too many time. we can now say that the bottom clamp is out of order*/
    clamp_is_dead
     )


FSM_START_WITH (CLAMP_START)

/*-------------------------------------
         TIMEOUT DEFINITION
---------------------------------------*/
#define TIMEOUT_OPEN_CLAMPS 40
#define TIMEOUT_CLOSE_CLAMPS 40
#define TIMEOUT_DROP_CD 100
#define TIMEOUT_FREE_ASSERV 100
#define TIMEOUT_RECENTRAGE 100
#define TIMEOUT_BLOCKED 100
#define TIMEOUT_IDLE 70

/*-------------------------------------
         ROTATION DEFINITION
---------------------------------------*/
#define POS_DELAY 1200
#define INIT_BOTTOM_POS (6.65+HALF_TURN)
#define HIDE_POS 3
#define BACK_TO_READY (16-HIDE_POS)
#define HALF_TURN 8
#define QUARTER_TURN 4
#define DECALAGE_CD_BAS 1.5
#define HIDE_POS_TREE 3
#define HIDE_POS_TREE_2 (2-DECALAGE_CD_BAS)
#define BACK_TO_READY_TREE (-HIDE_POS_TREE + DECALAGE_CD_BAS)
#define BACK_TO_READY_TREE_2 (HALF_TURN - HIDE_POS_TREE_2 - DECALAGE_CD_BAS)
#define RECALE_COIN (QUARTER_TURN - HIDE_POS_TREE_2 - DECALAGE_CD_BAS)
#define BACK_TO_READY_RECALE_COIN (-QUARTER_TURN)
#define FAST_ROTATION 0x30
#define MEDIUM_ROTATION 0x10
#define SLOW_ROTATION 0x01
#define LOW_PRESSURE 0x300
#define HIGH_PRESSURE 0x380
/*-------------------------------------
         Clamp context
---------------------------------------*/
struct clamp_t
{
    /** Current position. */
    int pos_current;
    /** Backup position in case of blocking. */
    int pos_backup;
    /* Clamp_1 is low, which means, it is ready to take a coin*/
    uint8_t clamp_1_down;
    /** True we are stopping the tree approach. */
    uint8_t stop_tree_approach;
    /** True if we are in "Calm mod" (clamps can't move)*/
    uint8_t calm_mode;   
    /** Compting the number of time the bottom clamp is being blocked in a row (reset when going back to  idle)*/
    uint8_t cpt_blocked;
    /** Request from top FSM to be processed once in IDLE state. */
    uint16_t idle_request;
};

/*Global context. */
struct clamp_t clamp_global;
#define ctx clamp_global


static void
clamp_blocked (void)
{
}

static void move_needed(int move,int speed)
{
    ctx.pos_current += move;
    mimot_move_motor0_absolute (ctx.pos_current, speed);
}

void clamp_calm_mode(int mode)
{
    ctx.calm_mode=mode;
}

uint8_t clamp_calm_mode_read(void)
{
    return ctx.calm_mode;
}

void
clamp_request (uint16_t event)
{
    /* If event can not be handled right now, keep it until going in IDLE
     * state. */
    if (!FSM_HANDLE_VAR (AI, event))
    {
        assert (!ctx.idle_request);
        ctx.idle_request = event;
    }
}

uint8_t clamp_read_blocked_cpt(void)
{
    return ctx.cpt_blocked;
}


/*---------------------------------------------------------*/
/*  INIT part of the FSM                                   */
/*---------------------------------------------------------*/

/* Init Bottom clamp. */
FSM_TRANS (CLAMP_START, init_actuators, CLAMP_INIT_OPEN_ALL_CLAMPS)
{
    /* Starting the pump */
    pressure_set(LOW_PRESSURE);
    /*Opening bottom Clamps*/
    IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    /*Opening upper clamps*/
    IO_SET (OUTPUT_UPPER_CLAMP_OPEN);
    
    return FSM_NEXT (CLAMP_START,init_actuators);
    
}

FSM_TRANS_TIMEOUT (CLAMP_INIT_OPEN_ALL_CLAMPS,TIMEOUT_OPEN_CLAMPS, 
           upper_set_in,CLAMP_INIT_CLOSE_ALL_CLAMPS,
           upper_set_out,CLAMP_SHITTY_STATE)
{
    /*if upper set is inside the robot*/
    if(IO_GET(CONTACT_UPPER_CLAMP_DOWN))
    {
        ctx.pos_current = 0;
        /* closing all the clamps except the clamp 1 to proceed the find_O function. */
        IO_SET (OUTPUT_LOWER_CLAMP_2_CLOSE);
        IO_CLR (OUTPUT_UPPER_CLAMP_OPEN);
        return FSM_NEXT_TIMEOUT (CLAMP_INIT_OPEN_ALL_CLAMPS,upper_set_in);
    }
    /*if it is outside, go to shitty state and stop everything*/
    else
    {
        return FSM_NEXT_TIMEOUT (CLAMP_INIT_OPEN_ALL_CLAMPS,upper_set_out);
    }
    

}

FSM_TRANS_TIMEOUT (CLAMP_INIT_CLOSE_ALL_CLAMPS, 5*TIMEOUT_OPEN_CLAMPS, CLAMP_INIT_FIND_0)
{
    /*Findig the 0 position. */
    move_needed(8000,SLOW_ROTATION);
    return FSM_NEXT_TIMEOUT (CLAMP_INIT_CLOSE_ALL_CLAMPS);
}

FSM_TRANS (CLAMP_INIT_FIND_0, 0_found, CLAMP_INIT_HIDE_CLAMP)
{
    /*init of the position*/
    ctx.pos_current = mimot_get_motor0_position();
    
    /* Hidding the clamp inside the robot. */
    move_needed(INIT_BOTTOM_POS  * 250,FAST_ROTATION);
    return FSM_NEXT (CLAMP_INIT_FIND_0, 0_found);
}

FSM_TRANS (CLAMP_INIT_HIDE_CLAMP, lower_clamp_rotation_success, 
           normal_start,CLAMP_INIT_READY,
           demo_start,CLAMP_GOING_IDLE)
{
    /*Opening the clamp 2. We do it know to save the batteries*/
    IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    /*Looking if we are in demo mode or normal mode*/
    if (!main_demo)
        return FSM_NEXT (CLAMP_INIT_HIDE_CLAMP,lower_clamp_rotation_success,normal_start);
    else
        return FSM_NEXT (CLAMP_INIT_HIDE_CLAMP,lower_clamp_rotation_success,demo_start);
}


FSM_TRANS (CLAMP_INIT_READY,init_start_round, CLAMP_GOING_IDLE)
{
    return FSM_NEXT (CLAMP_INIT_READY, init_start_round);
}

FSM_TRANS_TIMEOUT (CLAMP_GOING_IDLE, 3*TIMEOUT_OPEN_CLAMPS, CLAMP_WAIT_BEFORE_IDLE)
{
    /*Going back to the idle position, ready for showtime.*/
    ctx.cpt_blocked = 0;
    ctx.clamp_1_down = 0;
    /* Open upper clamp. */
    IO_SET (OUTPUT_UPPER_CLAMP_OPEN);
    /* recentrage the middle clamp. */
    IO_SET (OUTPUT_UPPER_CLAMP_OUT);
    IO_CLR (OUTPUT_UPPER_CLAMP_IN);
    /*Contrepression*/
    IO_SET (OUTPUT_UPPER_CLAMP_UP);
    return FSM_NEXT_TIMEOUT (CLAMP_GOING_IDLE);
}

/*---------------------------------------------------------*/
/*  IDLE!!                                                 */
/*---------------------------------------------------------*/
FSM_TRANS_TIMEOUT (CLAMP_WAIT_BEFORE_IDLE, TIMEOUT_IDLE,
                   request_clean_start, CLAMP_READY_TO_CLEAN,
		   request_tree_detected, CLAMP_BOTTOM_CLAMP_HIDE_POS,
		   idle, CLAMP_IDLE)
{
    if (ctx.idle_request)
    {
        uint16_t idle_request = ctx.idle_request;
        ctx.idle_request = 0;
        if (idle_request == FSM_EVENT (AI, clean_start))
        {
            fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
            return FSM_NEXT_TIMEOUT (CLAMP_WAIT_BEFORE_IDLE,
                                     request_clean_start);
        }
        else if (idle_request == FSM_EVENT (AI, tree_detected))
        {
            /* Hidding the clamp inside the robot. */
            if (ctx.clamp_1_down)
            {
                ctx.pos_backup = ctx.pos_current;
                move_needed (HIDE_POS_TREE * 250, FAST_ROTATION);
            }
            else
            {
                ctx.pos_backup = ctx.pos_current + HALF_TURN * 250;
                move_needed ((HALF_TURN + HIDE_POS_TREE) * 250, FAST_ROTATION);
                ctx.clamp_1_down = 1;
            }
            return FSM_NEXT_TIMEOUT (CLAMP_WAIT_BEFORE_IDLE,
                                     request_tree_detected);
        }
        else
        {
            assert (0);
            return FSM_NEXT_TIMEOUT (CLAMP_WAIT_BEFORE_IDLE, idle);
        }
    }
    else
    {
        /* Going back to the idle position, ready for showtime. */
        fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
        return FSM_NEXT_TIMEOUT (CLAMP_WAIT_BEFORE_IDLE, idle);
    }
}

/*---------------------------------------------------------*/
/*  parts of the FSM that Clean areas                      */
/*---------------------------------------------------------*/
FSM_TRANS (CLAMP_IDLE, clean_start,CLAMP_READY_TO_CLEAN) 
{
    fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
    return FSM_NEXT(CLAMP_IDLE,clean_start);
}

FSM_TRANS (CLAMP_READY_TO_CLEAN, clean_catch,CLAMP_CATCH_COIN) 
{
     /*If the clamp 1 has the CD.*/
    if (ctx.clamp_1_down)
    {     
        /*Close it.*/
        IO_SET (OUTPUT_LOWER_CLAMP_1_CLOSE);
    }
    /*If the clamp 2 is closed. */
    else       
    {
        IO_SET (OUTPUT_LOWER_CLAMP_2_CLOSE);
    }
    return FSM_NEXT(CLAMP_READY_TO_CLEAN,clean_catch);
}
FSM_TRANS_TIMEOUT (CLAMP_CATCH_COIN, TIMEOUT_CLOSE_CLAMPS*2,CLAMP_READY_TO_LOAD) 
{
    fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
    return FSM_NEXT_TIMEOUT(CLAMP_CATCH_COIN);
}

FSM_TRANS (CLAMP_READY_TO_LOAD, clean_load,CLAMP_TAKE_COIN) 
{
    return FSM_NEXT(CLAMP_READY_TO_LOAD,clean_load);
}



/*---------------------------------------------------------*/
/*  parts of the FSM that Takes coin                       */
/*---------------------------------------------------------*/
FSM_TRANS (CLAMP_IDLE, coin_detected,CLAMP_TAKE_COIN)
{
    if (ctx.clamp_1_down)
    {
        /*Close it.*/
        IO_SET (OUTPUT_LOWER_CLAMP_1_CLOSE);
    }
    /*If the clamp 2 is ready*/
    else             
    {
        IO_SET(OUTPUT_LOWER_CLAMP_2_CLOSE);
    }
    fsm_queue_post_event (FSM_EVENT (AI, taking_coin));
    return FSM_NEXT (CLAMP_IDLE, coin_detected);
}


FSM_TRANS_TIMEOUT (CLAMP_TAKE_COIN, TIMEOUT_CLOSE_CLAMPS, CLAMP_TURN_HALF_WAY)
{
    main_set_drop_coin_pos(ctx.pos_current + (HALF_TURN * 250) - POS_DELAY);
    move_needed(HALF_TURN * 250,FAST_ROTATION);
    ctx.clamp_1_down = !ctx.clamp_1_down;
    return FSM_NEXT_TIMEOUT (CLAMP_TAKE_COIN);
}

FSM_TRANS (CLAMP_TURN_HALF_WAY, time_to_drop_coin, CLAMP_DROP_CD)
{
    /*If the clamp 1 has the CD.*/
    if (!ctx.clamp_1_down)
    {     
        /*Open it.*/
        IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    }
    /*If the clamp 2 is closed. */
    else       
    {
        IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    }
    return FSM_NEXT (CLAMP_TURN_HALF_WAY,time_to_drop_coin);
}

FSM_TRANS (CLAMP_TURN_HALF_WAY, lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    return FSM_NEXT (CLAMP_TURN_HALF_WAY,lower_clamp_rotation_failure);
}

FSM_TRANS (CLAMP_DROP_CD,lower_clamp_rotation_success,CLAMP_WAIT_BEFORE_IDLE)
{
    return FSM_NEXT (CLAMP_DROP_CD,lower_clamp_rotation_success);
}

FSM_TRANS (CLAMP_DROP_CD, lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    /*The clamp is blocked somehow. */
    clamp_blocked();
    return FSM_NEXT (CLAMP_DROP_CD, lower_clamp_rotation_failure);
}
/*---------------------------------------------------------*/
/*  Parts of the FSM that is Approaching the tree (totem)  */
/*---------------------------------------------------------*/

FSM_TRANS (CLAMP_IDLE, tree_detected,CLAMP_BOTTOM_CLAMP_HIDE_POS)
{
    /*Hidding the clamp inside the robot*/
    if (ctx.clamp_1_down)
    {
        ctx.pos_backup = ctx.pos_current;
        move_needed(HIDE_POS_TREE * 250,FAST_ROTATION);
    }
    else
    {
        ctx.pos_backup = ctx.pos_current + HALF_TURN * 250;
        move_needed((HALF_TURN + HIDE_POS_TREE) * 250,FAST_ROTATION);
        ctx.clamp_1_down = 1;
    }
    return FSM_NEXT (CLAMP_IDLE, tree_detected);
}


FSM_TRANS (CLAMP_BOTTOM_CLAMP_HIDE_POS, lower_clamp_rotation_success, CLAMP_UNFOLD_UPPER_SET)
{
    IO_CLR (OUTPUT_UPPER_CLAMP_UP);
    IO_SET (OUTPUT_UPPER_CLAMP_DOWN);
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_HIDE_POS, lower_clamp_rotation_success);
}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_HIDE_POS, lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    /*The clamp is blocked somehow.*/
    clamp_blocked();
    ctx.pos_current = ctx.pos_backup;
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_HIDE_POS, lower_clamp_rotation_failure);
}


FSM_TRANS (CLAMP_UNFOLD_UPPER_SET, upper_set_down, CLAMP_BOTTOM_CLAMP_READY)
{
    /*Putting the bottom clamp back to ready.*/
    move_needed(BACK_TO_READY_TREE * 250,FAST_ROTATION);    
    
    /*Opening the top clamp.*/
     IO_SET (OUTPUT_UPPER_CLAMP_OPEN);

    return FSM_NEXT (CLAMP_UNFOLD_UPPER_SET, upper_set_down);

}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_READY, lower_clamp_rotation_success, CLAMP_READY_TO_EMPTY_TREE)
{

    fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_READY, lower_clamp_rotation_success);

}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_READY, lower_clamp_rotation_failure, CLAMP_UPPER_BLOCKED)
{
    /*The clamp is blocked somehow.*/
    clamp_blocked();
    ctx.pos_current = ctx.pos_backup;
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_HIDE_POS, lower_clamp_rotation_failure);
}

FSM_TRANS (CLAMP_READY_TO_EMPTY_TREE, empty_tree, CLAMP_CLOSE_ALL_CLAMPS)
{
    /*Closgin bottom clamp. */
    IO_SET (OUTPUT_LOWER_CLAMP_1_CLOSE);
    /*We need to close both clamp to have an easier turn around. */
    IO_SET (OUTPUT_LOWER_CLAMP_2_CLOSE);


    /*Closing upper & middle clamps.*/
    IO_CLR (OUTPUT_UPPER_CLAMP_OPEN);

    fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
    return FSM_NEXT (CLAMP_READY_TO_EMPTY_TREE, empty_tree);

}

/*---------------------------------------------------------*/
/*  Parts of the FSM that is emptying the tree (totem)     */
/*---------------------------------------------------------*/

FSM_TRANS(CLAMP_CLOSE_ALL_CLAMPS, robot_is_back,CLAMP_REARRANGE_CD)
{
    IO_CLR (OUTPUT_UPPER_CLAMP_OUT);
    IO_SET (OUTPUT_UPPER_CLAMP_IN);
    return FSM_NEXT (CLAMP_CLOSE_ALL_CLAMPS, robot_is_back);

}

FSM_TRANS (CLAMP_CLOSE_ALL_CLAMPS, stop_tree_approach, CLAMP_READY_TO_EMPTY_TREE)
{
    IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    IO_SET (OUTPUT_UPPER_CLAMP_OPEN);
    fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
    return FSM_NEXT (CLAMP_CLOSE_ALL_CLAMPS, stop_tree_approach);
}

FSM_TRANS_TIMEOUT (CLAMP_REARRANGE_CD, TIMEOUT_RECENTRAGE, CLAMP_BOTTOM_CLAMP_HIDE_POS2)
{
    /*Hidding the clamp inside the robot.*/
    move_needed(HIDE_POS_TREE_2 * 250,FAST_ROTATION);
    return FSM_NEXT_TIMEOUT (CLAMP_REARRANGE_CD);
}
FSM_TRANS (CLAMP_BOTTOM_CLAMP_HIDE_POS2, lower_clamp_rotation_success, CLAMP_RELEASE_ASSERV)
{
    mimot_motor_free (0, 1);
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_HIDE_POS2, lower_clamp_rotation_success);

}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_HIDE_POS2, lower_clamp_rotation_failure, CLAMP_UPPER_BLOCKED)
{
    clamp_blocked();
    ctx.pos_current = ctx.pos_backup;
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_HIDE_POS2, lower_clamp_rotation_failure);

}

FSM_TRANS_TIMEOUT (CLAMP_RELEASE_ASSERV, TIMEOUT_FREE_ASSERV, CLAMP_FOLD_UPPER_SET)
{
    IO_CLR (OUTPUT_UPPER_CLAMP_DOWN);
    IO_SET (OUTPUT_UPPER_CLAMP_UP);
    return FSM_NEXT_TIMEOUT (CLAMP_RELEASE_ASSERV);

}
FSM_TRANS (CLAMP_FOLD_UPPER_SET, upper_set_up, CLAMP_OPEN_UPPER_CLAMPS)
{
    IO_SET (OUTPUT_UPPER_CLAMP_OPEN);
    IO_CLR (OUTPUT_UPPER_CLAMP_IN);
    IO_SET (OUTPUT_UPPER_CLAMP_OUT);
    return FSM_NEXT (CLAMP_FOLD_UPPER_SET, upper_set_up);

}

FSM_TRANS_TIMEOUT (CLAMP_OPEN_UPPER_CLAMPS, TIMEOUT_OPEN_CLAMPS, 
                   stop_tree_branch,CLAMP_TURN_HALF_WAY,
                   continue_empty_tree_branch,CLAMP_READY_TO_RECALE)
{
    if (ctx.clamp_1_down)
        IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    else
        IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    if (ctx.stop_tree_approach)
    {
        main_set_drop_coin_pos(ctx.pos_current + ((HALF_TURN - HIDE_POS_TREE) * 250) - POS_DELAY);
        move_needed((HALF_TURN - HIDE_POS_TREE) * 250,FAST_ROTATION);
        ctx.stop_tree_approach = 0;
        fsm_queue_post_event (FSM_EVENT (AI, clamps_ready));
        return FSM_NEXT_TIMEOUT (CLAMP_OPEN_UPPER_CLAMPS,stop_tree_branch);
    }
    else
    {
        move_needed(RECALE_COIN * 250,SLOW_ROTATION);
        return FSM_NEXT_TIMEOUT (CLAMP_OPEN_UPPER_CLAMPS,continue_empty_tree_branch);
    }
}

FSM_TRANS (CLAMP_READY_TO_RECALE, lower_clamp_rotation_success, CLAMP_READY_TO_RECALE_2)
{
    if (ctx.clamp_1_down)
        IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    else
        IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    return FSM_NEXT (CLAMP_READY_TO_RECALE,lower_clamp_rotation_success);
}

FSM_TRANS (CLAMP_READY_TO_RECALE, lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    ctx.pos_current = ctx.pos_backup;
    return FSM_NEXT (CLAMP_READY_TO_RECALE,lower_clamp_rotation_failure);
}

FSM_TRANS_TIMEOUT (CLAMP_READY_TO_RECALE_2, TIMEOUT_OPEN_CLAMPS, CLAMP_TEMPO_RECALE)
{
    if (ctx.clamp_1_down)
        IO_SET (OUTPUT_LOWER_CLAMP_1_CLOSE);
    else
        IO_SET (OUTPUT_LOWER_CLAMP_2_CLOSE);
    return FSM_NEXT_TIMEOUT (CLAMP_READY_TO_RECALE_2);
}

FSM_TRANS_TIMEOUT (CLAMP_TEMPO_RECALE, TIMEOUT_OPEN_CLAMPS,CLAMP_END_RECALE)
{
    move_needed(BACK_TO_READY_RECALE_COIN  * 250,FAST_ROTATION);
    return FSM_NEXT_TIMEOUT (CLAMP_TEMPO_RECALE);
}

FSM_TRANS (CLAMP_END_RECALE, lower_clamp_rotation_failure,CLAMP_BLOCKED)
{
    ctx.pos_current = ctx.pos_backup;
    return FSM_NEXT (CLAMP_END_RECALE,lower_clamp_rotation_failure);
}

FSM_TRANS (CLAMP_END_RECALE, lower_clamp_rotation_success,CLAMP_TURN_HALF_WAY)
{
    main_set_drop_coin_pos(ctx.pos_current + (HALF_TURN * 250) - POS_DELAY);
    move_needed(HALF_TURN  * 250,FAST_ROTATION);
    ctx.clamp_1_down = !ctx.clamp_1_down;
    return FSM_NEXT (CLAMP_END_RECALE, lower_clamp_rotation_success);
}
/*---------------------------------------------------------------------------------*/
/*Parts of the FSM that goes back to idle after receiving the stop approach signal */
/*---------------------------------------------------------------------------------*/
FSM_TRANS (CLAMP_BOTTOM_CLAMP_HIDE_POS,stop_tree_approach,CLAMP_BOTTOM_CLAMP_BACK)
{
    /*Putting the clamp back to take coin*/
    if (ctx.clamp_1_down)
    {
        move_needed(BACK_TO_READY_TREE * 250,FAST_ROTATION);
    }
    else
    {
        move_needed((16-BACK_TO_READY_TREE) * 250,FAST_ROTATION);
    }
    return FSM_NEXT(CLAMP_BOTTOM_CLAMP_HIDE_POS,stop_tree_approach);
  
}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_BACK,lower_clamp_rotation_success,CLAMP_WAIT_BEFORE_IDLE)
{
    return FSM_NEXT(CLAMP_BOTTOM_CLAMP_BACK,lower_clamp_rotation_success);
    
}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_BACK,lower_clamp_rotation_failure,CLAMP_BLOCKED)
{
    clamp_blocked();
    return FSM_NEXT(CLAMP_BOTTOM_CLAMP_BACK,lower_clamp_rotation_failure);
    
}

FSM_TRANS (CLAMP_UNFOLD_UPPER_SET, stop_tree_approach,CLAMP_RELEASE_ASSERV)
{
    ctx.stop_tree_approach = 1;
    return FSM_NEXT (CLAMP_UNFOLD_UPPER_SET, stop_tree_approach);

}

FSM_TRANS (CLAMP_BOTTOM_CLAMP_READY, stop_tree_approach,CLAMP_FINISH_BOTTOM_CLAMP_READY)
{
    return FSM_NEXT (CLAMP_BOTTOM_CLAMP_READY, stop_tree_approach);

}
FSM_TRANS (CLAMP_FINISH_BOTTOM_CLAMP_READY, lower_clamp_rotation_success,CLAMP_REARRANGE_CD)
{
    return FSM_NEXT (CLAMP_FINISH_BOTTOM_CLAMP_READY, lower_clamp_rotation_success);

}
FSM_TRANS (CLAMP_FINISH_BOTTOM_CLAMP_READY, lower_clamp_rotation_failure,CLAMP_BLOCKED)
{
    clamp_blocked();
    return FSM_NEXT (CLAMP_FINISH_BOTTOM_CLAMP_READY, lower_clamp_rotation_failure);

}

FSM_TRANS (CLAMP_READY_TO_EMPTY_TREE, stop_tree_approach, CLAMP_REARRANGE_CD)
{
    return FSM_NEXT (CLAMP_READY_TO_EMPTY_TREE, stop_tree_approach);
}

/*---------------------------------------------------------------------------------*/
/*     Parts of the FSM that takes care of the bottom clamp when it's blocked      */
/*---------------------------------------------------------------------------------*/

FSM_TRANS_TIMEOUT (CLAMP_UPPER_BLOCKED, TIMEOUT_OPEN_CLAMPS,
                   CLAMP_UPPER_BLOCKED_WAIT)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_blocked));
    IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    return FSM_NEXT_TIMEOUT (CLAMP_BLOCKED);
}

FSM_TRANS (CLAMP_UPPER_BLOCKED_WAIT, clamp_unblock, CLAMP_UPPER_BLOCKED_UP)
{
    IO_SET (OUTPUT_UPPER_CLAMP_UP);
    IO_CLR (OUTPUT_UPPER_CLAMP_DOWN);
    IO_SET (OUTPUT_UPPER_CLAMP_OPEN);
    IO_CLR (OUTPUT_UPPER_CLAMP_IN);
    IO_SET (OUTPUT_UPPER_CLAMP_OUT);
    return FSM_NEXT (CLAMP_UPPER_BLOCKED_WAIT, clamp_unblock);
}

FSM_TRANS_TIMEOUT (CLAMP_UPPER_BLOCKED_UP, 5 * TIMEOUT_OPEN_CLAMPS,
                   CLAMP_OPEN_BOTTOM_CLAMPS)
{
    mimot_move_motor0_absolute (mimot_get_motor0_position() - 16 * 250, MEDIUM_ROTATION);
    return FSM_NEXT_TIMEOUT (CLAMP_UPPER_BLOCKED_UP);
}

FSM_TRANS_TIMEOUT (CLAMP_BLOCKED,TIMEOUT_OPEN_CLAMPS,CLAMP_OPEN_BOTTOM_CLAMPS)
{
    fsm_queue_post_event (FSM_EVENT (AI, clamp_blocked));
    /* Opening the 2 clamps. */
    IO_CLR (OUTPUT_LOWER_CLAMP_1_CLOSE);
    IO_CLR (OUTPUT_LOWER_CLAMP_2_CLOSE);
    return FSM_NEXT_TIMEOUT (CLAMP_BLOCKED);
}

FSM_TRANS (CLAMP_OPEN_BOTTOM_CLAMPS, clamp_unblock, CLAMP_WAIT)
{
    mimot_move_motor0_absolute (mimot_get_motor0_position() - 16 * 250, MEDIUM_ROTATION);
    return FSM_NEXT (CLAMP_OPEN_BOTTOM_CLAMPS, clamp_unblock);
}

FSM_TRANS (CLAMP_OPEN_BOTTOM_CLAMPS,clamp_is_dead, CLAMP_SHITTY_STATE)
{
    return FSM_NEXT (CLAMP_OPEN_BOTTOM_CLAMPS, clamp_is_dead);
}

FSM_TRANS (CLAMP_WAIT,lower_clamp_rotation_success, CLAMP_TURN_BACKWARD)
{
    return FSM_NEXT (CLAMP_WAIT, lower_clamp_rotation_success);
}

FSM_TRANS (CLAMP_WAIT,lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    ctx.cpt_blocked += 1;
    return FSM_NEXT (CLAMP_WAIT, lower_clamp_rotation_failure);
}

FSM_TRANS_TIMEOUT (CLAMP_TURN_BACKWARD,TIMEOUT_BLOCKED, CLAMP_TURN_FORWARD)
{
    move_needed(0,MEDIUM_ROTATION);
    return FSM_NEXT_TIMEOUT (CLAMP_TURN_BACKWARD);
}


FSM_TRANS (CLAMP_TURN_FORWARD,lower_clamp_rotation_success, CLAMP_WAIT_BEFORE_IDLE)
{
    ctx.cpt_blocked = 0;
    return FSM_NEXT (CLAMP_TURN_FORWARD, lower_clamp_rotation_success);
}


FSM_TRANS (CLAMP_TURN_BACKWARD,lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    ctx.cpt_blocked += 1;
    return FSM_NEXT (CLAMP_TURN_BACKWARD, lower_clamp_rotation_failure);
}


FSM_TRANS (CLAMP_TURN_FORWARD,lower_clamp_rotation_failure, CLAMP_BLOCKED)
{
    ctx.cpt_blocked += 1;
    return FSM_NEXT (CLAMP_TURN_FORWARD, lower_clamp_rotation_failure);
}

/* vim: set cino={0\:0t0(0 et: */
