/* logistic.c */
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
#include "logistic.h"

#include "clamp.h"
#include "defs.h"

#include "contact.h"
#include "io.h"

#include "debug.host.h"

/** Handle elements stored inside the robot. */

/** Global context. */
struct logistic_t logistic_global;
#define ctx logistic_global

inline void
logistic_debug_dump (void)
{
#ifdef HOST
    uint8_t i;
    static const char *names[][CLAMP_SLOT_NB] = {
	  { "f1", "f2", "f3", "b1", "b2", "b3", "s1" },
	  { "F1", "F2", "F3", "B1", "B2", "B3", "S1" }
    };
    static const char *names_dir[] = { "--", "<-", "->" };
    DPRINTF ("%s", names_dir[ctx.collect_direction]);
    for (i = 0; i < CLAMP_SLOT_NB; i++)
      {
	DPRINTF (" %s", ctx.slots[i]
		 ? names[ELEMENT_IS_HEAD (ctx.slots[i]) ? 1 : 0][i]
		 : "__");
      }
    if (ctx.moving_from != CLAMP_SLOT_NB)
      {
	DPRINTF ("    %s => %s", names[0][ctx.moving_from], names[0][ctx.moving_to]);
      }
    DPRINTF ("\n");
    DPRINTF ("construct_possible: %u\n", ctx.construct_possible);
    DPRINTF ("prepare: %u\n", ctx.prepare);
    DPRINTF ("ready: %u\n", ctx.ready);
    DPRINTF ("need_prepare: %u\n", ctx.need_prepare);
#endif
}

/** Return 1 if location corresponds to element. */
inline uint8_t
logistic_case_test (uint8_t loc, uint8_t e)
{
    if (e == LOG_a && ctx.slots[loc])
	return 1;
    if (e == LOG__)
	return 1;
    if ((e == LOG_e || e == LOG_D) && !ctx.slots[loc])
	return 1;
    if ((e == LOG_P || e == LOG_p) && ctx.slots[loc] &&
	!ELEMENT_IS_HEAD (ctx.slots[loc]))
	return 1;
    if ((e == LOG_H || e == LOG_h) && ELEMENT_IS_HEAD (ctx.slots[loc]))
	return 1;
    if (e == LOG_t && ctx.slots[loc] == ELEMENT_TOWER)
	return 1;
    return 0;
}

static uint8_t
logistic_case (uint8_t e1, uint8_t e2, uint8_t e3, uint8_t e4, uint8_t e5,
	       uint8_t e6, uint8_t e7, uint8_t new_dir, uint8_t ready,
	       uint8_t check_symetric)
{
    /* Define direction bay and opposed bay. */
    uint8_t dir_bay, opp_bay;
    if (ctx.collect_direction == DIRECTION_FORWARD)
      {
	dir_bay = CLAMP_SLOT_FRONT_BOTTOM;
	opp_bay = CLAMP_SLOT_BACK_BOTTOM;
      }
    else
      {
	dir_bay = CLAMP_SLOT_BACK_BOTTOM;
	opp_bay = CLAMP_SLOT_FRONT_BOTTOM;
      }
    /*if (check_symetric)
      {
	uint8_t tmp = dir_bay;
	dir_bay = opp_bay;
	opp_bay = tmp;
      }*/

    /* Emplacement of elements. */
    uint8_t
	e1_loc = dir_bay + 2,
	e3_loc = dir_bay + 1,
	e6_loc = dir_bay,
	e4_loc = CLAMP_SLOT_SIDE,
	e2_loc = opp_bay + 2,
	e5_loc = opp_bay + 1,
	e7_loc = opp_bay;

    /* Check elements are here. */
    if (!(logistic_case_test (e1_loc, e1) &&
	  logistic_case_test (e2_loc, e2) &&
	  logistic_case_test (e3_loc, e3) &&
	  logistic_case_test (e4_loc, e4) &&
	  logistic_case_test (e5_loc, e5) &&
	  logistic_case_test (e6_loc, e6) &&
	  logistic_case_test (e7_loc, e7)))
      {
	/* Check mirror. */
	/*if (!check_symetric)
	    logistic_case (e1, e2, e3, e4, e5, e6, e7, new_dir, ready, 1);*/
	return 0;
      }

    /* Find source/destination if we have to make a move. */
    /* Find source. */
    uint8_t src = CLAMP_SLOT_NB, dst = CLAMP_SLOT_NB;
    if (e1 == LOG_P ||e1 == LOG_H)
	src = e1_loc;
    else if (e3 == LOG_P ||e3 == LOG_H)
	src = e3_loc;
    else if (e6 == LOG_P ||e6 == LOG_H)
	src = e6_loc;
    else if (e4 == LOG_P ||e4 == LOG_H)
	src = e4_loc;
    else if (e2 == LOG_P ||e2 == LOG_H)
	src = e2_loc;
    else if (e5 == LOG_P ||e5 == LOG_H)
	src = e5_loc;
    else if (e7 == LOG_P ||e7 == LOG_H)
	src = e7_loc;

    /* Find destination. */
    if (e1 == LOG_D)
	dst = e1_loc;
    else if (e3 == LOG_D)
	dst = e3_loc;
    else if (e6 == LOG_D)
	dst = e6_loc;
    else if (e4 == LOG_D)
	dst = e4_loc;
    else if (e2 == LOG_D)
	dst = e2_loc;
    else if (e5 == LOG_D)
	dst = e5_loc;
    else if (e7 == LOG_D)
	dst = e7_loc;
    
    /* We are making a move. */
    if (src != CLAMP_SLOT_NB && dst != CLAMP_SLOT_NB)
      {
	if (ctx.slots[src] && !ctx.slots[dst])
	{
	    ctx.moving_from = src;
	    ctx.moving_to = dst;
	    ctx.ready = 0;
	}
      }

    /* Set collect direction. */
    /* LEFT means we keep the same side, RIGHT means we put the opposed side. */
    if (new_dir == LOG_DIR_RIGHT)
      {
	if (ctx.collect_direction == DIRECTION_FORWARD)
	    ctx.collect_direction = DIRECTION_BACKWARD;
	else
	    ctx.collect_direction = DIRECTION_FORWARD;
      }

    /* Don't touch clamp's idle position if broken. */
    if (ctx.prepare != 3)
      {
	/* Set clamp position idle. */
	if (ctx.collect_direction == DIRECTION_FORWARD)
	    ctx.clamp_pos_idle = CLAMP_SLOT_FRONT_MIDDLE;
	else
	    ctx.clamp_pos_idle = CLAMP_SLOT_BACK_MIDDLE;
      }
    /* Set ready */
    ctx.ready = ready;
    return 1;
}

static void
logistic_update_construct_possible ()
{
    uint8_t pawn = 0, head = 0, i;
    /* Check for tower. */
    if (ctx.slots[CLAMP_SLOT_FRONT_BOTTOM] ==  ELEMENT_TOWER ||
	ctx.slots[CLAMP_SLOT_BACK_BOTTOM] == ELEMENT_TOWER)
      {
	ctx.construct_possible = 1;
	return;
      }

    for (i = CLAMP_SLOT_FRONT_BOTTOM; i < CLAMP_SLOT_NB; i++)
      {
	    if (ELEMENT_IS_HEAD (ctx.slots[i]))
		head++;
	    else if (ctx.slots[i])
		pawn++;
	    if (head >= 1 && pawn >= 2)
	    {
	      ctx.construct_possible = 1;
	      return;
	    }
      }
    if (pawn || head)
	ctx.construct_possible = 2;
    else
	ctx.construct_possible = 0;
}

static void
logistic_update_need_prepare ()
{
    uint8_t i, head = 0, pawn = 0;
    for (i = CLAMP_SLOT_FRONT_BOTTOM; i < CLAMP_SLOT_NB; i++)
      {
	    if (ELEMENT_IS_HEAD (ctx.slots[i]))
		head++;
	    else if (ctx.slots[i])
		pawn++;
      }
    if ((head == 1 && pawn == 3) ||
	head == 2 || pawn >= 4)
	ctx.need_prepare = 1;
    else
	ctx.need_prepare = 0;

    /* Define direction bay and opposed bay. */
    uint8_t dir_bay, opp_bay;
    if (ctx.collect_direction == DIRECTION_FORWARD)
      {
	dir_bay = CLAMP_SLOT_FRONT_BOTTOM;
	opp_bay = CLAMP_SLOT_BACK_BOTTOM;
      }
    else
      {
	dir_bay = CLAMP_SLOT_BACK_BOTTOM;
	opp_bay = CLAMP_SLOT_FRONT_BOTTOM;
      }
    /* If a head appear at the back (?) */
    if (ELEMENT_IS_HEAD (ctx.slots[opp_bay]))
	ctx.need_prepare = 1;

    /* We founded a tower ! */
    if (ctx.slots[CLAMP_SLOT_FRONT_BOTTOM] == ELEMENT_TOWER ||
	ctx.slots[CLAMP_SLOT_FRONT_BOTTOM] == ELEMENT_TOWER)
	ctx.need_prepare = 1;
}

static void
logisitic_make_broken ()
{
    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   a,      _, RIGHT, 1);

    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   _,      a, LEFT, 1);
}

static void
logistic_make_tower ()
{
    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   _,      t, LEFT, 1);

    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   t,      _, RIGHT, 1);

    LOGISTIC_CASE (D,      _,
		   e,  _,  _,
		   H,      _, RIGHT, 0);

    LOGISTIC_CASE (_,      h,
		   _,  _,  p,
		   _,      p, LEFT, 1);

    LOGISTIC_CASE (P,      h,
		   e,  _,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (_,      h,
		   e,  _,  e,
		   P,      D, LEFT, 0);

    LOGISTIC_CASE (_,      h,
		   _,  P,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (P,      h,
		   e,  _,  D,
		   _,      p, LEFT, 0);

    LOGISTIC_CASE (_,      h,
		   e,  _,  D,
		   P,      p, LEFT, 0);

    LOGISTIC_CASE (_,      h,
		   _,  P,  D,
		   _,      p, LEFT, 0);
}

static void
logistic_make_unload ()
{
    /* Making a small tower. Move head when having pawn. */
    LOGISTIC_CASE (D,      _,
		   e,  p,  _,
		   H,      _, RIGHT, 0);

    LOGISTIC_CASE (D,      p,
		   e,  e,  e,
		   H,      _, RIGHT, 0);

    LOGISTIC_CASE (D,      _,
		   e,  e,  e,
		   H,      p, RIGHT, 0);

    /* Making a small tower. Move pawn under head. */
    LOGISTIC_CASE (_,      h,
		   _,  P,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (P,      h,
		   e,  e,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (_,      h,
		   e,  e,  e,
		   P,      D, LEFT, 0);

    /* Making a small tower. Finally move head on pawn. */
    LOGISTIC_CASE (_,      H,
		   _,  _,  D,
		   _,      p, LEFT, 0);

    LOGISTIC_CASE (_,      _,
		   _,  _,  h,
		   _,      p, LEFT, 1);

    /* Having any element. */
    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   _,      a, LEFT, 1);

    LOGISTIC_CASE (_,      _,
		   _,  _,  _,
		   a,      _, RIGHT, 1);

    LOGISTIC_CASE (_,      a,
		   _,  _,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (a,      _,
		   e,  _,  _,
		   D,      _, RIGHT, 0);

    LOGISTIC_CASE (_,      _,
		   _,  P,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (_,      _,
		   e,  P,  _,
		   D,      _, RIGHT, 0);
}

static void
logisitic_make_switches ()
{
    LOGISTIC_CASE (_,      P,
		   _,  D,  e,
		   _,      _, LEFT, 0);

    LOGISTIC_CASE (P,      _,
		   e,  D,  _,
		   _,      _, LEFT, 0);

    LOGISTIC_CASE (_,      _,
		   e,  D,  _,
		   P,      _, LEFT, 0);

    LOGISTIC_CASE (_,      _,
		   _,  D,  e,
		   _,      P, LEFT, 0);

    LOGISTIC_CASE (_,      D,
		   e,  p,  e,
		   P,      _, LEFT, 0);

    LOGISTIC_CASE (_,      a,
		   e,  p,  e,
		   P,      D, LEFT, 0);
    
    LOGISTIC_CASE (D,      _,
		   e,  _,  _,
		   H,      _, RIGHT, 0);

    LOGISTIC_CASE (h,      P,
		   e,  _,  e,
		   D,      _, RIGHT, 0);

    LOGISTIC_CASE (h,      e,
		   e,  _,  e,
		   D,      P, RIGHT, 0);

    LOGISTIC_CASE (P,      h,
		   e,  _,  e,
		   _,      D, LEFT, 0);

    LOGISTIC_CASE (e,      h,
		   e,  _,  e,
		   P,      D, LEFT, 0);
}

void
logistic_decision (void)
{
    /* Reset. */
    ctx.moving_from = CLAMP_SLOT_NB;
    ctx.moving_to = CLAMP_SLOT_NB;
    ctx.construct_possible = 1;
    ctx.ready = 0;
    ctx.need_prepare = 0;

    /* Update context. */
    logistic_update_construct_possible ();
    /* Update if a something is possible. */
    logistic_update_need_prepare ();

    /* Broken clamp. */
    if (ctx.construct_possible && ctx.prepare == 3)
      {
	DPRINTF ("\nlogisitic_make_broken\n");
	logisitic_make_broken ();
      }
    /* Prepare tower. */
    else if (ctx.construct_possible == 1 && ctx.prepare != 0)
      {

	DPRINTF ("\nlogisitic_make_tower\n");
	logistic_make_tower ();
      }
    /* Need to unload. */
    else if (ctx.construct_possible == 2 && ctx.prepare == 2)
      {
	DPRINTF ("\nlogisitic_make_unload\n");
	logistic_make_unload ();
      }
    /* Internal switches. */
    else
      {
	DPRINTF ("\nlogisitic_make_switches\n");
	logisitic_make_switches ();
      }

    logistic_debug_dump ();
    return;
}

void
logistic_init (void)
{
    uint8_t i;
    for (i = 0; i < CLAMP_SLOT_NB; i++)
	ctx.slots[i] = 0;
    ctx.moving_from = ctx.moving_to = CLAMP_SLOT_NB;
    ctx.collect_direction = DIRECTION_FORWARD;
    ctx.clamp_pos_idle = ctx.collect_direction == DIRECTION_FORWARD
	? CLAMP_SLOT_FRONT_MIDDLE : CLAMP_SLOT_BACK_MIDDLE;
    ctx.construct_possible = 0;
    ctx.prepare = 1;
    ctx.ready = 0;
    ctx.need_prepare = 0;
}

void
logistic_update (void)
{
    uint8_t side_now = !IO_GET (CONTACT_SIDE);
    /* Filter side contact. */
    if (side_now)
      {
	ctx.side_filter = 0;
	ctx.side_state = 1;
      }
    else if (ctx.side_filter++ == 2 * 250)
      {
	ctx.side_state = 0;
	ctx.side_filter = 0;
      }
    /* Side slot element can be lost. */
    if (ctx.moving_from != CLAMP_SLOT_SIDE && !ctx.side_state)
	ctx.slots[CLAMP_SLOT_SIDE] = 0;
}

void
logistic_element_new (uint8_t pos, uint8_t element_type)
{
    assert (pos < CLAMP_SLOT_NB);
    assert (!ctx.slots[pos]);
    ctx.slots[pos] = element_type;
    logistic_decision ();
}

void
logistic_element_change (uint8_t pos, uint8_t element_type)
{
    assert (pos < CLAMP_SLOT_NB);
    ctx.slots[pos] = element_type;
    logistic_decision ();
}

void
logistic_element_move_done (void)
{
    assert (!ctx.slots[ctx.moving_to] || ctx.moving_to == ctx.moving_from);
    ctx.slots[ctx.moving_to] = ctx.slots[ctx.moving_from];
    ctx.slots[ctx.moving_from] = 0;
    ctx.moving_from = ctx.moving_to = CLAMP_SLOT_NB;
    logistic_decision ();
}

void
logistic_drop (uint8_t direction)
{
    uint8_t bay = direction == DIRECTION_FORWARD
	? CLAMP_SLOT_BACK_BOTTOM : CLAMP_SLOT_FRONT_BOTTOM;
    uint8_t i;
    for (i = bay; i < bay + 3; i++)
	ctx.slots[i] = 0;
    logistic_decision ();
}

uint8_t
logistic_drop_element_type (uint8_t direction)
{
    uint8_t bay = direction == DIRECTION_FORWARD
	? CLAMP_SLOT_BACK_BOTTOM : CLAMP_SLOT_FRONT_BOTTOM;
    uint8_t nb = 0;
    uint8_t element_type = ELEMENT_NONE;
    uint8_t i;
    for (i = bay; i < bay + 3; i++)
      {
	if (ctx.slots[i])
	  {
	    nb++;
	    element_type = ctx.slots[i];
	  }
      }
    if (ELEMENT_IS_HEAD (element_type))
	return ELEMENT_TOWER;
    else if (nb > 1)
	return ELEMENT_TOWER;
    else
	return ELEMENT_PAWN;
}

void
logistic_dump (uint8_t direction, uint8_t drop_top)
{
    /* Drop. */
    uint8_t i;
    uint8_t bay = direction == DIRECTION_FORWARD
	? CLAMP_SLOT_FRONT_BOTTOM : CLAMP_SLOT_BACK_BOTTOM;
    for (i = bay; i < bay + 2 + drop_top; i++)
	ctx.slots[i] = 0;
}

static uint8_t
logistic_slot_clear (uint8_t slot)
{
    if (CLAMP_IS_SLOT_IN_FRONT_BAY (slot))
      {
	if (ctx.slots[CLAMP_SLOT_FRONT_MIDDLE])
	    return 0;
	uint8_t middle_type = ctx.slots[CLAMP_SLOT_FRONT_BOTTOM];
	if (ELEMENT_IS_HEAD (middle_type) || middle_type == ELEMENT_TOWER)
	    return 0;
      }
    else if (CLAMP_IS_SLOT_IN_BACK_BAY (slot))
      {
	if (ctx.slots[CLAMP_SLOT_BACK_MIDDLE])
	    return 0;
	uint8_t middle_type = ctx.slots[CLAMP_SLOT_BACK_BOTTOM];
	if (ELEMENT_IS_HEAD (middle_type) || middle_type == ELEMENT_TOWER)
	    return 0;
      }
    return 1;
}

uint8_t
logistic_path_clear (uint8_t slot1, uint8_t slot2)
{
    return logistic_slot_clear (slot1) && logistic_slot_clear (slot2);
}

