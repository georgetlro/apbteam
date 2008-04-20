/* getsamples.c */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
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
#include "getsamples.h"
#include "fsm.h"
#include "trap.h"

#include "io.h"

/**
 * The approach angle to face the distributor.
 */
int16_t approach_angle_;

/**
 * The samples bit field to collect.
 */
uint8_t sample_bitfield_;

/* Start the get samples FSM. */
void
getsamples_start (int16_t approach_angle, uint8_t sample_bitfield)
{
    /* Set parameters */
    approach_angle_ = approach_angle;
    sample_bitfield_ = sample_bitfield;

    /* Start the get samples FSM */
    fsm_init (&getsamples_fsm);
    fsm_handle_event (&getsamples_fsm, GETSAMPLES_EVENT_start);
}

/* Configure the classifier (using the trap and the internal bit field) for the first bit set to 1. */
void
getsamples_configure_classifier (void)
{
    uint8_t trap_num;
    /* Go through all the bits of the sample bit field */
    for (trap_num = 0; trap_num < trap_count; trap_num++)
      {
	/* Is the bit set? */
	if (bit_is_set (sample_bitfield_, trap_num))
	  {
	    /* Configure the classifier */
	    trap_setup_path_to_box (trap_num);
	    /* Reset this bit */
	    sample_bitfield_ &= ~_BV (trap_num);
	    /* Stop here */
	    return;
	  }
      }
}
