/* trace.c */
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
#include "modules/flash/flash.h"
#include "trace.h"

static trace_t trace_global;

void
trace_init (void)
{
    trace_global.flash_status = flash_init ();

    /* Get the first sector to write. */
    if (trace_global.flash_status)
      {
	trace_global.flash_addr = flash_sector_next (0);
      }
}

void
trace_print_word (uint8_t arg)
{
    if (trace_global.flash_status)
	flash_write (trace_global.flash_addr++, arg);
}

