/* test-write-onebyte.c */
/* avr.flash - AVR Flash SPI use. {{{
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
#include "io.h"
#include "../flash.h"
#include "modules/proto/proto.h"
#include "modules/utils/utils.h"
#include "modules/uart/uart.h"

#define TEST_BASE 0x00

void
proto_callback (uint8_t cmd, uint8_t size, uint8_t *args)
{
#define c(cmd, size) (cmd << 8 | size)
    switch (c (cmd, size))
      {
      case c ('z', 0):
	/* Reset */
	utils_reset ();
	break;
      default:
	/* Error */
	proto_send0 ('?');
	return;
      }
    /* Acknowledge what has been done */
    proto_send (cmd, size, args);
}

int
main (void)
{
    uart0_init ();
    proto_send0 ('z');
    proto_send0 ('c');
    flash_init ();
    proto_send0 ('f');
    flash_write (TEST_BASE, 'a');
    proto_send1b ('o',flash_read(TEST_BASE));
    
    while (1)
	proto_accept (uart0_getc ());
}

