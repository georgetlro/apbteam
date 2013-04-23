/* test_rvb.c */
/* rvb - Carte de détection de couleur. {{{
 *
 * Copyright (C) 2006 Nicolas Schodet
 *
 * Robot APB Team/Efrei 2005.
 *        Web: http://assos.efrei.fr/robot/
 *      Email: robot AT efrei DOT fr
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
#include "modules/uart/uart.h"
#include "modules/proto/proto.h"
#include "modules/utils/utils.h"

int rvb_output, rvb_output_cpt;
#ifdef __AVR_ATmega8__
#define RVB_TIMER TCNT0
#else
#define RVB_TIMER TCNT2
#endif

void
proto_callback (uint8_t cmd, uint8_t size, uint8_t *args)
{
/* This macro combine command and size in one integer. */
#define c(cmd, size) (cmd << 8 | size)
    switch (c (cmd, size))
      {
      case c ('z', 0):
	utils_reset ();
	break;
      case c ('R', 1):
	rvb_output_cpt = rvb_output = args[0];
	break;
      default:
	proto_send0 ('?');
	return;
      }
    /* When no error acknoledge. */
    proto_send (cmd, size, args);
#undef c
}

/** Initialise the timer. */
static inline void
timer_init (void)
{
    TCCR1B = regv (ICNC1, ICES1, 5, WGM13, WGM12, CS12, CS11, CS10,
		       0,     0, 0,     0,     0,    0,    0,    1);
    /* Fov = F_io / (prescaler * (TOP + 1))
     * TOP = 0xffff
     * prescaler = 1
     * Tov = 1 / Fov = 4.444 ms */
}

/** Wait for timer overflow. */
static inline void
timer_wait (void)
{
    while (!(TIFR & _BV (TOV1)))
	;
    /* Write 1 to clear. */
    TIFR = _BV (TOV1);
}

/** Initialise the counter. */
static inline void
counter_init (void)
{
#ifdef __AVR_ATmega8__
    TCCR0 = regv (7, 6, 5, 4, 3, CS02, CS01, CS00,
		  0, 0, 0, 0, 0,    1,    1,    1);
#else
    TCCR2 = regv (FOC2, WGM20, COM21, COM20, WGM21, CS22, CS21, CS20,
		  0   , 0    , 0    , 0    , 0    , 1   , 1   , 1   );

#endif
    /* Increment on rising edge. */
}

int
main (void)
{
    uint8_t r, v, b, c;
    uart0_init ();
    timer_init ();
    counter_init ();
    sei ();
    proto_send0 ('z');
#ifdef __AVR_ATmega8__
    DDRC = 0x0f;
    PORTC = 0x0e;
#else
    DDRC = 0xff;
    DDRD |= _BV(6);
    DDRF |= _BV(6) | _BV(7);
    PORTD &= ~_BV(6);
    PORTC = 0xff;
#endif
    r = v = b = c = 0;
    while (1)
      {
	/* Rouge. */
#ifdef __AVR_ATmega8__
	PORTC = 0x3;
#else
	PORTF = (PORTF & ~0xC0) | (0 << 6);
#endif
	timer_wait ();
	RVB_TIMER = 0;
	timer_wait ();
	r = RVB_TIMER;
	/* Bleu. */
#ifdef __AVR_ATmega8__
	PORTC = 0x7;
#else
	PORTF = (PORTF & ~0xC0) | (1 << 6);
#endif
	timer_wait ();
	RVB_TIMER = 0;
	timer_wait ();
	b = RVB_TIMER;
	/* Vert. */
#ifdef __AVR_ATmega8__
	PORTC = 0xf;
#else
	PORTF = (PORTF & ~0xC0) | (3 << 6);
#endif
	timer_wait ();
	RVB_TIMER = 0;
	timer_wait ();
	v = RVB_TIMER;
	/* Clair. */
#ifdef __AVR_ATmega8__
	PORTC = 0xb;
#else
	PORTF = (PORTF & ~0xC0) | (2 << 6);
#endif
	timer_wait ();
	RVB_TIMER = 0;
	timer_wait ();
	c = RVB_TIMER;
	/* Envois. */
	if (rvb_output && !--rvb_output_cpt)
	  {
	    proto_send4b ('R', r, v, b, c);
	    rvb_output_cpt = rvb_output;
	  }
	while (uart0_poll ())
	    proto_accept (uart0_getc ());
      }
}

