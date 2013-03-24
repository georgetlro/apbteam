/* main_avr.c */
/* Beacon State Machine & Main. {{{
 *
 * Copyright (C) 2012 Florent Duchon
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

#include <types.h>
#include <util/delay.h>
#include "configuration.h"
#include "codewheel.h"
#include "print.h"
#include "led.h"


void APL_TaskHandler(void)
{
	static int init_done = 0;
	
	if(init_done == 0)
	{	
		/* Init Led */
		init_led();
		
		/* Init Serial Interface for debug */ 
		initSerialInterface();          	
		
		codewheel_init();
		
		init_done = 1;
		
		uprintf("test_codewheel : init done\r\n");
	}
	SYS_PostTask(APL_TASK_ID);
}

int main(void)
{ 
	SYS_SysInit();
	for(;;)
	{
 		SYS_RunTask();
	}
}


