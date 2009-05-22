#ifndef elevator_h
#define elevator_h
/* elevator.h */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
 *
 * Copyright (C) 2009 Nicolas Haller
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

/**
 * State of the elevator
 */
extern uint8_t elvt_is_ready;

/**
 * nb puck in the elevator
 */
extern uint8_t elvt_nb_puck;

/**
 * new_puck information (from filterbridge)
 */
extern uint8_t elvt_new_puck;

/**
 * elevator orders
 */
typedef enum elvt_order_e {CLOSE, PREPARE, OPEN} elvt_order_e;
extern elvt_order_e elvt_order_position;
extern uint8_t elvt_order_in_progress;
extern uint8_t elvt_degraded_mode;
extern uint8_t elvt_position_required;

/**
 * We prepare the elevator
*/
void
elvt_prepare(uint8_t pos);

/**
 * We open the elevator
*/
void
elvt_open(uint8_t pos);

/**
 * We open the elevator in degradad mode
*/
void
elvt_open_degraded(uint8_t pos);

/**
 * We close the elevator and go away
*/
void
elvt_close(void);

/**
 * conversion stop/millimeter
 */

#define ELEVATOR_MM_TO_STEP 72.34

/**
 * pwm constant for elevator doors
 */

#define OPEN_DOOR_PWM 0x200
#define CLOSE_DOOR_PWM -0x200
#define TIME_DOORS_PWM 0xB0
#define TIME_LIGHT_DOORS_PWM 0xaa

#endif // elevator_h
