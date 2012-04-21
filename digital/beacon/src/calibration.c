/* calibration.c */
/* Beacon servomotor calibration. {{{
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

#include <appTimer.h>
#include "debug_avr.h"
#include "servo.h"
#include "calibration.h"

HAL_AppTimer_t calibrationTimer;		// TIMER descripor used by the DEBUG task
calibration_s calibration;


/* This function iniatializes the calibration structure*/
void calibration_init_structure(void)
{
	/* Calibration initial values */
	calibration.state = CALIBRATION_INIT;
	calibration.laser_flag = CLEAR;
}

/* This function starts the calibration task */
void calibration_start_task(void)
{
	calibrationTimer.interval = CALIBRATION_FAST_TASK_PERIOD;
	calibrationTimer.mode     = TIMER_REPEAT_MODE;
	calibrationTimer.callback = calibration_task;
	calibration_init_structure();
	servo_structure_init();
	HAL_StartAppTimer(&calibrationTimer);
}

/* This function stops the calibration task */
void calibration_stop_task(void)
{
	HAL_StopAppTimer(&calibrationTimer);
}

/* This function starts or stops the calibration task depending on the current state */
void calibration_start_stop_task(void)
{
	static bool task_step = 0;
	if(task_step == 0)
	{
		calibration_start_task();
		task_step = 1;
	}
	else
	{
		calibration_stop_task();
		task_step = 0;
	}
}

/* This function restarts the calibration task changing its frequency */
void calibration_change_task_frequency(uint32_t frequency)
{
	HAL_StopAppTimer(&calibrationTimer);
	calibrationTimer.interval = frequency;
	calibrationTimer.mode     = TIMER_REPEAT_MODE;
	calibrationTimer.callback = calibration_task;
	HAL_StartAppTimer(&calibrationTimer);
}


/* Calibration task is used to calibrated the servo motors in order to run the laser parallel to the table */
void calibration_task(void)
{	
	TServo_ID servo;
	static uint16_t top = 0;
	static uint16_t bottom = 0;

	/* Select which servo need to be calibrated */
	if(servo_get_state(SERVO_1) == servo_get_state(SERVO_2))
		servo = SERVO_1;
	else
		servo = SERVO_2;
	
	switch(calibration.state)
	{
		case CALIBRATION_INIT:
			
			/* Go directly to next step */
			calibration.state = CALIBRATION_FAST_SCANNING;
			break;
			
		case CALIBRATION_FAST_SCANNING:
			
			/* Check if the laser catchs something */
			if(calibration_get_laser_flag() == CLEAR) 
			{
				/* Nothing appended, update the state and continue scanning */
				servo_set_state(servo,SERVO_SCANNING_FAST_IN_PROGRESS);
				calibration_scanning(servo);
			}
			else
			{
				/* Clear the laser flag */
				calibration_set_laser_flag(CLEAR);
				
				/* Laser detected the aim, reset the servo with the previous value and update the servo status */
				servo_set_value(servo,servo_get_value(servo)+FAST_SCANNING_OFFSET);
				servo_set_state(servo,SERVO_SCANNING_FAST_FINISHED);
				
				if((servo_get_state(SERVO_1) == SERVO_SCANNING_FAST_FINISHED) && (servo_get_state(SERVO_2) == SERVO_SCANNING_FAST_FINISHED))
				{
					/* If both servo have finished FAST SCANNING go to SLOW SCANNING state */
					calibration.state = CALIBRATION_SLOW_SCANNING;
					
					/* For better accuracy change task frequency for SLOW SCANNING */
					calibration_change_task_frequency(CALIBRATION_SLOW_TASK_PERIOD);
				}
			}
			break;
			
		case CALIBRATION_SLOW_SCANNING:
			
			/* Check if the laser catched something */
			if(calibration_get_laser_flag() == CLEAR)
			{
				/* Nothing appended, update the state and continue scanning */
				calibration_scanning(servo);
				servo_set_state(servo,SERVO_SCANNING_FAST_IN_PROGRESS);
			}
			else
			{
				/* Clear the laser flag */
				calibration_set_laser_flag(CLEAR);
				
				/* Check if it TOP AIM was already found */
				if(top == 0)
				{
					/* If no save the value */
					top = servo_get_value(servo);
				}
				else
				{
					/* If top TOP AIM was already found, avarage the TOP & BOTTOM value and update the servo status */
					bottom = servo_get_value(servo);
					servo_set_value(servo,(top+bottom)/2);
					servo_set_state(servo,SERVO_SCANNING_SLOW_FINISHED);
					top = 0;
					bottom = 0;
					
					/* If both servo have finished FAST SCANNING go to SLOW SCANNING*/
					if((servo_get_state(SERVO_1) == SERVO_SCANNING_SLOW_FINISHED) && (servo_get_state(SERVO_2) == SERVO_SCANNING_SLOW_FINISHED))
					{
						calibration.state = SCANNING_STATE_CALIBRATED;
						calibration_stop_task();
					}
				}
			}
			break;
		case SCANNING_STATE_CALIBRATED:
			break;
		default:
			break;
	}
}

/* This function drives the defined servo motor in order to scan verticaly */
void calibration_scanning(TServo_ID servo_id)
{
	uint16_t next_value = 0;
	uint16_t current_value = 0;
	
	/* Compute next value to set to the servo */
	next_value = servo_get_value(servo_id) + servo_get_scanning_sense(servo_id)*SCANNING_STEP;

	/* Set it and check the return value in order to inverse the sense MIN or MAX is reached */
	current_value = servo_set_value(servo_id,next_value);
	
	if((current_value <= SERVO_ANGLE_MIN) || (current_value >= SERVO_ANGLE_MAX))
	{
		servo_inverse_scanning_sense(servo_id);
	}
}

/* This function sets the laser flag according the given value SET or CLEAR */
void calibration_set_laser_flag(TLaser_flag_type value)
{
	calibration.laser_flag = value;
}

/* This function returns the laser flag */
TLaser_flag_type calibration_get_laser_flag(void)
{
	return calibration.laser_flag;
}

/* This function returns the calibration state */
TCalibration_state calibration_get_state(void)
{
	return calibration.state;
}
