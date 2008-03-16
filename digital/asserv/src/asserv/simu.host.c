/* simu.host.c */
/* asserv - Position & speed motor control on AVR. {{{
 *
 * Copyright (C) 2006 Nicolas Schodet
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
#include "simu.host.h"

#include "modules/host/host.h"
#include "modules/utils/utils.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pwm.h"

#include "motor_model.host.h"
#include "models.host.h"

/** Simulate some AVR regs. */
uint8_t DDRF, PORTC, PORTD, PORTE, PORTF, PORTG, PINC;

/** Overall counter values. */
uint16_t counter_left, counter_right, counter_aux0;
/** Counter differences since last update.
 * Maximum of 7 significant bits, sign included. */
int16_t counter_left_diff, counter_right_diff, counter_aux0_diff;

/** PWM values, this is an error if absolute value is greater than the
 * maximum. */
int16_t pwm_left, pwm_right, pwm_aux0;
/** PWM reverse directions. */
uint8_t pwm_reverse;

/* Robot model. */
const struct robot_t *simu_robot;

/** Motor models. */
struct motor_t simu_left_model, simu_right_model, simu_aux0_model;

/** Computed simulated position (mm, rad). */
double simu_pos_x, simu_pos_y, simu_pos_a;

/** Full counter values. */
uint32_t simu_counter_left, simu_counter_right, simu_counter_aux0;
double simu_counter_left_th, simu_counter_right_th;

/** Initialise simulation. */
static void
simu_init (void)
{
    int argc;
    char **argv;
    host_get_program_arguments (&argc, &argv);
    if (argc != 1)
      {
	fprintf (stderr, "need model name as first argument\n");
	exit (1);
      }
    simu_robot = models_get (argv[0]);
    if (!simu_robot)
      {
	fprintf (stderr, "unknown model name: %s\n", argv[0]);
	exit (1);
      }
    models_init (simu_robot, &simu_left_model, &simu_right_model,
		 &simu_aux0_model);
    simu_pos_x = simu_pos_y = simu_pos_a = 0;
}

/** Update simulation position. */
static void
simu_pos_update (double dl, double dr, double footing)
{
    double d = 0.5 * (dl + dr);
    double da = (dr - dl) / footing;
    double na = simu_pos_a + da;
    if (da < 0.0001 && da > -0.0001)
      {
	/* Avoid a division by zero when angle is too small. */
	double a = simu_pos_a + da * 0.5;
	simu_pos_x += d * cos (a);
	simu_pos_y += d * sin (a);
      }
    else
      {
	/* Radius of turn is d / da. */
	simu_pos_x += (sin (na) - sin (simu_pos_a)) * d / da;
	simu_pos_y += (cos (simu_pos_a) - cos (na)) * d / da;
      }
    simu_pos_a = na;
}

/** Do a simulation step. */
static void
simu_step (void)
{
    double old_left_th, old_right_th, old_aux0_th;
    /* Convert pwm value into voltage. */
    assert (pwm_left >= -PWM_MAX && pwm_left <= PWM_MAX);
    assert (pwm_right >= -PWM_MAX && pwm_right <= PWM_MAX);
    assert (pwm_aux0 >= -PWM_MAX && pwm_aux0 <= PWM_MAX);
    simu_left_model.u = simu_left_model.m.u_max
	* ((double) pwm_left / (PWM_MAX + 1));
    simu_right_model.u = simu_right_model.m.u_max
	* ((double) pwm_right / (PWM_MAX + 1));
    simu_aux0_model.u = simu_aux0_model.m.u_max
	* ((double) pwm_aux0 / (PWM_MAX + 1));
    /* Make one step. */
    old_left_th = simu_left_model.th;
    old_right_th = simu_right_model.th;
    old_aux0_th = simu_aux0_model.th;
    motor_model_step (&simu_left_model);
    motor_model_step (&simu_right_model);
    if (simu_robot->aux0_motor)
	motor_model_step (&simu_aux0_model);
    /* Modify counters. */
    uint32_t counter_left_new;
    uint32_t counter_right_new;
    if (!simu_robot->encoder_separated)
      {
	counter_left_new = simu_left_model.th / (2*M_PI)
	    * simu_robot->main_encoder_steps;
	counter_right_new = simu_right_model.th / (2*M_PI)
	    * simu_robot->main_encoder_steps;
      }
    else
      {
	/* Thanks Thales. */
	double left_diff = (simu_left_model.th - old_left_th)
	    / simu_left_model.m.i_G * simu_robot->wheel_r;
	double right_diff = (simu_right_model.th - old_right_th)
	    / simu_right_model.m.i_G * simu_robot->wheel_r;
	double sum = left_diff + right_diff;
	double diff = (left_diff - right_diff)
	    * (simu_robot->encoder_footing / simu_robot->footing);
	double left_enc_diff = 0.5 * (sum + diff);
	double right_enc_diff = 0.5 * (sum - diff);
	simu_counter_left_th += left_enc_diff / simu_robot->encoder_wheel_r;
	simu_counter_right_th += right_enc_diff / simu_robot->encoder_wheel_r;
	counter_left_new = simu_counter_left_th / (2*M_PI)
	    * simu_robot->main_encoder_steps;
	counter_right_new = simu_counter_right_th / (2*M_PI)
	    * simu_robot->main_encoder_steps;
      }
    /* Update an integer counter. */
    counter_left_diff = counter_left_new - simu_counter_left;
    counter_left += counter_left_diff;
    simu_counter_left = counter_left_new;
    counter_right_diff = counter_right_new - simu_counter_right;
    counter_right += counter_right_diff;
    simu_counter_right = counter_right_new;
    /* Update auxiliary counter. */
    if (simu_robot->aux0_motor)
      {
	uint32_t counter_aux0_new = simu_aux0_model.th / (2*M_PI)
	    * simu_robot->aux0_encoder_steps;
	counter_aux0_diff = counter_aux0_new - simu_counter_aux0;
	counter_aux0 += counter_aux0_diff;
	simu_counter_aux0 = counter_aux0_new;
      }
    /* Update position */
    simu_pos_update ((simu_left_model.th - old_left_th)
		     / simu_left_model.m.i_G * simu_robot->wheel_r * 1000,
		     (simu_right_model.th - old_right_th)
		     / simu_right_model.m.i_G * simu_robot->wheel_r * 1000,
		     simu_robot->footing * 1000);
}

/** Initialise the timer. */
void
timer_init (void)
{
    simu_init ();
}

/** Wait for timer overflow. */
void
timer_wait (void)
{
    simu_step ();
}

/** Read timer value. Used for performance analysis. */
uint8_t
timer_read (void)
{
    return 0;
}

/** Initialize the counters. */
void
counter_init (void)
{
}

/** Update overall counter values and compute diffs. */
void
counter_update (void)
{
}

/** Initialise PWM generator. */
void
pwm_init (void)
{
}

/** Update the hardware PWM values. */
void
pwm_update (void)
{
}

void
eeprom_read_params (void)
{
}

void
eeprom_write_params (void)
{
}

void
eeprom_clear_params (void)
{
}

void
pwm_set_reverse (uint8_t reverse)
{
    pwm_reverse = reverse;
}

