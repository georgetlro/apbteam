/* models.host.c */
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
#define _GNU_SOURCE 1 /* Need ISO C99 features as well. */
#include "common.h"

#include "modules/utils/utils.h"

#include "io.h"

#include "models.host.h"
#include "simu.host.h"

#include "aux_traj.h"

#include AC_ASSERV_CONTACTS_H

#include <math.h>
#include <string.h>

#define NO_CORNER { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }

static int
simu_table_test_guybrush (double p_x, double p_y);

static int
simu_table_test_apbirthday (double p_x, double p_y);

/* Guybrush, APBTeam 2012. */
static const struct robot_t guybrush_robot =
{
    /* Main motors. */
    &motor_model_def_faulhaber_2657_x9_7,
    /* Motors voltage (V). */
    24.0,
    /* Number of steps on the main motors encoders. */
    2500,
    /* Wheel radius (m). */
    0.065 / 2,
    /* Distance between the wheels (m). */
    0.16,
    /* Weight of the robot (kg). */
    10.0,
    /* Distance of the gravity center from the center of motors axis (m). */
    0.0,
    /* Whether the encoder is mounted on the main motor (false) or not (true). */
    1,
    /** Encoder wheel radius (m). */
    0.063 / 2,
    /** Distance between the encoders wheels (m). */
    0.28,
    /** No auxiliary motors. */
    { }, { }, { },
    /** Sensor update function. */
    NULL,
    /** Table test function, return false if given robot point is not in
     * table. */
    simu_table_test_guybrush,
    /** Robot corners, from front left, then clockwise. */
    { { 150, 171.5 }, { 150, -171.5 }, { -135, -121.5 }, { -135, 121.5 },
	{ 150, 0 } },
    /** Initialisation function. */
    NULL,
};

/* APBirthday, APBTeam 2013. TODO: update base dimensions. */
static const struct robot_t apbirthday_robot =
{
    /* Main motors. */
    &motor_model_def_faulhaber_2657_x9_7,
    /* Motors voltage (V). */
    24.0,
    /* Number of steps on the main motors encoders. */
    2500,
    /* Wheel radius (m). */
    0.065 / 2,
    /* Distance between the wheels (m). */
    0.16,
    /* Weight of the robot (kg). */
    10.0,
    /* Distance of the gravity center from the center of motors axis (m). */
    0.0,
    /* Whether the encoder is mounted on the main motor (false) or not (true). */
    1,
    /** Encoder wheel radius (m). */
    0.063 / 2,
    /** Distance between the encoders wheels (m). */
    0.28,
    /** No auxiliary motors. */
    { }, { }, { },
    /** Sensor update function. */
    NULL,
    /** Table test function, return false if given robot point is not in
     * table. */
    simu_table_test_apbirthday,
    /** Robot corners, from front left, then clockwise. */
    { { 96, 142 }, { 96, -142 }, { -126, -142 }, { -126, 81 },
	{ -80, 142 } },
    /** Initialisation function. */
    NULL,
};

/* Table of models. */
static const struct
{
    const char *name;
    const struct robot_t *robot;
} models[] = {
      { "guybrush", &guybrush_robot },
      { "apbirthday", &apbirthday_robot },
      { 0, 0 }
};

/** Get a pointer to a model by name, or return 0. */
const struct robot_t *
models_get (const char *name)
{
    int i;
    for (i = 0; models[i].name; i++)
      {
	if (strcmp (models[i].name, name) == 0)
	    return models[i].robot;
      }
    return 0;
}

/** Initialise simulation models. */
void
models_init (const struct robot_t *robot, motor_model_t *main_motor_left,
	     motor_model_t *main_motor_right, motor_model_t aux_motor[])
{
    int i;
    if (main_motor_left)
      {
	main_motor_left->m = *robot->main_motor;
	main_motor_left->m.J = robot->weight * robot->wheel_r
	    * robot->wheel_r / 2;
	main_motor_left->h = ECHANT_PERIOD;
	main_motor_left->d = 1000;
      }
    if (main_motor_right)
      {
	main_motor_right->m = *robot->main_motor;
	main_motor_right->m.J = robot->weight * robot->wheel_r
	    * robot->wheel_r / 2;
	main_motor_right->h = ECHANT_PERIOD;
	main_motor_right->d = 1000;
      }
    if (aux_motor)
      {
	for (i = 0; i < AC_ASSERV_AUX_NB; i++)
	  {
	    if (robot->aux_motor[i])
	      {
		aux_motor[i].m = *robot->aux_motor[i];
		aux_motor[i].m.J = robot->aux_load[i];
		aux_motor[i].h = ECHANT_PERIOD;
		aux_motor[i].d = 1000;
	      }
	  }
      }
    if (robot->init)
	robot->init (robot, main_motor_left, main_motor_right, aux_motor);
}

/* Table test for Guybrush. */
int
simu_table_test_guybrush (double p_x, double p_y)
{
    static const double table_width = 3000.0, table_height = 2000.0;
    double x, y;
    simu_compute_absolute_position (p_x, p_y, &x, &y);
    if (x < 0 || y < 0 || x >= table_width || y >= table_height)
	return 0;
    /* Totems. */
    if (((x >= 1500 - 400 - 125 && x < 1500 - 400 + 125)
	 || (x >= 1500 + 400 - 125 && x < 1500 + 400 + 125))
	&& (y >= 1000 - 125 && y < 1000 + 125))
	return 0;
    /* Boats. */
    double bx = 325 + (400 - 325) * y / 1500. + 22;
    if (((x >= bx - 40 && x < bx) || (x >= 3000 - bx && x < 3000 - bx + 40))
	&& y < 740)
	return 0;
    return 1;
}

/* Table test for APBirthday. */
int
simu_table_test_apbirthday (double p_x, double p_y)
{
    static const double table_width = 3000.0, table_height = 2000.0;
    double x, y;
    simu_compute_absolute_position (p_x, p_y, &x, &y);
    if (x < 0 || y < 0 || x >= table_width || y >= table_height)
	return 0;
    /* Sideboards. */
    if ((x < 400 || x >= table_width - 400)
	&& (y < 100 || y >= table_height - 100))
	return 0;
    /* Cake. */
    double cake_dx = table_width / 2 - x;
    double cake_dy = table_height - y;
    if (cake_dx * cake_dx + cake_dy * cake_dy < 500 * 500)
	return 0;
    return 1;
}

