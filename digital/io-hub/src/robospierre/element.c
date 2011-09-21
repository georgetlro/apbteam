/* element.c */
/* io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
 *
 * Copyright (C) 2011 Jérôme Jutteau
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
#include "element.h"

#include "modules/utils/utils.h"
#include "modules/math/geometry/distance.h"
#include "modules/math/geometry/geometry.h"

#include "chrono.h"
#include "logistic.h"
#include "bot.h"
#include "playground.h"

/** Offset to be used when everything fail. */
static uint8_t failure_offset_s;

/** Elements on table. */
struct element_t element_table[] =
{
    /*
       20 elements on intersections.
       To be symmetric, alternate % 2.
       See ELEMENT_INTERSEC_START and ELEMENT_INTERSEC_END
       ELEMENT_INTERSEC_END don't takes central pawn.
    */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 2 * 350, 5 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0}, /* top left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 2 * 350, 5 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0}, /* top right */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 1 * 350, 5 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0}, /* middle left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 1 * 350, 5 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0}, /* middle right */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 2 * 350, 4 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0}, /* 2nd line left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 2 * 350, 4 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 1 * 350, 4 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 1 * 350, 4 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 2 * 350, 3 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0}, /* 3th line left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 2 * 350, 3 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 1 * 350, 3 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 1 * 350, 3 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 2 * 350, 2 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0}, /* 4th line left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 2 * 350, 2 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 1 * 350, 2 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 1 * 350, 2 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 30, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 2 * 350, 1 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, 0, 0, 0}, /* 5th line left */
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 2 * 350, 1 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 - 1 * 350, 1 * 350}, ELEMENT_INTERSEC | ELEMENT_LEFT, -99, 0, 0},
    {ELEMENT_NONE | ELEMENT_PAWN, {1500 + 1 * 350, 1 * 350}, ELEMENT_INTERSEC | ELEMENT_RIGHT, -99, 0, 0},

    /* Central pawn. (see ELEMENT_CENTRAL_PAWN) */
    {ELEMENT_PAWN, {1500, 3 * 350}, ELEMENT_INTERSEC | ELEMENT_CENTER, 30, 0, 0},

    /*
       10 elements on green zones.
       To be symmetric, alternate % 2.
       See ELEMENT_GREEN_START and ELEMENT_GREEN_END
    */
    {ELEMENT_ANY, {200, 10 + 280 * 5}, ELEMENT_GREEN |ELEMENT_LEFT, 110, 0, 0}, /* top left */
    {ELEMENT_ANY, {3000 - 200, 10 + 280 * 5}, ELEMENT_GREEN | ELEMENT_RIGHT, 110, 0, 0}, /* top right */
    {ELEMENT_ANY, {200, 10 + 280 * 4}, ELEMENT_GREEN |ELEMENT_LEFT, 100, 0, 0}, /* 2nd line left */
    {ELEMENT_ANY, {3000 - 200, 10 + 280 * 4}, ELEMENT_GREEN | ELEMENT_RIGHT, 100, 0, 0}, /* 2nd line right */
    {ELEMENT_ANY, {200, 10 + 280 * 3}, ELEMENT_GREEN |ELEMENT_LEFT, 110, 0, 0}, /* ... */
    {ELEMENT_ANY, {3000 - 200, 10 + 280 * 3}, ELEMENT_GREEN | ELEMENT_RIGHT, 110, 0, 0},
    {ELEMENT_ANY, {200, 10 + 280 * 2}, ELEMENT_GREEN |ELEMENT_LEFT, 100, 0, 0},
    {ELEMENT_ANY, {3000 - 200, 10 + 280 * 2}, ELEMENT_GREEN | ELEMENT_RIGHT, 100, 0, 0},
    {ELEMENT_ANY, {200, 10 + 280 * 1}, ELEMENT_GREEN |ELEMENT_LEFT, -100, 0, 0},
    {ELEMENT_ANY, {3000 - 200, 10 + 280 * 1}, ELEMENT_GREEN | ELEMENT_RIGHT, -100, 0, 0},

    /*
       36 elements in the middle of a square.
       Altern colors in order to retrieve position % 2.
       See ELEMENT_UNLOAD_START and ELEMENT_UNLOAD_END
    */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 2 * 350 - 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, -100, 0, 0}, /* Top left blue */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 1 * 350 - 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, -100, 10, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 0 * 350 - 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, -100, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 0 * 350 + 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, -100, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 1 * 350 + 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, -100, 10, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 2 * 350 + 175, 5 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, -100, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 2 * 350 - 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0}, /* 2nd line left red */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 1 * 350 - 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT | ELEMENT_BONUS, 0, 10, 0}, /* bonus */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 0 * 350 - 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 0 * 350 + 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 1 * 350 + 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT | ELEMENT_BONUS, 0, 10, 0}, /* bonus */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 2 * 350 + 175, 4 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 2 * 350 - 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0}, /* 3th line left blue */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 1 * 350 - 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 0 * 350 - 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 0 * 350 + 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 1 * 350 + 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 2 * 350 + 175, 3 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 2 * 350 - 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0}, /* 4th line left red */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 1 * 350 - 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT | ELEMENT_BONUS, 0, 4, 0}, /* bonus */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 0 * 350 - 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 0 * 350 + 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 1 * 350 + 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT | ELEMENT_BONUS, 0, 4, 0}, /* bonus */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 2 * 350 + 175, 2 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 2 * 350 - 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0}, /* 5th line left blue */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 1 * 350 - 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 0 * 350 - 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 0 * 350 + 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 1 * 350 + 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_RIGHT, 0, 0, 0},
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 2 * 350 + 175, 1 * 350 + 175}, ELEMENT_CENTER | ELEMENT_LEFT, 0, 0, 0},
    {ELEMENT_NONE, {1500 - 2 * 350 - 175, 175}, ELEMENT_CENTER | ELEMENT_SAFE | ELEMENT_LEFT, 0, -100, 0}, /* left red */
    {ELEMENT_NONE, {1500 - 1 * 350 - 175, 175}, ELEMENT_CENTER | ELEMENT_SAFE | ELEMENT_RIGHT, 0, -100, 0}, /* left blue */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 - 175, 175}, ELEMENT_CENTER | ELEMENT_LEFT | ELEMENT_BONUS, 0, 9, 0}, /* middle bonus left, red. */
    {ELEMENT_NONE | ELEMENT_ANY, {1500 + 175, 175}, ELEMENT_CENTER | ELEMENT_RIGHT | ELEMENT_BONUS, 0, 9, 0}, /* middle bonus right, blue. */
    {ELEMENT_NONE, {1500 + 1 * 350 + 175, 175}, ELEMENT_CENTER | ELEMENT_SAFE | ELEMENT_LEFT, 0, -100, 0}, /* right red */
    {ELEMENT_NONE, {1500 + 2 * 350 + 175, 175}, ELEMENT_CENTER | ELEMENT_SAFE | ELEMENT_RIGHT, 0, -100, 0} /* right blue */
};

inline void
element_set (uint8_t element_id, element_t element)
{
    assert (element_id < UTILS_COUNT (element_table));
    element_table[element_id] = element;
}

void
element_init ()
{
    /* Set NONE in middle of our squares and keep ELEMENT_NONE | ELEMENT_ANY
       in others square middle. */
    uint8_t i;
    element_t e;
    for (i = ELEMENT_UNLOAD_START; i <= ELEMENT_UNLOAD_END; i++)
      {
	e = element_get (i);
	if ((team_color == TEAM_COLOR_LEFT &&
	     (e.attr & ELEMENT_LEFT)) ||
	    (team_color == TEAM_COLOR_RIGHT &&
	     (e.attr & ELEMENT_RIGHT)))
	  {
	    e.type = ELEMENT_NONE;
	    element_set (i, e);
	  }
      }

    /* Negative bonus for the other green zone at start. */
    /* Do not touch last green emplacement. */
    for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END - 2; i++)
      {
	    element_t e = element_get (i);
	    if (!((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
		  (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT))))
	      {
		if (e.bonus_load > 0)
		  {
		    e.bonus_load *= -1;
		    element_set (i, e);
		  }
	      }
      }
}

int32_t
element_unload_score (position_t robot_pos, uint8_t element_id)
{
    int32_t score = 0;
    assert (element_id < UTILS_COUNT (element_table));
    element_t e = element_get (element_id);

    /* If it is not an unload zone, quit. */
    if (!((e.attr & ELEMENT_CENTER) || (e.attr & ELEMENT_INTERSEC)))
	return -1;

    /* Unload color. */
    if (!(
	  (e.attr & ELEMENT_CENTER) &&
	  ((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT))
	   ||(team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
	 ))
	return -1;

    /* If there is already something here, do not score. */
    if (e.type != ELEMENT_NONE)
	return -1;

    /* Failed element. */
    if (e.failure_until_s
	&& e.failure_until_s + failure_offset_s < (int) (chrono_remaining_time () / 1000))
	return -1;

    /* Bonus adjust. */
    score += e.bonus_unload * ELEMENT_BONUS_COEFF;

    /* Unload distance. */
    /* TODO: minimal distance may not be the best choice. */
    vect_t v = e.pos;
    int32_t dr = distance_point_point (&v, &robot_pos.v);
    score += 4242 - dr;

    /* Alignment with the robot. */
    if (dr > 100)
      {
	vect_t vr = v;
	vect_sub (&vr, &robot_pos.v);
	vect_t u;
	uint16_t a;
	if (logistic_global.collect_direction == DIRECTION_FORWARD)
	    a = robot_pos.a;
	else
	    a = robot_pos.a + G_ANGLE_UF016_DEG (180);
	vect_from_polar_uf016 (&u, 100, a);
	int32_t dp = vect_dot_product (&u, &vr);
	int32_t align = dp / dr;
	score += align;
      }

    return score;
}

uint8_t
element_unload_best (position_t robot_pos)
{
    uint8_t i;
    uint8_t best = 0xff;
    int32_t score, best_score = 0;
    uint8_t retry = 0;
    do
      {
	if (retry && failure_offset_s != 255)
	    failure_offset_s++;
	for (i = ELEMENT_UNLOAD_START;
	     i <= ELEMENT_UNLOAD_END;
	     i++)
	  {
	    score = element_unload_score (robot_pos , i);
	    if (best == 0xff || best_score < score)
	      {
		best = i;
		best_score = score;
	      }
	  }
      } while (best_score == -1 && retry++ < 10);
    return best;
}

int32_t
element_score (position_t robot_pos, uint8_t element_id)
{
    int32_t score = 0;
    assert (element_id < UTILS_COUNT (element_table));
    element_t e = element_get (element_id);

    if (e.attr & ELEMENT_SAFE)
	return -1;

    if ((e.attr & ELEMENT_CENTER) &&
	((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
	 (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT))))
	return -1;

    if (e.type == ELEMENT_NONE &&
	((e.attr & ELEMENT_INTERSEC) ||(e.attr & ELEMENT_GREEN)))
	return -1;

    /* Failed element. */
    if (e.failure_until_s
	&& e.failure_until_s + failure_offset_s < (int) (chrono_remaining_time () / 1000))
	return -1;

    /* Forbidden element. */
    if (e.bonus_load == -100)
	return -1;

    if (e.type == ELEMENT_KING)
	score += ELEMENT_KING_SCORE;
    else if (e.type == ELEMENT_QUEEN)
	score += ELEMENT_QUEEN_SCORE;
    else if (e.type == ELEMENT_PAWN)
	score += ELEMENT_PAWN;
    else if (e.type ==  ELEMENT_ANY)
	score += ELEMENT_ANY_SCORE;
    if (e.type & ELEMENT_NONE)
	score /= 2;

    /* Add score modifier. */
    score += e.bonus_load * ELEMENT_BONUS_COEFF;

    /* We are sure of this element. */
    if (!(e.type & ELEMENT_NONE) &&
	(e.attr == ELEMENT_INTERSEC || e.attr == ELEMENT_CENTER) &&
	element_id != ELEMENT_CENTRAL_PAWN)
	score += score / 4;

    /* Distance from the robot. */
    vect_t v = e.pos;
    int32_t dr = distance_point_point (&v, &robot_pos.v);
    score += (4242 - dr);

    /* Alignment with the robot. */
    if (dr > 100)
      {
	vect_t vr = v;
	vect_sub (&vr, &robot_pos.v);
	vect_t u;
	uint16_t a;
	if (logistic_global.collect_direction == DIRECTION_FORWARD)
	    a = robot_pos.a;
	else
	    a = robot_pos.a + G_ANGLE_UF016_DEG (180);
	vect_from_polar_uf016 (&u, 100, a);
	int32_t dp = vect_dot_product (&u, &vr);
	int32_t align = dp / dr;
	score += align;
      }

    return score;
}

uint32_t
element_proba (uint8_t element_id)
{
    assert (element_id < UTILS_COUNT (element_table));
    element_t e = element_get (element_id);
    uint32_t p_t = chrono_remaining_time () * 500 / CHRONO_MATCH_DURATION_MS;
    uint8_t p_pos;
    int32_t out = 0;

    /* Intersections. */
    if (e.attr & ELEMENT_INTERSEC)
      {
	/* Element on our side ? */
	if ((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
	    (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
	  {
	    p_pos = 20 - element_id / 2;
	    out = p_pos * p_t;
	  }
	/* Element on the other side ? */
	else
	  {
	    p_pos = element_id / 2 + 1;
	    out = p_pos * p_t;
	  }
      }
    /* Green zone. */
    else if (e.attr & ELEMENT_GREEN)
      {
	/* In our side. */
	if ((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
	    (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
	  {
	    p_pos = 5 - ((element_id - ELEMENT_GREEN_START) / 2);
	    out = p_t * p_pos;

	  }
	/* Other side. */
	else
	  {
	    p_pos = (element_id - ELEMENT_GREEN_START) / 2 + 1;
	    out =  p_t * p_pos;
	  }

      }
    /* Central pawn */
    else if (e.type == ELEMENT_CENTRAL_PAWN)
	out = p_t;
    /* Centre of squares. */
    else if (e.attr & ELEMENT_CENTER)
      {
	/* In our side. */
	if ((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
	    (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
	    out = 0;
	/* Other side. */
	else
	  {
	    p_pos = (element_id - ELEMENT_UNLOAD_START) / 2 + 1;
	    out =  (1000 - p_t) * p_pos;
	  }

      }
    else
	out = 0;
    return out;
}

uint8_t
element_best (position_t robot_pos)
{
    uint8_t i;
    uint8_t best = 0xff;
    int32_t score = 0, best_score = 0;
    uint8_t retry = 0;
    do
      {
	if (retry && failure_offset_s != 255)
	    failure_offset_s++;
	for (i = 0; i < UTILS_COUNT (element_table); i++)
	  {
	    score = element_score (robot_pos ,i);
	    if (best == 0xff || best_score < score)
	      {
		best = i;
		best_score = score;
	      }
	  }
      } while (best_score == -1 && retry++ < 10);
    return best;
}

void
element_not_here (uint8_t element_id)
{
    assert (element_id < UTILS_COUNT (element_table));
    element_t e = element_get (element_id);
    e.type = ELEMENT_NONE;
    element_set (element_id, e);

    /* Invalidate the same element to the other side. */
    if (e.attr & ELEMENT_INTERSEC)
      {
	uint8_t other_side_id = element_opposed (element_id);
	element_t other_side = element_get (other_side_id);
	other_side.type = ELEMENT_NONE;
	element_set (other_side_id, other_side);
      }

    /* If the element is on an intersection, try to guess last elements. */
    element_intersec_symetric (element_id, ELEMENT_NONE);
}

inline void
element_intersec_symetric (uint8_t element_id, uint8_t element_type)
{
    static uint8_t element_columns[2][5] =
      {
	  {
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN
	  },
	  {
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN,
	    ELEMENT_NONE | ELEMENT_PAWN
	  }
      };
    element_t e = element_get (element_id);

    /* if the element is on an intersection, try to guess last elements. */
    if ((e.attr & ELEMENT_INTERSEC) && !(e.attr & ELEMENT_CENTER))
      {
	uint8_t cpt_none = 0;
	uint8_t cpt_pawn = 0;
	uint8_t col;
	uint8_t i;
	uint8_t type = 0;

	if (element_id % 4 <= 1)
	    col = 0;
	else
	    col = 1;

	/* Nothing to see. */
	if (element_columns[col][4] != (ELEMENT_NONE | ELEMENT_PAWN))
	    return;

	/* Count. */
	for (i = 0; i < 5; i++)
	  {
	    if (element_columns[col][i] == ELEMENT_PAWN || element_type == ELEMENT_PAWN)
		cpt_pawn++;
	    if (element_columns[col][i] == ELEMENT_NONE || element_type == ELEMENT_NONE)
		cpt_none++;
	    if (element_columns[col][i] == (ELEMENT_NONE | ELEMENT_PAWN))
	      {
		element_columns[col][i] = element_type;
		break;
	      }
	  }

	/* With which element are we going to fill the rest ? */
	if (cpt_pawn == 2)
	    type = ELEMENT_NONE;
	else if (cpt_none == 3)
	    type = ELEMENT_PAWN;
	else
	    return;

	for (i = 0; i < 5; i++)
	    if (element_columns[col][i] == (ELEMENT_NONE | ELEMENT_PAWN))
		element_columns[col][i] = type;

	/* Complete if possible. */
	for (i = ELEMENT_INTERSEC_START; i < ELEMENT_INTERSEC_END; i++)
	  {
	    uint8_t col_i;
	    if (i % 4 <= 1)
		col_i = 0;
	    else
		col_i = 1;
	    if (col_i == col)
	      {
		element_t el = element_get (i);
		element_t sym = element_get (element_opposed (i));
		if (el.type == (ELEMENT_NONE | ELEMENT_PAWN))
		  {
		    el.type = type;
		    element_set (i, el);
		    /* Set opposed. */
		    if (sym.type == (ELEMENT_NONE | ELEMENT_PAWN))
		      {
			sym.type = type;
			element_set (element_opposed (i), sym);
		      }
		  }
	      }
	  }
      }
}

void
element_taken (uint8_t element_id, uint8_t element_type)
{
    assert (element_id < UTILS_COUNT (element_table));
    static uint8_t pawn_c = 3, queen_c = 1, king_c = 1, any_nb = 5;
    uint8_t other_side_id, other_side_id_element = 0;

    if (element_type != ELEMENT_PAWN && element_type != ELEMENT_QUEEN && element_type != ELEMENT_KING)
	return;

    /* Set element. */
    element_t e = element_get (element_id);
    e.type = ELEMENT_NONE;
    element_set (element_id, e);

    /* Deduce symmetric position. */
    if ((e.attr & ELEMENT_INTERSEC) || (e.attr & ELEMENT_GREEN))
      {
	other_side_id_element = element_opposed (element_id);
	element_t other_side = element_get (other_side_id_element);
	if (other_side.type != ELEMENT_NONE)
	  {
	    other_side.type = element_type;
	    element_set (other_side_id_element, other_side);
	  }
      }

    /* If the element is on an intersection, try to guess last elements. */
    element_intersec_symetric (element_id, element_type);

    /* If the element is in the green zone, try to guess last elements. */
    if ((e.attr & ELEMENT_GREEN) && any_nb > 0)
      {
	uint8_t i;
	if (element_type == ELEMENT_PAWN)
	    pawn_c--;
	else if (element_type == ELEMENT_QUEEN)
	    queen_c--;
	else if (element_type == ELEMENT_KING)
	    king_c--;
	any_nb--;

	/* If there is something to guess. */
	if (any_nb <= 3)
	  {
	    /* All others are pawns. */
	    if (!queen_c && !king_c)
	      {
		for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END; i++)
		  {
		    element_t el = element_get (i);
		    if (el.type == ELEMENT_ANY)
		      {
			el.type = ELEMENT_PAWN;
			element_set (i, el);
			any_nb--;
			/* Set opposed side. */
			other_side_id = element_opposed (element_id);
			element_t other_side = element_get (other_side_id);
			if (other_side.type != ELEMENT_NONE &&
			    other_side_id != other_side_id_element)
			  {
			    other_side.type = ELEMENT_PAWN;
			    element_set (other_side_id, other_side);
			  }
		      }
		  }
	      }
	    /* All others are unidentified heads */
	    else if (pawn_c == 0 && any_nb == 2)
	      {
		for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END; i++)
		  {
		    element_t el = element_get (i);
		    if (el.type == ELEMENT_ANY)
		      {
			el.type = ELEMENT_HEAD;
			element_set (i, el);
			/* Set opposed side. */
			other_side_id = element_opposed (element_id);
			element_t other_side = element_get (other_side_id);
			if (other_side.type != ELEMENT_NONE &&
			    other_side_id != other_side_id_element)
			  {
			    other_side.type = ELEMENT_HEAD;
			    element_set (other_side_id, other_side);
			  }
		      }
		  }
	      }
	    /* Last element. */
	    if (any_nb == 1)
	      {
		uint8_t last_type;
		if (pawn_c == 1) last_type = ELEMENT_PAWN;
		else if (queen_c == 1) last_type = ELEMENT_QUEEN;
		else last_type = ELEMENT_KING;
		for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END; i++)
		  {
		    element_t el = element_get (i);
		    if (el.type == ELEMENT_ANY || el.type == ELEMENT_HEAD)
		      {
			el.type = last_type;
			element_set (i, el);
			any_nb--;
			/* Set opposed side. */
			other_side_id = element_opposed (i);
			element_t other_side = element_get (other_side_id);
			if (other_side.type != ELEMENT_NONE &&
			    other_side_id != other_side_id_element)
			  {
			    other_side.type = last_type;
			    element_set (other_side_id, other_side);
			  }
			break;
		      }
		  }
	      }
	  }
      }
}

void
element_down (uint8_t element_id, uint8_t element_type)
{
    uint8_t i;
    element_t e = element_get (element_id);
    e.type = element_type;
    element_set (element_id, e);

    /* Malus elements near this element. */
    for (i = ELEMENT_INTERSEC_START; i <= ELEMENT_INTERSEC_END; i++)
      {
	element_t ie = element_get (i);
	if (UTILS_ABS (e.pos.x - ie.pos.x)
	    < BOT_ELEMENT_RADIUS + BOT_SIZE_SIDE + 20
	    && UTILS_ABS (e.pos.y - ie.pos.y)
	    < BOT_ELEMENT_RADIUS + BOT_SIZE_SIDE + 20)
	  {
	    ie.bonus_load = -50;
	    element_set (i, ie);
	  }
      }
}

void
element_failure (uint8_t element_id)
{
    element_t e = element_get (element_id);
    e.failure_until_s = chrono_remaining_time () / 1000 - 7;
    element_set (element_id, e);
}

uint8_t
element_nearest_element_id (position_t robot_pos)
{
    uint8_t i;
    uint8_t id = 0;
    int32_t distance = 4242;
    element_t e;
    for (i = 0; i < UTILS_COUNT (element_table); i++)
      {
	e = element_get (i);
	vect_t v = e.pos;
	int32_t dr = distance_point_point (&v, &robot_pos.v);
	if (dr < distance)
	  {
	    id = i;
	    distance = dr;
	  }
      }
    return id;
}

uint8_t
element_opposed (uint8_t element_id)
{
    uint8_t op = 0xff;
    element_t e = element_get (element_id);
    if ((e.attr & ELEMENT_GREEN) ||(e.attr & ELEMENT_INTERSEC))
      {
	if (e.attr & ELEMENT_LEFT)
	    op = element_id + 1;
	else
	    op = element_id - 1;
      }
    return op;
}

vect_t
element_get_pos (uint8_t element_id)
{
    element_t e = element_get (element_id);
    vect_t pos;
    pos = e.pos;
    if (e.attr == (ELEMENT_GREEN |ELEMENT_RIGHT))
      {
	/* To the right. */
	pos.x = PG_WIDTH - BOT_GREEN_ELEMENT_DISTANCE_MM;
      }
    if (e.attr == (ELEMENT_GREEN |ELEMENT_LEFT))
      {
	/* To the left. */
	pos.x = BOT_GREEN_ELEMENT_DISTANCE_MM;
      }
    return pos;
}

uint8_t
element_blocking (uint8_t element_id, uint8_t escape)
{
    element_t e = element_get (element_id);
    return e.type == ELEMENT_TOWER || (!escape && e.type == ELEMENT_PAWN);
}

uint8_t
element_blocking_path (vect_t a, vect_t b, int16_t ab, uint8_t escape)
{
     uint8_t i;
     element_t e;
     /* Only unload area are blocking. */
     for (i = ELEMENT_UNLOAD_START; i <= ELEMENT_UNLOAD_END; i++)
       {
	 e = element_get (i);
	 if (e.type == ELEMENT_TOWER || (!escape && e.type == ELEMENT_PAWN))
	   {
	     /* Compute square of distance to obstacle, see
	      * distance_segment_point in modules/math/geometry for the method
	      * explanation. */
	     int32_t absq = (int32_t) ab * ab;
	     vect_t vab = b; vect_sub (&vab, &a);
	     vect_t vao = e.pos; vect_sub (&vao, &a);
	     int32_t dp = vect_dot_product (&vab, &vao);
	     int32_t dsq;
	     if (dp < 0)
	       {
		 dsq = vect_dot_product (&vao, &vao);
	       }
	     else if (dp > absq)
	       {
		 vect_t vbo = e.pos; vect_sub (&vbo, &b);
		 dsq = vect_dot_product (&vbo, &vbo);
	       }
	     else
	       {
		 vect_t vabn = vab; vect_normal (&vabn);
		 dsq = vect_dot_product (&vabn, &vao) / ab;
		 dsq *= dsq;
	       }
	     /* Compare with square of authorised distance. */
	     if (dsq < (int32_t) (BOT_ELEMENT_RADIUS + BOT_SIZE_SIDE + 20) *
		 (BOT_ELEMENT_RADIUS + BOT_SIZE_SIDE + 20))
		 return 1;
	   }
       }
     return 0;
}

void
element_i_like_green ()
{
    /* Negative bonus for the other green zone at start. */
    /* Do not touch last green emplacement. */
    int i;
    for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END - 2; i++)
      {
	    element_t e = element_get (i);
	    if (!((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
		  (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
		&& e.bonus_load < 0
		&& (e.type == ELEMENT_KING || e.type == ELEMENT_QUEEN))
		  {
		    e.bonus_load = 40;
		    element_set (i, e);
		  }
      }
}

void
element_no_more_green (void)
{
    uint8_t i;
    /* Remove our green zone score at first unload. */
    for (i = ELEMENT_GREEN_START; i <= ELEMENT_GREEN_END - 2; i++)
      {
	element_t e = element_get (i);
	if ((team_color == TEAM_COLOR_LEFT && (e.attr & ELEMENT_LEFT)) ||
	    (team_color == TEAM_COLOR_RIGHT && (e.attr & ELEMENT_RIGHT)))
	  {
	    e.bonus_load = 0;
	    element_set (i, e);
	  }
      }
}
