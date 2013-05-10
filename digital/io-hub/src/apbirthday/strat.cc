// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Nicolas Schodet
//
// APBTeam:
//        Web: http://apbteam.org/
//      Email: team AT apbteam DOT org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
// }}}
#include "strat.hh"
#include "robot.hh"
#include "top.hh"
#include "debug.host.hh"
#include "bot.hh"

static const vect_t plate_pos[Strat::plate_nb] = {
    { 200, 1750 },
    { 200, 1400 },
    { 200, 1000 },
    { 200, 600 },
    { 200, 250 },
    { 3000 - 200, 1750 },
    { 3000 - 200, 1400 },
    { 3000 - 200, 1000 },
    { 3000 - 200, 600 },
    { 3000 - 200, 250 },
};

static const int plate_app = pg_plate_size_border + BOT_SIZE_RADIUS + 20;
static const int plate_load = pg_plate_size_border + BOT_SIZE_BACK - 60;

Strat::Strat ()
{
}

void
Strat::color_init ()
{
    if (team_color)
        plate_visited_[1] = false;
    else
    {
        plate_visited_[5 + 1] = false;
        // TODO: plate out of reach.
        plate_visited_[0] = true;
        // TODO: this gift is out of reach.
        robot->gifts.open[3] = true;
    }
    // TODO: plate out of reach.
    plate_visited_[9] = true;
    // Pre-computed positions.
    pos_[POS_CANDLES] = pg_cake_pos;
    for (int i = 0; i < plate_nb; i++)
    {
        plate_visited_[i] = false;
        int sign = i < 5 ? 1 : -1;
        pos_[POS_PLATE_FIRST + i].x = plate_pos[i].x + sign * plate_app;
        pos_[POS_PLATE_FIRST + i].y = plate_pos[i].y + sign * 35;
    }
    if (team_color)
    {
        pos_[POS_CANNON].x = 1462;
        pos_[POS_CANNON].y = 1256;
    }
    else
    {
        pos_[POS_CANNON].x = 1642;
        pos_[POS_CANNON].y = 1276;
    }
    pos_[POS_GIFT_1].x = 900;
    pos_[POS_GIFT_1].y = pg_gifts_distance + BOT_SIZE_SIDE;
    pos_[POS_GIFT_2].x = 1500;
    pos_[POS_GIFT_2].y = pg_gifts_distance + BOT_SIZE_SIDE;
    pos_[POS_GIFT_3].x = 2100;
    pos_[POS_GIFT_3].y = pg_gifts_distance + BOT_SIZE_SIDE;
}

void
Strat::score_pos ()
{
    bool escape = false;
    Position current_pos = robot->asserv.get_position ();
    robot->path.reset ();
    robot->obstacles.add_obstacles (robot->path);
    robot->path.endpoints (current_pos.v, current_pos.v);
    robot->path.prepare_score (current_pos.v);
    for (int i = 0; i < POS_NB; i++)
    {
        robot->path.endpoints (current_pos.v, pos_[i]);
        pos_score_[i] = (int16_t) robot->path.get_score (pos_[i]);
        if (pos_score_[i] == -1)
            escape = true;
    }
    if (escape)
    {
        robot->path.prepare_score (current_pos.v, 8);
        for (int i = 0; i < POS_NB; i++)
        {
            if (pos_score_[i] == -1)
            {
                robot->path.endpoints (current_pos.v, pos_[i]);
                pos_score_[i] = (int16_t) robot->path.get_score (pos_[i]);
            }
        }
    }
}

int
Strat::score_candles (Position &pos)
{
    if (pos_score_[POS_CANDLES] == -1)
        return -1;
    // TODO: +1/-1 until candles at ends can be reached.
    int candles = candles_score (0 + 1, 7 - 1)
        + candles_score (8 + 1, 19 - 1);
    if (!candles)
        return -1;
    int score = 1000 * candles + 10000 - pos_score_[POS_CANDLES];
#ifdef TARGET_host
    robot->hardware.simu_report.draw_number (pg_cake_pos, score);
#endif
    pos.v = pg_cake_pos;
    pos.a = 0;
    return score;
}

int
Strat::score_plate (Position &pos)
{
    int plate = -1, score = -1;
    bool above = false, below = false;
    bool leave = true;
    // Important start points.
    if (team_color)
    {
        if (!plate_visited_[2])
        {
            plate = 2;
            below = true;
            score = 100000;
            leave = false;
        }
        else if (!plate_visited_[0])
        {
            plate = 0;
            above = true;
            score = 100000;
        }
    }
    else
    {
        if (!plate_visited_[0 + 5])
        {
            plate = 0 + 5;
            above = true;
            score = 100000;
            leave = false;
        }
        else if (!plate_visited_[2 + 5])
        {
            plate = 2 + 5;
            below = true;
            score = 100000;
        }
    }
#ifdef TARGET_host
    if (plate != -1)
        robot->hardware.simu_report.draw_number (plate_pos[plate], score);
#endif
    if (plate == -1 && robot->plate.get_plate_nb () < 2)
    {
        for (int i = 0; i < plate_nb; i++)
        {
            if (plate_visited_[i] || pos_score_[i] == -1)
                continue;
            int tscore = 10000 - pos_score_[i];
            // Prefer our side.
            if ((team_color && i >= 5)
                || (!team_color && i < 5))
                tscore -= 1500;
#ifdef TARGET_host
            robot->hardware.simu_report.draw_number (plate_pos[i], tscore);
#endif
            if (tscore > score)
            {
                plate = i;
                score = tscore;
            }
        }
    }
    // One plate chosen?
    if (plate != -1)
    {
        if (above)
        {
            pos.v.x = plate_pos[plate].x + 35;
            pos.v.y = plate_pos[plate].y - plate_app;
            pos.a = G_ANGLE_UF016_DEG (-90);
            plate_decision_.loading_pos.x = plate_pos[plate].x + 35;
            plate_decision_.loading_pos.y = plate_pos[plate].y - plate_load;
        }
        else if (below)
        {
            pos.v.x = plate_pos[plate].x - 35;
            pos.v.y = plate_pos[plate].y + plate_app;
            pos.a = G_ANGLE_UF016_DEG (90);
            plate_decision_.loading_pos.x = plate_pos[plate].x - 35;
            plate_decision_.loading_pos.y = plate_pos[plate].y + plate_load;
        }
        else if (plate < 5)
        {
            pos.v.x = plate_pos[plate].x + plate_app;
            pos.v.y = plate_pos[plate].y + 35;
            pos.a = G_ANGLE_UF016_DEG (0);
            plate_decision_.loading_pos.x = plate_pos[plate].x + plate_load;
            plate_decision_.loading_pos.y = plate_pos[plate].y + 35;
        }
        else
        {
            pos.v.x = plate_pos[plate].x - plate_app;
            pos.v.y = plate_pos[plate].y - 35;
            pos.a = G_ANGLE_UF016_DEG (180);
            plate_decision_.loading_pos.x = plate_pos[plate].x - plate_load;
            plate_decision_.loading_pos.y = plate_pos[plate].y - 35;
        }
        plate_decision_.approaching_pos = pos;
        plate_decision_.plate = plate;
        plate_decision_.leave = leave;
    }
    return score;
}

int
Strat::score_cannon (Position &pos)
{
    if (pos_score_[POS_CANNON] == -1)
        return -1;
    if (!robot->plate.get_plate_nb ())
        return -1;
    pos.v = pos_[POS_CANNON];
    pos.a = G_ANGLE_UF016_DEG (90);
    int score = 1000 * robot->plate.get_plate_nb ()
        + 10000 - pos_score_[POS_CANNON];
#ifdef TARGET_host
    robot->hardware.simu_report.draw_number (pos.v, score);
#endif
    return score;
}

int
Strat::score_gifts_sub (Position &pos, int gift_min, int gift_max,
                        int best_score)
{
    // Find first and last gifts.
    int min = -1, max = -1, nb = 0;
    for (int i = gift_min; i <= gift_max; i++)
    {
        if (!robot->gifts.open[i])
        {
            if (min == -1) min = i;
            max = i;
            nb++;
        }
    }
    if (!nb)
        return best_score;
    // Compute positions.
    int p;
    if (min <= 1) p = POS_GIFT_1;
    else if (min == 2) p = POS_GIFT_2;
    else p = POS_GIFT_3;
    // Compute score.
    if (pos_score_[p] == -1)
        return best_score;
    int score = 10000 - pos_score_[p] + nb * 1000;
#ifdef TARGET_host
    robot->hardware.simu_report.draw_number
        ((vect_t) { robot->gifts.x[gift_min], pg_gifts_distance }, score);
#endif
    // Update if better.
    if (score > best_score)
    {
        pos.v = pos_[p];
        pos.a = 0;
        if (min == 0)
        {
            gifts_decision_.go_first = true;
            gifts_decision_.begin_pos.x = robot->gifts.x[0]
                - bot_gift_arm_x;
            gifts_decision_.begin_pos.y = pg_gifts_distance + BOT_SIZE_SIDE;
        }
        else
        {
            gifts_decision_.go_first = false;
        }
        gifts_decision_.end_pos.x = robot->gifts.x[max] - bot_gift_arm_x;
        gifts_decision_.end_pos.y = pg_gifts_distance + BOT_SIZE_SIDE;
        return score;
    }
    else
        return best_score;
}

int
Strat::score_gifts (Position &pos)
{
    int best_score = -1;
    for (int i = 0; i < Gifts::nb; i++)
        best_score = score_gifts_sub (pos, i, Gifts::nb - 1, best_score);
    return best_score;
}

Strat::Decision
Strat::decision (Position &pos)
{
    Decision best_decision;
    int best_score = -1;
    Position tpos;
    int tscore;
    score_pos ();
#ifdef TARGET_host
    robot->hardware.simu_report.draw_start ();
#endif
    // Plate?
    tscore = score_plate (tpos);
    if (tscore > best_score)
    {
        best_score = tscore;
        pos = tpos;
        best_decision = PLATE;
    }
    // Cannon?
    tscore = score_cannon (tpos);
    if (tscore > best_score)
    {
        best_score = tscore;
        pos = tpos;
        best_decision = CANNON;
    }
    // Candles?
    tscore = score_candles (tpos);
    if (tscore > best_score)
    {
        best_score = tscore;
        pos = tpos;
        best_decision = CANDLES;
    }
    // Gifts?
    tscore = score_gifts (tpos);
    if (tscore > best_score)
    {
        best_score = tscore;
        pos = tpos;
        best_decision = GIFTS;
    }
#ifdef TARGET_host
    robot->hardware.simu_report.draw_send ();
#endif
    // Good score.
    if (best_score > 0)
    {
        last_decision_ = best_decision;
        return best_decision;
    }
    else
        return WAIT;
}

int
Strat::candles_score (int first, int last)
{
    int score = 0;
    Candles::Color other_color = team_color == TEAM_COLOR_RIGHT
        ? Candles::BLUE : Candles::RED;
    for (int i = first; i < last + 1; i++)
    {
        if (robot->candles.state[i] != Candles::PUNCHED
            && robot->candles.color[i] != other_color)
            score++;
    }
    return score;
}

bool
Strat::decision_candles (CandlesDecision &decision, uint16_t robot_angle)
{
    if (candles_tries_ > 5)
        return false;
    candles_tries_++;
    // Make an evaluation of the best direction to follow.
    // TODO: +1/-1 until candles at ends can be reached.
    int limit, score_forward, score_backward;
    limit = top_candle_for_angle (robot_angle, Candles::FAR, 1);
    score_backward = candles_score (0 + 1, limit);
    score_forward = candles_score (limit + 1, 7 - 1);
    limit = top_candle_for_angle (robot_angle, Candles::NEAR, 1);
    score_backward += candles_score (8 + 1, limit);
    score_forward += candles_score (limit + 1, 19 - 1);
    // Can not choose a direction with an obstacle.
    if (score_backward && top_follow_blocking (-1))
        score_backward = 0;
    if (score_forward && top_follow_blocking (1))
        score_forward = 0;
    // Now choose.
    host_debug ("score: forward = %d, backward = %d\n",
                score_forward, score_backward);
    if (score_forward == 0 && score_backward == 0)
    {
        return false;
    }
    else
    {
        // So near... let blow them...
        if (score_forward && robot_angle > G_ANGLE_UF016_DEG (-45))
            score_forward += 100;
        if (score_backward && robot_angle < G_ANGLE_UF016_DEG (180 + 45))
            score_backward += 100;
        // Compare.
        if (score_forward > score_backward)
        {
            decision.dir_sign = 1;
            decision.end_angle = G_ANGLE_UF016_DEG (-20);
        }
        else
        {
            decision.dir_sign = -1;
            decision.end_angle = G_ANGLE_UF016_DEG (180 + 20);
        }
        return true;
    }
}

void
Strat::decision_plate (PlateDecision &decision)
{
    decision = plate_decision_;
}

void
Strat::decision_gifts (GiftsDecision &decision)
{
    decision = gifts_decision_;
}

void
Strat::failure ()
{
    if (last_decision_ == PLATE)
        plate_visited_[plate_decision_.plate] = true;
}

void
Strat::success ()
{
    if (last_decision_ == PLATE)
        plate_visited_[plate_decision_.plate] = true;
    else if (last_decision_ == CANNON)
        robot->plate.reset_plate_nb ();
}

