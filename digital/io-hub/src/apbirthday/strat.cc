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

Strat::Strat ()
{
    for (int i = 0; i < plate_nb; i++)
        plate_visited_[i] = false;
}

void
Strat::color_init ()
{
    if (team_color)
        plate_visited_[1] = false;
    else
        plate_visited_[5 + 1] = false;
}

int
Strat::score_plate (Position &pos)
{
    static const int plate_app = pg_plate_size_border + BOT_SIZE_RADIUS + 20;
    static const int plate_load = pg_plate_size_border + BOT_SIZE_BACK - 40;
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

Strat::Decision
Strat::decision (Position &pos)
{
    Decision best_decision;
    int best_score = -1;
    Position tpos;
    int tscore;
    // Plate?
    tscore = score_plate (tpos);
    if (tscore > best_score)
    {
        best_score = tscore;
        pos = tpos;
        best_decision = PLATE;
    }
    // Good score, XXX temp hack.
    if (best_score >= 100000)
    {
        last_decision_ = best_decision;
        return best_decision;
    }
    // TODO: this is a stub.
    static int step;
    if (step > 2)
        step = 0;
    switch (step++)
    {
    case 1:
        gifts_decision_.go_first = true;
        gifts_decision_.begin_pos = (vect_t) { 600, pg_gifts_distance
            + BOT_SIZE_SIDE };
        gifts_decision_.end_pos = (vect_t) { 2400, pg_gifts_distance
            + BOT_SIZE_SIDE };
        gifts_decision_.dir = Asserv::FORWARD;
        pos.v = (vect_t) { 900, pg_gifts_distance + BOT_SIZE_SIDE };
        pos.a = 0;
        last_decision_ = GIFTS;
        return GIFTS;
    default:
    case 0:
        pos.v = pg_cake_pos;
        pos.a = 0;
        last_decision_ = CANDLES;
        return CANDLES;
    case 2:
        if (team_color)
            pos = pg_position_deg (1500, 1200, 90);
        else
            pos = pg_position_deg (1500 - 200, 1200, 90);
        last_decision_ = CANNON;
        return CANNON;
    }
}

/// Compute score for candles between first and last.
static int
strat_candles_score (int first, int last)
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
    // Make an evaluation of the best direction to follow.
    // TODO: +1/-1 until candles at ends can be reached.
    int limit, score_forward, score_backward;
    limit = top_candle_for_angle (robot_angle, Candles::FAR, 1);
    score_backward = strat_candles_score (0 + 1, limit);
    score_forward = strat_candles_score (limit + 1, 7 - 1);
    limit = top_candle_for_angle (robot_angle, Candles::NEAR, 1);
    score_backward += strat_candles_score (8 + 1, limit);
    score_forward += strat_candles_score (limit + 1, 19 - 1);
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
}

