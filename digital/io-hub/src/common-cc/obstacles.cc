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
#include "obstacles.hh"
#include "bot.hh"

#include "robot.hh"

extern "C" {
#include "modules/math/geometry/distance.h"
}

#ifdef TARGET_host
# include "robot.hh"
#endif

Obstacles::Obstacles ()
    : changed_ (false), report_rate_limit_cpt_ (0)
{
    for (int i = 0; i < obstacles_nb_; i++)
        obstacles_[i].valid = 0;
}

void
Obstacles::update ()
{
    for (int i = 0; i < obstacles_nb_; i++)
    {
        if (obstacles_[i].valid)
        {
            obstacles_[i].valid--;
            if (!obstacles_[i].valid)
                changed_ = true;
        }
    }
    if (changed_ && !report_rate_limit_cpt_)
    {
        vect_t o[obstacles_nb_];
        int o_nb = 0;
        for (int i = 0; i < obstacles_nb_; i++)
        {
            if (obstacles_[i].valid)
                o[o_nb++] = obstacles_[i].pos;
        }
#ifdef TARGET_host
        SimuReport &r = robot->hardware.simu_report;
        r.pos (o, o_nb, 0);
#endif
        robot->lcd.obstacles (o, o_nb);
        changed_ = false;
        report_rate_limit_cpt_ = report_rate_limit_;
    }
    if (report_rate_limit_cpt_)
        report_rate_limit_cpt_--;
}

void
Obstacles::add (const vect_t &pos)
{
    // Try to merge, and find the oldest on the way.
    int oldest_i = 0, oldest_i_valid = valid_new_;
    for (int i = 0; i < obstacles_nb_; i++)
    {
        if (obstacles_[i].valid)
        {
            if (distance_point_point (&pos, &obstacles_[i].pos) < same_mm_)
            {
                // Replace if older.
                if (obstacles_[i].valid < valid_new_)
                {
                    obstacles_[i].pos = pos;
                    obstacles_[i].valid = valid_new_;
                    changed_ = true;
                }
                return;
            }
        }
        if (obstacles_[i].valid < oldest_i_valid)
        {
            oldest_i = i;
            oldest_i_valid = obstacles_[i].valid;
        }
    }
    // If not found, replace the oldest.
    obstacles_[oldest_i].pos = pos;
    obstacles_[oldest_i].valid = valid_new_;
    changed_ = true;
}

void
Obstacles::add (const vect_t &pos, int slot)
{
    obstacles_[slot].pos = pos;
    obstacles_[slot].valid = valid_new_;
    changed_ = true;
}

bool
Obstacles::blocking (const vect_t &robot, const vect_t &dest, int stop_mm_) const
{
    if (stop_mm_ == -1)
        stop_mm_ = stop_mm;
    // Stop here if no obstacle.
    bool obs_valid = false;
    for (int i = 0; !obs_valid && i < obstacles_nb_; i++)
        obs_valid = obstacles_[i].valid != 0;
    if (!obs_valid)
        return false;
    // If destination is realy near, stop here.
    vect_t vd = dest; vect_sub (&vd, &robot);
    int d = vect_norm (&vd);
    if (d < epsilon_mm)
        return false;
    // If destination is near, use clearance to destination point instead of
    // stop length.
    vect_t t;
    if (d < stop_mm_)
        t = dest;
    else
    {
        vect_scale_f824 (&vd, (1ll << 24) / d * stop_mm_);
        t = robot;
        vect_translate (&t, &vd);
    }
    // Now, look at obstacles.
    for (int i = 0; i < obstacles_nb_; i++)
    {
        if (!obstacles_[i].valid)
            continue;
        // Vector from robot to obstacle.
        vect_t vo = obstacles_[i].pos; vect_sub (&vo, &robot);
        // Ignore if in our back.
        int dp = vect_dot_product (&vd, &vo);
        if (dp < 0)
            continue;
        // Check distance.
        int od = distance_segment_point (&robot, &t, &obstacles_[i].pos);
        if (od > BOT_SIZE_SIDE + clearance_mm / 2 + obstacle_radius_mm)
            continue;
        // Else, obstacle is blocking.
        return true;
    }
    return false;
}

void
Obstacles::add_obstacles (Path &path) const
{
    int index = 0;
    for (int i = 0; i < obstacles_nb_; i++)
    {
        if (obstacles_[i].valid)
        {
            path.obstacle (index++, obstacles_[i].pos,
                           obstacle_radius_mm /*+ clearance_mm + BOT_SIZE_SIDE*/);
        }
    }
}

