// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Jerome Jutteau
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

#include "robot.hh"
#include "defs.hh"
#include "drinks.hh"

extern "C" {
#define ANGFSM_NAME AI
#include "angfsm.h"
}

Drinks::Drinks ()
{
    nb_ = 0;
}

inline int Drinks::nb ()
{
    return nb_;
}

inline void Drinks::add ()
{
    robot->drinks.nb_++;
}

inline void Drinks::clear ()
{
    robot->drinks.nb_ = 0;
}

inline void Drinks::upper_open ()
{
    robot->hardware.glass_upper_clamp_close.set (false);
    robot->hardware.glass_upper_clamp_open.set (true);
}

inline void Drinks::upper_close ()
{
    robot->hardware.glass_upper_clamp_close.set (true);
    robot->hardware.glass_upper_clamp_open.set (false);
}

inline void Drinks::upper_up ()
{
    robot->hardware.glass_upper_clamp_up.set (true);
    robot->hardware.glass_upper_clamp_down.set (false);
}

inline void Drinks::upper_down ()
{
    robot->hardware.glass_upper_clamp_up.set (false);
    robot->hardware.glass_upper_clamp_down.set (true);
}

inline void Drinks::upper_unleach ()
{
    robot->hardware.glass_upper_clamp_up.set (false);
    robot->hardware.glass_upper_clamp_down.set (false);
}

inline void Drinks::lower_open ()
{
    robot->hardware.glass_lower_clamp_close.set (false);
    robot->hardware.glass_lower_clamp_open.set (true);
}

inline void Drinks::lower_close ()
{
    robot->hardware.glass_lower_clamp_close.set (true);
    robot->hardware.glass_lower_clamp_open.set (false);
}

FSM_STATES (DRINKS_OFF,
            DRINKS_WAIT_PRESSURE,
            DRINKS_INIT_PREPARING,
            DRINKS_INIT_UPING,
            DRINKS_INIT_DOWNING,
            DRINKS_READY,
            DRINKS_TAKE_GLUE,
            DRINKS_TAKE_DOWNING,
            DRINKS_TAKE_FIXING_UP,
            DRINKS_SERVE_SERVING,
            DRINKS_SERVE_LIBERATING
            )

FSM_EVENTS (drinks_take,
            drinks_taken,
            drinks_serve,
            drinks_served)

FSM_START_WITH (DRINKS_OFF)

FSM_TRANS (DRINKS_OFF, init_actuators, DRINKS_WAIT_PRESSURE)
{
}

FSM_TRANS_TIMEOUT (DRINKS_WAIT_PRESSURE, 1500,
                   off, DRINKS_OFF,
                   on, DRINKS_INIT_PREPARING)
{
    // TODO: disabled until present.
    if (1)
        return FSM_BRANCH (off);
    else
    {
        Drinks::upper_down ();
        Drinks::upper_close ();
        Drinks::lower_close ();
        return FSM_BRANCH (on);
    }
}

FSM_TRANS_TIMEOUT (DRINKS_INIT_PREPARING, 100, DRINKS_INIT_UPING)
{
    Drinks::upper_up ();
    Drinks::lower_open ();
}

FSM_TRANS_TIMEOUT (DRINKS_INIT_UPING, 100, DRINKS_INIT_DOWNING)
{
    Drinks::upper_down ();
}

FSM_TRANS_TIMEOUT (DRINKS_INIT_DOWNING, 100, DRINKS_READY)
{
    Drinks::upper_open ();
}

FSM_TRANS (DRINKS_READY, drinks_take,
           no_drinks, DRINKS_TAKE_DOWNING,
           has_drinks, DRINKS_TAKE_GLUE,
           too_much_drinks, DRINKS_READY)
{
    Drinks::lower_close ();
    int nb = robot->drinks.nb ();
    if (nb == 0)
        return FSM_BRANCH (no_drinks);
    else if (nb == 4)
        return FSM_BRANCH (too_much_drinks);
    else
        return FSM_BRANCH (has_drinks);
}

FSM_TRANS_TIMEOUT (DRINKS_TAKE_GLUE, 100, DRINKS_TAKE_DOWNING)
{
    Drinks::upper_unleach ();
    Drinks::upper_down ();
}

FSM_TRANS_TIMEOUT (DRINKS_TAKE_DOWNING, 100, DRINKS_TAKE_FIXING_UP)
{
    Drinks::upper_close ();
}

FSM_TRANS_TIMEOUT (DRINKS_TAKE_FIXING_UP, 100, DRINKS_READY)
{
    Drinks::upper_up ();
    if (robot->drinks.nb () < 4)
        Drinks::lower_open ();
    robot->drinks.add ();
    robot->fsm_queue.post (FSM_EVENT (drinks_taken));
}

FSM_TRANS (DRINKS_READY, drinks_serve,
           has_drinks, DRINKS_SERVE_SERVING,
           no_drinks, DRINKS_READY)
{
    if (robot->drinks.nb () > 0)
    {
        Drinks::upper_down ();
        Drinks::lower_close ();
        return FSM_BRANCH (has_drinks);
    }
    else
    {
        robot->fsm_queue.post (FSM_EVENT (drinks_served));
        return FSM_BRANCH (no_drinks);
    }
}

FSM_TRANS_TIMEOUT (DRINKS_SERVE_SERVING, 100, DRINKS_SERVE_LIBERATING)
{
    Drinks::upper_open ();
    Drinks::lower_open ();
}

FSM_TRANS_TIMEOUT (DRINKS_SERVE_LIBERATING, 100, DRINKS_READY)
{
    robot->drinks.clear ();
    robot->fsm_queue.post (FSM_EVENT (drinks_served));
}
