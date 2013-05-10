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
#include "plate.hh"

Plate::Plate ()
{
    reset_plate_nb ();
    is_up = 0;
}

inline void Plate::arm_down ()
{
    robot->hardware.cherry_plate_down.set (true);
    robot->hardware.cherry_plate_up.set (false);
}

inline void Plate::arm_up ()
{
    robot->hardware.cherry_plate_down.set (false);
    robot->hardware.cherry_plate_up.set (true);
}

inline void Plate::clamp_open ()
{
    robot->hardware.cherry_plate_clamp_close.set (false);
    robot->hardware.cherry_plate_clamp_open.set (true);
}

inline void Plate::clamp_close ()
{
    robot->hardware.cherry_plate_clamp_close.set (true);
    robot->hardware.cherry_plate_clamp_open.set (false);
}

FSM_STATES (PLATE_OFF,
            PLATE_WAITING_PRESSURE,
            PLATE_INIT_PREPARE,
            PLATE_INIT_TAKING,
            PLATE_INIT_UPING,
            PLATE_INIT_DOWNING,
            PLATE_READY,
            PLATE_TAKE_GLUE,
            PLATE_TAKE_UPING,
            PLATE_I_HAZ_PLATE,
            PLATE_DROP_DOWNING,
            PLATE_DROP_OPENING
            )

FSM_EVENTS (plate_take,
            plate_taken,
            plate_drop,
            plate_droped)

FSM_START_WITH (PLATE_OFF)

FSM_TRANS (PLATE_OFF, init_actuators, PLATE_WAITING_PRESSURE)
{
}

FSM_TRANS_TIMEOUT (PLATE_WAITING_PRESSURE, 1250, PLATE_INIT_PREPARE)
{
    Plate::arm_down ();
    Plate::clamp_open ();
}

FSM_TRANS_TIMEOUT (PLATE_INIT_PREPARE, 100, PLATE_INIT_TAKING)
{
    Plate::clamp_close ();
}

FSM_TRANS_TIMEOUT (PLATE_INIT_TAKING, 100, PLATE_INIT_UPING)
{
    Plate::arm_up ();
}

FSM_TRANS_TIMEOUT (PLATE_INIT_UPING, 100, PLATE_INIT_DOWNING)
{
    Plate::arm_down ();
}

FSM_TRANS_TIMEOUT (PLATE_INIT_DOWNING, 100, PLATE_READY)
{
    Plate::clamp_open ();
}

FSM_TRANS (PLATE_READY, plate_drop, PLATE_READY)
{
    robot->fsm_queue.post (FSM_EVENT (plate_droped));
}

FSM_TRANS (PLATE_READY, plate_take, PLATE_TAKE_GLUE)
{
    Plate::clamp_close ();
}

FSM_TRANS_TIMEOUT (PLATE_TAKE_GLUE, 100, PLATE_TAKE_UPING)
{
    Plate::arm_up ();
}

FSM_TRANS_TIMEOUT (PLATE_TAKE_UPING, 300, PLATE_I_HAZ_PLATE)
{
    robot->fsm_queue.post (FSM_EVENT (plate_taken));
    robot->plate.ppp ();
    robot->plate.is_up = 1;
}

FSM_TRANS (PLATE_I_HAZ_PLATE, plate_take, PLATE_I_HAZ_PLATE)
{
    robot->fsm_queue.post (FSM_EVENT (plate_taken));
}

FSM_TRANS (PLATE_I_HAZ_PLATE, plate_drop, PLATE_DROP_DOWNING)
{
    Plate::arm_down ();
    robot->plate.is_up = 0;
}

FSM_TRANS_TIMEOUT (PLATE_DROP_DOWNING, 100, PLATE_DROP_OPENING)
{
    Plate::clamp_open ();
}

FSM_TRANS_TIMEOUT (PLATE_DROP_OPENING, 100, PLATE_READY)
{
    robot->fsm_queue.post (FSM_EVENT (plate_droped));
}
