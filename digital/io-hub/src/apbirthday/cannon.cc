// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Maxime Hadjinlian
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
#include "cannon.hh"

Cannon::Cannon ()
{
    // Init the cannon and the RGB sensor
}

inline void Cannon::blower_on ()
{
    // Start the blower
    robot->pot_regul.set_wiper(0, 256);
}

inline void Cannon::blower_off ()
{
    // Shutdown the blower
    robot->pot_regul.set_wiper (0, 0);
}

inline void Cannon::set_servo_pos (int pos)
{
    // Switch the servo to BLOCK, POS1 or POS2
}

inline void Cannon::set_router_state (int state)
{
    // Set router to OPEN or CLOSE
}

// Trap FSM
FSM_STATES (CANNON_TRAP_OFF,
            CANNON_TRAP_BLOCK,
            CANNON_TRAP_MOVE_1,
            CANNON_TRAP_MOVE_2)

FSM_EVENTS (cannon_fire,
            cannon_fire_ok)

FSM_START_WITH (CANNON_TRAP_OFF)

FSM_TRANS (CANNON_TRAP_OFF, init_actuators, CANNON_TRAP_BLOCK)
{
    Cannon::set_servo_pos (Cannon::BLOCK);
}

FSM_TRANS (CANNON_TRAP_BLOCK, cannon_fire, CANNON_TRAP_MOVE_1)
{
    Cannon::set_servo_pos (Cannon::POS1);
}

FSM_TRANS_TIMEOUT (CANNON_TRAP_MOVE_1, 75, CANNON_TRAP_MOVE_2)
{
    Cannon::set_servo_pos (Cannon::POS2);
}

FSM_TRANS_TIMEOUT (CANNON_TRAP_MOVE_2, 75, CANNON_TRAP_MOVE_1)
{
    Cannon::set_servo_pos (Cannon::POS1);
}

FSM_TRANS (CANNON_TRAP_MOVE_1, cannon_fire_ok, CANNON_TRAP_BLOCK)
{
    Cannon::set_servo_pos (Cannon::BLOCK);
}

FSM_TRANS (CANNON_TRAP_MOVE_2, cannon_fire_ok, CANNON_TRAP_BLOCK)
{
    Cannon::set_servo_pos (Cannon::BLOCK);
}


// Router FSM

FSM_STATES (CANNON_ROUTER_OFF,
            CANNON_ROUTER_SLEEPING,
            CANNON_ROUTER_TRACKING,
            CANNON_ROUTER_PUSHING)

FSM_START_WITH (CANNON_ROUTER_OFF)

FSM_TRANS (CANNON_ROUTER_OFF, init_actuators, CANNON_ROUTER_SLEEPING)
{
    robot->cannon.set_router_state (Cannon::OPEN);
}

FSM_TRANS (CANNON_ROUTER_SLEEPING, cannon_fire, CANNON_ROUTER_TRACKING)
{
    // We do nothing here
}

FSM_TRANS (CANNON_ROUTER_TRACKING, cannon_fire_ok, CANNON_ROUTER_SLEEPING)
{
   // We do nothing here.
}

FSM_TRANS_TIMEOUT (CANNON_ROUTER_TRACKING, 250,
                    nothing_to_push, CANNON_ROUTER_TRACKING,
                    destroy, CANNON_ROUTER_PUSHING)
{
    // Check if we have something to push or not
    // return FSM_BRANCH(nothing_to_push);
    return FSM_BRANCH(destroy);
    // If we have something to push, push next event
}

FSM_TRANS_TIMEOUT (CANNON_ROUTER_PUSHING, 250, CANNON_ROUTER_TRACKING)
{
    // Open the router after the timeout
    robot->cannon.set_router_state (Cannon::OPEN);
}

FSM_TRANS (CANNON_ROUTER_PUSHING, cannon_fire_ok, CANNON_ROUTER_SLEEPING)
{
    // Open router
    robot->cannon.set_router_state (Cannon::OPEN);
}

// Cannon main FSM

FSM_STATES (CANNON_OFF,
            CANNON_PURGING,
            CANNON_READY,
            CANNON_FIRING)

FSM_START_WITH (CANNON_OFF)

FSM_TRANS (CANNON_OFF, init_actuators, CANNON_PURGING)
{
    // Start the blower to purge the canon
    robot->cannon.blower_on ();
}

FSM_TRANS_TIMEOUT (CANNON_PURGING, 500, CANNON_READY)
{
    // Stop the blower
    robot->cannon.blower_off ();
}

FSM_TRANS (CANNON_READY, cannon_fire, CANNON_FIRING)
{
    // Start the blower
    Cannon::blower_on ();
    // Start RGB sensor
}

FSM_TRANS_TIMEOUT (CANNON_FIRING, 1250, CANNON_READY)
{
    // Stop the blower
    Cannon::blower_off ();
    // Stop the RGB sensor
    robot->fsm_queue.post (FSM_EVENT (cannon_fire_ok));
}


