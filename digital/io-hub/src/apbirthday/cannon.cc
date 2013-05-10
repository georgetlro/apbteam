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

void
Cannon::fire (int duration)
{
    if (duration < 1)
        duration = 1;
    else if (duration > 3)
        duration = 3;
    duration_ = duration;
    ANGFSM_HANDLE (AI, cannon_fire);
}

inline void Cannon::blower_on (int speed)
{
    // Start the blower
    // speed is between 0-256 (max->min)
    robot->pot_regul.set_wiper(0, 256);
    robot->pot_regul.set_wiper(1, speed);
}

inline void Cannon::blower_off ()
{
    // Shutdown the blower
    robot->pot_regul.set_wiper (0, 0);
}

inline void Cannon::set_servo_pos (int pos)
{
    // Switch the servo to BLOCK, POS1 or POS2
    robot->hardware.servos.set_position (Servo::SERVO_CHERRY, pos);
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

// Cannon main FSM

FSM_STATES (CANNON_OFF,
            CANNON_PURGING,
            CANNON_READY,
            CANNON_FIRING)

FSM_START_WITH (CANNON_OFF)

FSM_TRANS (CANNON_OFF, init_actuators, CANNON_PURGING)
{
    // Start the blower to purge the canon
    robot->cannon.blower_on (0);
}

FSM_TRANS_TIMEOUT (CANNON_PURGING, 500, CANNON_READY)
{
    // Stop the blower
    robot->cannon.blower_off ();
}

FSM_TRANS (CANNON_READY, cannon_fire, CANNON_FIRING)
{
    // Start the blower
    Cannon::blower_on (23);
    // Start RGB sensor
    robot->rgb.calibrate_cannon_sensor ();
    robot->rgb.start_cannon_color ();
}

FSM_TRANS_TIMEOUT (CANNON_FIRING, 1750,
                   again, CANNON_FIRING,
                   end, CANNON_READY)
{
    robot->cannon.duration_--;
    if (robot->cannon.duration_)
    {
        return ANGFSM_BRANCH (again);
    }
    else
    {
        // Stop the blower
        Cannon::blower_off ();
        // Stop the RGB sensor
        robot->rgb.stop_cannon_color ();
        robot->fsm_queue.post (FSM_EVENT (cannon_fire_ok));
        return ANGFSM_BRANCH (end);
    }
}


