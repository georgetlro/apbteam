#ifndef cannon_hh
#define cannon_hh

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

class Cannon
{
    public:
        Cannon ();
        enum ServoPos
        {
            BLOCK = 0x0800,
            POS1 = 0x1290,
            POS2 = 0x0a00
        };
        enum StateRouter
        {
            OPEN,
            CLOSE
        };
        // GPIO manipulation.
        static void blower_off ();
        static void blower_on ();
        static void set_servo_pos (int pos);
};

#endif // cannon_hh
