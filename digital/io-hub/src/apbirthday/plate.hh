#ifndef plate_hh
#define plate_hh

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

class Plate
{
    public:
        Plate ();
        int get_plate_nb ();
        void reset_plate_nb ();
        // Increment plate_nb, internal use for FSM
        void ppp ();
        // Read only: 1 if plate is up, 0 otherwhise
        int is_up;
        // GPIO manipulation.
        static void arm_down ();
        static void arm_up ();
        static void clamp_open ();
        static void clamp_close ();
    private:
        int nb_plate;
};

inline int Plate::get_plate_nb ()
{
    return nb_plate;
}

inline void Plate::reset_plate_nb ()
{
    nb_plate = 0;
}

inline void Plate::ppp ()
{
    nb_plate++;
}

#endif // plate_hh
