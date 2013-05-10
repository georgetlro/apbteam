#ifndef gifts_hh
#define gifts_hh
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
#include "playground_2013.hh"

/// Keep gift status.
class Gifts
{
  public:
    /// Constructor.
    Gifts ()
    {
        for (int i = 0; i < nb; i++)
            open[i] = false;
    }
    /// Update gifts positions according to team color.
    void compute_pos ()
    {
        int sign = team_color ? -1 : 1;
        for (int i = 0; i < nb; i++)
            x[i] = 600 * (i + 1) + sign *  pg_gift_width / 2;
    }
  public:
    /// Number of gifts.
    static const int nb = 4;
    /// Are gifts open?
    bool open[nb];
    /// Gifts x positions.
    int x[nb];
};

#endif // gifts_hh
