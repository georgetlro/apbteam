#ifndef candles_hh
#define candles_hh

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

/* The cake is a lie !

-----------------------------------------------------------
-  8      0                                    7     19   -
 -                                                        -
  -   9      1                             6       18    -
    -                                                   -
      -   10      2                   5         17    -
        -                3      4                   -
          -    11                        16      -
             -       12   13   14   15      -
                    -     ______    -
 */

class Candles
{
    public:
        Candles (int calif_mode);
        void blow (int candle);
        void deduce ();
        enum State
        {
            UNPUNCHED,
            PUNCHED,
        };
        enum Floor
        {
            NEAR = 0,
            FAR = 1,
            FLOOR_NB
        };
        enum Color
        {
            RED = TEAM_COLOR_RIGHT,
            BLUE = TEAM_COLOR_LEFT,
            WHITE,
            UNKNOWN,
        };
        static bool is_near (int pos);
        static bool is_far (int pos);
        // GPIO manipulation.
        static void deploy_arm ();
        static void undeploy_arm_1 ();
        static void undeploy_arm_2 ();
        static void undeploy_arm_3 ();
        static void arm_out ();
        static void arm_back ();
        static void push_near ();
        static void unpush_near ();
        static void push_far ();
        static void unpush_far ();
        static void flamby_arm ();
        static void flamby_far ();
        static void crampe_arm ();
        static void crampe_far ();
        static void servo_near_deploy ();
        static void servo_near_undeploy ();
    private:
        static const int total_count = 20;
        static const int near_count = 12;
        static const int far_count = 8;
    public:
        State state[total_count];
        Color color[total_count];
        int actual_pos[FLOOR_NB];
};

#endif // candles_hh
