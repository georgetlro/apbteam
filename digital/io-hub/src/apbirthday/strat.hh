#ifndef strat_hh
#define strat_hh
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
#include "defs.hh"
#include "playground_2013.hh"
#include "asserv.hh"

/// High level strategy decision making.
class Strat
{
  public:
    enum Decision
    {
        CANDLES,
        PLATE,
        CANNON,
        GIFTS,
    };
    /// Information on a candle decision.
    struct CandlesDecision
    {
        /// Movement direction, 1 (trigo) or -1 (antitrigo).
        int dir_sign;
        /// Angle relative to cake to end the movement.
        uint16_t end_angle;
    };
    /// Information on a plate decision.
    struct PlateDecision
    {
        /// Chosen plate.
        int plate;
        /// Leave movement after plate is taken.
        bool leave;
        /// Approach position, where the robot should be before starting
        /// approaching.
        Position approaching_pos;
        /// Loading position, point where to go backward to load the plate. If
        /// the point is reached, there is no plate.
        vect_t loading_pos;
    };
    /// Information on a gift decision.
    struct GiftsDecision
    {
        /// Need to go to begin_pos first.
        bool go_first;
        /// Begin of movement position, only used if go_first.
        vect_t begin_pos;
        /// End of movement position.
        vect_t end_pos;
        /// Movement direction.
        Asserv::DirectionConsign dir;
    };
    /// Number of plates.
    static const int plate_nb = 10;
  public:
    /// Constructor.
    Strat ();
    /// Color specific initialisation.
    void color_init ();
    /// Return new decision and associated position.
    Decision decision (Position &pos);
    /// Take a decision related to candles, return false to give up candles.
    bool decision_candles (CandlesDecision &decision,
                           uint16_t robot_angle);
    /// Take a decision related to plate.
    void decision_plate (PlateDecision &decision);
    /// Take a decision related to gifts.
    void decision_gifts (GiftsDecision &decision);
    /// Report a failure to apply the previous decision.
    void failure ();
    /// Report a success.
    void success ();
  private:
    /// Compute best score for a plate.
    int score_plate (Position &pos);
  private:
    /// Last taken decision.
    Decision last_decision_;
    /// Number of candles tries.
    int candles_tries_;
    /// Last plate decision.
    PlateDecision plate_decision_;
    /// Last gifts decision.
    GiftsDecision gifts_decision_;
    /// Plate visited?
    bool plate_visited_[plate_nb];
};

#endif // strat_hh
