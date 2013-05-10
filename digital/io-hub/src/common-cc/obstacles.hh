#ifndef obstacles_hh
#define obstacles_hh
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
#include "path.hh"

/// Handle obstacles "database".
///
/// At each cycle, obstacle detections can come and go.  This class will keep
/// tracks of previously seen obstacles, and merge new detection events with
/// past history.
///
/// This class also decide if an obstacle is in the robot way.
class Obstacles
{
  public:
    /// Constructor.
    Obstacles ();
    /// To be called at each cycle.
    void update ();
    /// Add an obstacle detection.
    void add (const vect_t &pos);
    /// Add an obstacle detection, to specific slot.
    void add (const vect_t &pos, int slot);
    /// Return true if there is an obstacle near the robot while going to a
    /// destination point.
    bool blocking (const vect_t &robot, const vect_t &dest, int stop_mm_ = -1)
        const;
    /// Setup obstacles in path finding object.
    void add_obstacles (Path &path) const;
  public:
    /// Estimated obstacle radius.  The obstacle may be larger than at the
    /// detected edge.
    static const int obstacle_radius_mm = 150;
    /// Stop distance. Distance under which an obstacle is considered harmful
    /// when moving.
    static const int stop_mm = 350;
    /// Clearance distance.  Distance over which an obstacle should be to the
    /// side when moving.
    ///
    /// OK, more explanations: when moving, a rectangle is placed in front of
    /// the robot, of length STOP_MM and width 2 * (CLEARANCE_MM +
    /// BOT_SIZE_SIDE).  If an obstacle is inside this rectangle, it is
    /// considered in the way.
    ///
    /// If the destination point is near (< STOP_MM), this reduce the
    /// rectangle length.
    ///
    /// If the destination is really near (< EPSILON_MM), ignore all this.
    static const int clearance_mm = 100;
    /// Destination distance near enough so that obstacles could be ignored.
    static const int epsilon_mm = 70;
  private:
    /// Validity period of a new obstacle.
    static const int valid_new_ = 125;
    /// Distance under which obstacles are considered the same.
    static const int same_mm_ = 150;
    /// Set when changed since last update.
    bool changed_;
    /// Change report rate limit.
#ifdef TARGET_host
    static const int report_rate_limit_ = 0;
#else
    static const int report_rate_limit_ = 125;
#endif
    /// Change report rate limit counter.
    int report_rate_limit_cpt_;
    /// Single obstacle.
    struct Obstacle
    {
        /// Position.
        vect_t pos;
        /// Validity period as a number of cycles, or 0 if invalid.
        int valid;
    };
    /// Number of obstacles.
    static const int obstacles_nb_ = 4;
    /// Obstacles array.
    Obstacle obstacles_[obstacles_nb_];
};

#endif // obstacles_hh
