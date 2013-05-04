#ifndef path_hh
#define path_hh
// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Nicolas Schodet
// Copyright (C) 2013 Olivier Lanneluc
//
// APBTeam:
//      Web: http://apbteam.org/
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
#include "playground.hh"
extern "C" {
#include "modules/path/astar/astar.h"
}

/** Enable path finding debug code */
#define PATH_DEBUG 0
#if PATH_DEBUG
# define path_debug host_debug
#else
# define path_debug(args...) do { } while (0)
#endif

/** Static nodes index for the endpoints */
enum {
    CC_NAVPOINT_SRC_IDX = 0,
    CC_NAVPOINT_DST_IDX,
    CC_RESERVED_NAVPOINTS_NB
};

/** Obstacle */
typedef struct path_obstacle_t
{
    /** Center. */
    vect_t c;
    /** Radius. */
    uint16_t r;
    /** Allow the robot to target the obstacle
     * in order to go in front of it, while stopping
     * at a given distance, before the collision happens */
    bool target_allowed;
} path_obstacle_t;

typedef uint16_t weight_t;

/** Path finding class */
class Path
{
  public:
    /** Initialise path */
    Path(const uint16_t, const uint16_t, const uint16_t, const uint16_t);
    /** Reset path computation, remove mobile obstacles, add static obstacles */
    virtual void reset(void);
    /** Set a moving obstacle position, radius and factor*/
    void obstacle(const int index, const vect_t &c, const uint16_t r, const int f = 0, const bool target = false);
    /** Set path source and destination */
    void endpoints(const vect_t &src, const vect_t &dst);
    /** Compute path with the given escape factor */
    void compute(weight_t escape = 0);
    /** Return a point vector by index */
    vect_t& get_point_vect(const int index);
    /** Return a point index */
    int get_point_index(const vect_t& point);
    /** Get next point in computed path, return false if none
       (no path found or last point given yet) */
    bool get_next(vect_t &p);
    /** Find all neighbors of a given node, fill the astar_neighbor structure */
    int find_neighbors(int node, struct astar_neighbor_t *neighbors);
    /** Prepare score computation for the given source, with given escape factor */
    void prepare_score(const vect_t &src, weight_t escape = 0);
    /** Return score for a given destination, using a previous preparation
       (also reuse previously given escape factor) */
    weight_t get_score(const vect_t &dst);

  protected:
    /** Add an obstacle on the field */
    void add_obstacle(const vect_t &c, uint16_t r, int nodes, const int nlayers, const uint16_t clearance, const bool target);
    /** Number of possible mobile obstacles. */
    static const int CC_OBSTACLES_NB = 4;
    /** Number of points per standard obstacle. */
    static const int CC_OBSTACLE_NAVPOINTS_NB = 10;
    /** Number of navigation points layers for each obstacle. */
    static const int CC_OBSTACLE_NAVPOINTS_LAYERS = 2;
    /** Number of navigation points. */
    static const int CC_NAVPOINTS_NB =
        (CC_RESERVED_NAVPOINTS_NB + CC_OBSTACLE_NAVPOINTS_LAYERS * CC_OBSTACLES_NB * CC_OBSTACLE_NAVPOINTS_NB);
    /** Borders, any point outside borders is eliminated. */
    const uint16_t border_xmin, border_ymin, border_xmax, border_ymax;
    /** List of obstacles. */
    path_obstacle_t* obstacles;
    /** List of navigation points coordonates */
    vect_t* navpoints;
    /** List of navigation points weights */
    weight_t* navweights;
    /** List of nodes used for A*. */
    struct astar_node_t* astar_nodes;

  private:
    /** Navigation points weight precision (2^-n).
     * Pay attention to overflow on weight_t variables */
    static const int CC_NAVWEIGHT_PRECISION = 4;
    /** Navigation points weight step * (2^-n). */
    static const int CC_NAVWEIGHT_STEP = 8;
    /** Extra clearance area added to the radius of the mobile obstacles
     * to counter the imprecision of the sonic sensors when the robot brakes */
    static const uint16_t CC_OBSTACLES_CLEARANCE = 60;
    /** Extra clearance area added to the radius of the navigation points
     * circle to move the navpoints away from the obstacle circle */
    static const uint16_t CC_NAVPOINTS_CLEARANCE = 40;
    /** Escape factor, 0 if none. */
    weight_t escape_factor;
    /** Number of obstacles */
    int obstacles_nb;
    /** Number of navigation points */
    int navpoints_nb;
    /** Which node to look at for next step. */
    uint8_t next_node;
    /** TRUE when a path has been found */
    bool path_found;
};

#endif // path_hh
