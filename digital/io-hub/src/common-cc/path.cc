// path.cc
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

#include "path.hh"
#include "bot.hh"
#include "robot.hh"
#ifdef HOST
#include "debug.host.hh"
#endif

extern "C" {
#include "modules/math/fixed/fixed.h"
#include "modules/math/geometry/distance.h"
#include "modules/utils/utils.h"
}

/** Define path finding plot ID for the python simu framework */
#define PATH_PLOT_ID    2

/** Angle between obstacles points. */
#define PATH_ANGLE_824(pOINTS_NB) \
    ((1L << 24) / (pOINTS_NB))

/** Check for vectors equality */
#define PATH_VECT_EQUAL(v1, v2) \
    ((v1)->x==(v2)->x && (v1)->y==(v2)->y)

/** Check if point is inside a circle */
#define PATH_IN_CIRCLE(pOINT, cENTER, rADIUS) \
    (distance_point_point((pOINT), (cENTER)) <= (rADIUS))

static int32_t pos_dot_product(vect_t* pa, vect_t* pb, vect_t* pc, vect_t* pd)
{
    vect_t vab = *pb; vect_sub(&vab, pa);
    vect_t vcd = *pd; vect_sub(&vcd, pc);
    return vect_dot_product(&vab, &vcd);
}

Path::Path( const uint16_t border_xmin,
            const uint16_t border_ymin,
            const uint16_t border_xmax,
            const uint16_t border_ymax) :
    border_xmin(border_xmin),
    border_ymin(border_ymin),
    border_xmax(border_xmax),
    border_ymax(border_ymax),
    obstacles(NULL),
    navpoints(NULL),
    navweights(NULL),
    astar_nodes(NULL),
    escape_factor(0),
    obstacles_nb(0),
    navpoints_nb(0),
    next_node(0)
{
    path_debug("Path constructor\n");
}

void Path::reset()
{
    vect_t nul = {0,0};
    path_debug("Path reset\n");
    obstacles_nb = 0;
    navpoints_nb = CC_RESERVED_NAVPOINTS_NB;
    navpoints[CC_NAVPOINT_SRC_IDX] = nul;
    navpoints[CC_NAVPOINT_DST_IDX] = nul;
    next_node = 0;
    escape_factor = 0;
}

void Path::add_obstacle( const vect_t &c,
                         uint16_t r,
                         int nodes,
                         const int nlayers,
                         const uint16_t nav_clearance,
                         const bool target)
{
    uint32_t rot_a, rot_b, nr;
    int32_t x, y, nx;
    int npt, layer;

    path_debug("Add obstacle c=(%u;%u) r=%u nodes=%u layers=%u\n",c.x, c.y, r, nodes, nlayers);

    /* Enlarge the obstacle radius by the robot size */
    r += BOT_SIZE_RADIUS;

    /* Store obstacle */
    obstacles[obstacles_nb].c = c;
    obstacles[obstacles_nb].r = r;
    obstacles[obstacles_nb].target_allowed = target;
    obstacles_nb++;

    /* Enlarge the navigation points circle radius by */
    /* the additionnal clearance area width */
    r += nav_clearance;

    /* Extend the points radius to allow the robot to go */
    /* from one to another without collision with the */
    /* obstacle circle. New radius is r / cos(angle/2) */
    nr = PATH_ANGLE_824(nodes*2);
    y = fixed_div_f824(r, fixed_cos_f824(nr)) + 3 /* margin for the unprecise fixed point computation */;
    x = 0;

    /* Add a number of sets of navigation points with different weights */
    for(layer=nlayers-1; layer>=0; layer--)
    {
        /* Complex number A = cos(angle) + i sin(angle) */
        nr = PATH_ANGLE_824(nodes);
        rot_a = fixed_cos_f824(nr);
        rot_b = fixed_sin_f824(nr);

        /* Compute obstacle points positions around a circle */
        for (npt=0; npt<nodes; npt++)
        {
            /* Compute the point absolute position */
            navpoints[navpoints_nb].x = c.x + (vect_value_t)x;
            navpoints[navpoints_nb].y = c.y + (vect_value_t)y;

            /* Check it is in playground */
            if (navpoints[navpoints_nb].x >= border_xmin
             && navpoints[navpoints_nb].y >= border_ymin
             && navpoints[navpoints_nb].x <= border_xmax
             && navpoints[navpoints_nb].y <= border_ymax)
            {
                /* Accept point */
                navweights[navpoints_nb] = (layer * CC_NAVWEIGHT_STEP);
                path_debug("Add point %u (%u;%u) w=%u\n",
                        navpoints_nb, navpoints[navpoints_nb].x, navpoints[navpoints_nb].y, navweights[navpoints_nb]);
                navpoints_nb++;
            }

            /* Complex multiply with A = cos(angle) + i sin(angle) */
            nx = fixed_mul_f824(x, rot_a) - fixed_mul_f824(y, rot_b);
            y =  fixed_mul_f824(y, rot_a) + fixed_mul_f824(x, rot_b);
            x = nx;
        }

        /* Prepare the next layer */
        /* Twice less navpoints */
        nodes >>= 1;

        /* Enlarge the circle */
        y += BOT_SIZE_RADIUS; x = 0;
    }

#ifdef TARGET_host
    /* Plot obstacle points */
    robot->hardware.simu_report.pos( &navpoints[CC_RESERVED_NAVPOINTS_NB], navpoints_nb-CC_RESERVED_NAVPOINTS_NB, PATH_PLOT_ID);
#endif
}

int Path::find_neighbors(int cur_point, struct astar_neighbor_t *neighbors)
{
    int neighbors_nb = 0;
    ucoo::assert(cur_point<navpoints_nb && neighbors!=NULL);

    /* Parse all navigation points */
    for(int next_point=0; next_point<navpoints_nb; next_point++)
    {
        /* Except the current one */
        if (next_point!=cur_point)
        {
            /* Compute the segment weight */
            /* 1st: compute the distance to go */
            weight_t weight =  (weight_t)distance_point_point(&navpoints[cur_point], &navpoints[next_point]);
            /* 2nd: Add the target navpoint extra weigth */
            weight += (weight * navweights[cur_point]) >> CC_NAVWEIGHT_PRECISION;
            path_debug("- Node %u (%u;%u) w=%u (%u) ", next_point, navpoints[next_point].x, navpoints[next_point].y, weight, navweights[next_point]);

            /* Check every obstacle */
            for(int j=0; j<obstacles_nb; j++)
            {
                /* Check for intersection with obstacle */
                uint16_t d = distance_segment_point(&navpoints[cur_point], &navpoints[next_point], &obstacles[j].c);
                if (d < obstacles[j].r)
                {
                    /* Collision while planing the last move to the */
                    /* the center of an obstacle. This is useful to */
                    /* target the center of an obstacle and stop */
                    /* in front of it (ex, the cake in apbirthday 2013) */
                    if (obstacles[j].target_allowed &&
                        cur_point==CC_NAVPOINT_DST_IDX &&
                        PATH_VECT_EQUAL(&navpoints[cur_point], &obstacles[j].c))
                    {
                        /* Skip this obstacle */
                        path_debug("in collision with c=(%u;%u) r=%u allowed ",
                            obstacles[j].c.x, obstacles[j].c.y, obstacles[j].r);
                        continue;
                    }
                    /* Collision while trying to escape the source point */
                    /* if and only if the source point is in this obstacle */
                    /* and we are going away from the obstacle center */
                    else if (escape_factor!=0 && next_point==CC_NAVPOINT_SRC_IDX &&
                             PATH_IN_CIRCLE(&navpoints[next_point], &obstacles[j].c, obstacles[j].r) &&
                             pos_dot_product(&navpoints[next_point], &obstacles[j].c, &navpoints[next_point], &navpoints[cur_point])<=0)
                    {
                        /* Allow this navigation point with an extra cost */
                        weight *= escape_factor;
                    }
                    /* Other collisions are not allowed */
                    else
                    {
                        /* Disable this navigation point */
                        weight = 0;
                    }
                    path_debug("in collision with c=(%u;%u) r=%u w=%u ",
                        obstacles[j].c.x, obstacles[j].c.y, obstacles[j].r, weight);
                    break; /* Stop checking for obstacles with this node */
                }
            }

            /* Add this navigation point in the neighbor list when valid */
            if (weight)
            {
                path_debug("=> validated w=%u\n", weight);
                neighbors[neighbors_nb].node = next_point;
                neighbors[neighbors_nb].weight = weight;
                neighbors_nb++;
            }
            else
            {
                path_debug("=> skipped\n");
            }
        }
    }

#if PATH_DEBUG
    path_debug("\tFound %u neighbors: ", neighbors_nb);
    for(int i=0;i<neighbors_nb;i++)
        path_debug("%u (%u)  ", neighbors[i].node, neighbors[i].weight);
    path_debug("\n");
#endif

    return neighbors_nb;
}

void Path::compute(weight_t escape)
{
    path_debug("** Path compute(start) escape=%u\n", escape);

    /* Store the escape factor */
    escape_factor = escape;

    /* Call the A* algorithm */
    path_found = (bool)astar(astar_nodes, navpoints_nb, CC_NAVPOINT_DST_IDX, CC_NAVPOINT_SRC_IDX);
    if (path_found)
    {
        /* Store next node to go to */
        next_node = astar_nodes[CC_NAVPOINT_SRC_IDX].prev;

#ifdef TARGET_host
        /* Log and display the path found */
        vect_t path[navpoints_nb];
        int node = CC_NAVPOINT_SRC_IDX;
        int path_nb = 0;

        path_debug(">> Path found: ");
        while(node!=CC_NAVPOINT_DST_IDX)
        {
            path_debug("%u (%u), ", node, navweights[node]);
            path[path_nb++] = navpoints[node];
            node = astar_nodes[node].prev;
        }
        path[path_nb++] = navpoints[node];
        robot->hardware.simu_report.path(path, path_nb);
        path_debug("%u\n", node);
#endif
    }

    path_debug("** Path compute(end) found=%u escape=%u\n", path_found, escape);
}

void Path::obstacle(const int index, const vect_t &c, const uint16_t r, const int f, const bool target)
{
    add_obstacle( c,
                  (r + Obstacles::clearance_mm + CC_OBSTACLES_CLEARANCE),
                  CC_OBSTACLE_NAVPOINTS_NB,
                  CC_OBSTACLE_NAVPOINTS_LAYERS,
                  CC_NAVPOINTS_CLEARANCE,
                  target);
}

void Path::endpoints(const vect_t &src, const vect_t &dst)
{
    /* Store endpoints location */
    path_debug("Set path endpoints src=(%u;%u) dst=(%u;%u)\n",
            src.x, src.y, dst.x, dst.y);
    navpoints[CC_NAVPOINT_SRC_IDX] = src;
    navpoints[CC_NAVPOINT_DST_IDX] = dst;

    /* Init endpoints weights */
    navweights[CC_NAVPOINT_SRC_IDX] = 0;
    navweights[CC_NAVPOINT_DST_IDX] = 0;
}

bool Path::get_next(vect_t &p)
{
    if (path_found)
    {
        ucoo::assert(next_node<navpoints_nb);
        p = navpoints[next_node];
        next_node = astar_nodes[next_node].prev;
    }
    return path_found;
}

vect_t& Path::get_point_vect(const int index)
{
    ucoo::assert(index<navpoints_nb);
    return navpoints[index];
}

int Path::get_point_index(const vect_t& point)
{
    for(int i=0; i<navpoints_nb; i++)
    {
        if (PATH_VECT_EQUAL(&navpoints[i], &point))
            return i;
    }
    return -1;
}

void Path::prepare_score(const vect_t &src, weight_t escape)
{
    path_debug("Path prepare score from src=(%u;%u) escape=%u\n", src.x, src.y, escape);
    escape_factor = escape;
    astar_dijkstra_prepare(astar_nodes, navpoints_nb, get_point_index(src), CC_NAVPOINT_DST_IDX);
}

weight_t Path::get_score(const vect_t &dst)
{
    uint16_t score = astar_dijkstra_finish(astar_nodes, navpoints_nb, get_point_index(dst));
    path_debug("Path get score=%u for dst=(%u;%u)\n", score, dst.x, dst.y);
    return score;
}

extern "C" uint8_t
AC_ASTAR_NEIGHBOR_CALLBACK (uint8_t node, struct astar_neighbor_t *neighbors)
{
#if PATH_DEBUG
    vect_t point_v = robot->path.get_point_vect(node);
    path_debug("AC_ASTAR_NEIGHBOR_CALLBACK node=%u (%u;%u)\n", node, point_v.x, point_v.y);
#endif
    return robot->path.find_neighbors(node, neighbors);
}

extern "C" uint16_t
AC_ASTAR_HEURISTIC_CALLBACK (uint8_t node)
{
    vect_t point_a = robot->path.get_point_vect(node);
    vect_t point_b = robot->path.get_point_vect(CC_NAVPOINT_SRC_IDX);
    int16_t dist = distance_point_point(&point_a, &point_b);
    path_debug("Heuristic node=%u dist=%u\n", node, dist);
    return dist;
}
