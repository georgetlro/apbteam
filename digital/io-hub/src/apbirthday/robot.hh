#ifndef robot_hh
#define robot_hh
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
#include "hardware.hh"
#include "asserv.hh"
#include "beacon.hh"
#ifdef TARGET_host
# include "potentiometer.host.hh"
#else
# include "potentiometer.stm32.hh"
#endif
#include "fsm_queue.hh"
#include "chrono.hh"
#ifdef TARGET_stm32
# include "rgb.hh"
#endif
#include "pressure.hh"
#include "debounce.hh"
#include "outputs.hh"
#include "radar_2013.hh"
#include "obstacles.hh"
#include "move.hh"
#include "path.hh"
#include "strat.hh"
#include "candles.hh"
#include "drinks.hh"
#include "plate.hh"
#include "cannon.hh"

#include "ucoolib/base/proto/proto.hh"
#include "ucoolib/dev/usdist/usdist.hh"

/// Main robot superclass.
class Robot : public ucoo::Proto::Handler
{
  public:
    /// Initialise robot singleton.
    Robot ();
    /// Main program loop.
    void main_loop ();
    /// Generate events for the FSM.
    bool fsm_gen_event ();
    /// Generate events for the FSM for demo mode.
    bool demo_fsm_gen_event ();
    /// Receive proto messages.
    void proto_handle (ucoo::Proto &proto, char cmd, const uint8_t *args, int size);
    /// Send stats.
    void proto_stats ();
  public:
    /// Public access to hardware class.
    Hardware hardware;
  private:
    /// I2C queues.
    I2cQueue main_i2c_queue_, zb_i2c_queue_;
  public:
    /// Public access to asserv class.
    Asserv asserv;
    /// Public access to mimot class.
    Mimot mimot;
    /// Public access to potentiometer class.
#ifdef TARGET_host
    PotentiometerHost pot_regul;
#else
    Potentiometer pot_regul;
#endif
    /// Public access to beacon class.
    Beacon beacon;
  private:
    /// Proto associated to each serial interface.
    ucoo::Proto dev_proto, zb_proto, usb_proto;
  public:
    /// Public access to deferred events FSM queue.
    FsmQueue fsm_queue;
    /// Public access to chrono.
    Chrono chrono;
    /// Public access to pressure handling.
    Pressure pressure;
#ifdef TARGET_stm32
    /// RGB Sensor
    RGB rgb;
#endif
    /// Jack debouncing.
    Debounce jack;
    /// Demo mode flag.
    bool demo;
  private:
    /// US distance sensors controller.
    ucoo::UsDistControl usdist_control_;
    /// US distance sensors.
    ucoo::UsDist usdist0_, usdist1_, usdist2_, usdist3_;
    /// Radar.
    Radar2013 radar_;
  public:
    /// Obstacle database.
    Obstacles obstacles;
    /// Path finding.
    Path path;
    /// Move FSM.
    Move move;
    /// Strategy.
    Strat strat;
    /// Candles.
    Candles candles;
    /// Drinks.
    Drinks drinks;
    /// Plate.
    Plate plate;
    /// Cannon
    Cannon cannon;
  private:
    /// FSM debug mode.
    enum FsmDebugState
    {
        /// Not debugging, running.
        FSM_DEBUG_RUN,
        /// Will stop after next transition.
        FSM_DEBUG_STEP,
        /// Stopped, waiting for orders.
        FSM_DEBUG_STOP,
    };
    FsmDebugState fsm_debug_state_;
    /// All inputs.
    ucoo::Io *inputs_[Hardware::inputs_nb];
    /// All outputs.
    ucoo::Io *outputs_[Hardware::outputs_nb];
    /// Handle set of outputs.
    Outputs outputs_set_;
    /// Proto used for stats.
    ucoo::Proto *stats_proto_;
    /// Asserv stats interval and counter.
    int stats_asserv_, stats_asserv_cpt_;
    /// Beacon stats interval and counter.
    int stats_beacon_, stats_beacon_cpt_;
    /// Enable chrono stats.
    bool stats_chrono_;
    /// Last stated second.
    int stats_chrono_last_s_;
    /// Input stats interval and counter.
    int stats_inputs_, stats_inputs_cpt_;
    /// US distance sensors stats interval and counter.
    int stats_usdist_, stats_usdist_cpt_;
    /// Cake distance sensors stats interval and counter.
    int stats_cake_, stats_cake_cpt_;
    /// Pressure stats interval and counter.
    int stats_pressure_, stats_pressure_cpt_;
};

/// Global instance pointer.
extern Robot *robot;

#endif // robot_hh
