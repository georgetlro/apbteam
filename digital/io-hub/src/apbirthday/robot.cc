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
#include "robot.hh"

#include "bot.hh"
#include "top.hh"

#include "ucoolib/arch/arch.hh"
#include "ucoolib/utils/bytes.hh"

Robot *robot;

Robot::Robot ()
    : main_i2c_queue_ (hardware.main_i2c), zb_i2c_queue_ (hardware.zb_i2c),
      asserv (main_i2c_queue_, BOT_SCALE),
      mimot (main_i2c_queue_),
#ifdef TARGET_host
      pot_regul (hardware.host),
#else
      pot_regul (main_i2c_queue_, 0x7),
#endif
      beacon (zb_i2c_queue_),
      dev_proto (*this, hardware.dev_uart),
      zb_proto (*this, hardware.zb_uart),
      usb_proto (*this, hardware.usb),
      chrono (90000 - 1000),
      pressure (hardware.adc_pressure, hardware.pneum_open, mimot.motor0),
#ifdef TARGET_stm32
      rgb (&hardware.cannon_rgb_2),
#endif
      jack (hardware.raw_jack, 50),
      demo (false),
      usdist_control_ (2),
      usdist0_ (usdist_control_, hardware.adc_dist0, hardware.dist0_sync, 100, 700, 650),
      usdist1_ (usdist_control_, hardware.adc_dist1, hardware.dist1_sync, 100, 700, 650),
      usdist2_ (usdist_control_, hardware.adc_dist2, hardware.dist2_sync, 100, 700, 650),
      usdist3_ (usdist_control_, hardware.adc_dist3, hardware.dist3_sync, 100, 700, 650),
      radar_ (usdist0_, usdist1_, usdist2_, usdist3_),
      candles (1),
      fsm_debug_state_ (FSM_DEBUG_RUN),
      outputs_set_ (outputs_, lengthof (outputs_)),
      stats_proto_ (0),
      stats_asserv_ (0), stats_beacon_ (0),
      stats_chrono_ (false), stats_chrono_last_s_ (-1),
      stats_inputs_ (0), stats_usdist_ (0), stats_cake_ (0), stats_pressure_ (0)
{
    robot = this;
    // Fill I/O arrays.
    unsigned i = 0;
    inputs_[i++] = &hardware.raw_jack;
    inputs_[i++] = &hardware.ihm_color;
    inputs_[i++] = &hardware.ihm_strat;
    inputs_[i++] = &hardware.ihm_robot_nb;
    inputs_[i++] = &hardware.ihm_lol;
    inputs_[i++] = &hardware.ihm_emerg_stop;
    inputs_[i++] = &hardware.glass_contact;
    inputs_[i++] = &hardware.cherry_plate_left_contact;
    inputs_[i++] = &hardware.cherry_plate_right_contact;
    inputs_[i++] = &hardware.cake_arm_out_contact;
    inputs_[i++] = &hardware.cake_arm_in_contact;
    ucoo::assert (i == lengthof (inputs_));
    i = 0;
    outputs_[i++] = &hardware.cherry_bad_out;
    outputs_[i++] = &hardware.cherry_bad_in;
    outputs_[i++] = &hardware.cherry_plate_up;
    outputs_[i++] = &hardware.cherry_plate_down;
    outputs_[i++] = &hardware.cherry_plate_clamp;
    outputs_[i++] = &hardware.cake_arm_out;
    outputs_[i++] = &hardware.cake_arm_in;
    outputs_[i++] = &hardware.cake_push_far_out;
    outputs_[i++] = &hardware.cake_push_far_in;
    outputs_[i++] = &hardware.cake_push_near_out;
    outputs_[i++] = &hardware.cake_push_near_in;
    outputs_[i++] = &hardware.glass_lower_clamp_close;
    outputs_[i++] = &hardware.glass_lower_clamp_open;
    outputs_[i++] = &hardware.glass_upper_clamp_close;
    outputs_[i++] = &hardware.glass_upper_clamp_open;
    outputs_[i++] = &hardware.glass_upper_clamp_up;
    outputs_[i++] = &hardware.glass_upper_clamp_down;
    outputs_[i++] = &hardware.gift_out;
    outputs_[i++] = &hardware.gift_in;
    outputs_[i++] = &hardware.ballon_funny_action;
    outputs_[i++] = &hardware.pneum_open;
    ucoo::assert (i == lengthof (outputs_));
    for (i = 0; i < lengthof (outputs_); i++)
        outputs_[i]->output ();
    // Set US dist sensors start state.
    hardware.dist0_sync.output ();
    hardware.dist1_sync.output ();
    hardware.dist2_sync.output ();
    hardware.dist3_sync.output ();
    usdist0_.disable ();
    usdist1_.disable ();
    usdist2_.disable ();
    usdist3_.disable ();
}

void
Robot::main_loop ()
{
    while (1)
    {
        // Wait until next cycle.
        hardware.wait ();
        // Update IO modules.
        obstacles.update ();
        pressure.update ();
        jack.update ();
        outputs_set_.update ();
        top_update ();
        // Handle communications.
        bool sync = main_i2c_queue_.sync ();
        zb_i2c_queue_.sync ();
        Position robot_pos = asserv.get_position ();
        beacon.send_position (robot_pos.v);
        // Look for obstacles.
        if (usdist_control_.update ())
        {
            radar_.update (robot_pos, obstacles);
        }
        // Handle events if synchronised.
        if (sync && fsm_debug_state_ != FSM_DEBUG_STOP)
        {
            if (fsm_gen_event ()
                && fsm_debug_state_ == FSM_DEBUG_STEP)
                fsm_debug_state_ = FSM_DEBUG_STOP;
        }
        // Handle commands.
        dev_proto.accept ();
        zb_proto.accept ();
        usb_proto.accept ();
        // Handle zb programmation.
        hardware.zb_handle ();
        // Send stats.
        proto_stats ();
    }
}

bool
Robot::fsm_gen_event ()
{
    // If an event is handled, stop generating any other event, because a
    // transition may have invalidated the current robot state.
#define fsm_handle_and_return(event) \
    do { if (ANGFSM_HANDLE (AI, event)) return true; } while (0)
    // FSM timeouts.
    if (FSM_HANDLE_TIMEOUT (AI))
        return true;
    // Motor status.
    Motor::Status robot_move_status;
    robot_move_status = asserv.get_status ();
    if (robot_move_status == Motor::SUCCESS)
        fsm_handle_and_return (robot_move_success);
    else if (robot_move_status == Motor::FAILURE)
        fsm_handle_and_return (robot_move_failure);
    // Jack.
    if (!jack.get ())
        fsm_handle_and_return (jack_inserted);
    else
        fsm_handle_and_return (jack_removed);
    // Demo mode.
    if (demo && demo_fsm_gen_event ())
        return true;
    // FSM queue.
    if (fsm_queue.poll ())
    {
        Event event = fsm_queue.pop ();
        if (ANGFSM_HANDLE_VAR (AI, event))
            return true;
    }
    // Top FSM events.
    if (top_fsm_gen_event ())
        return true;
    // Check obstacles.
    if (move.check_obstacles ())
        return true;
    return false;
}

bool
Robot::demo_fsm_gen_event ()
{
    if (robot->hardware.ihm_strat.get ())
        fsm_handle_and_return (top_demo_candles);
    return false;
}

void
Robot::proto_handle (ucoo::Proto &proto, char cmd, const uint8_t *args, int size)
{
#define c(cmd, size) ((cmd) << 8 | (size))
    switch (c (cmd, size))
    {
    case c ('z', 0):
        // Reset.
        ucoo::arch_reset ();
        break;
    case c ('Z', 0):
        // Enter zigbit update from uart.
        proto.send ('Z');
        hardware.zb_handle (hardware.dev_uart);
        return;
    case c ('f', 0):
        // Enter FSM debug mode, then step once.
        fsm_debug_state_ = FSM_DEBUG_STEP;
        break;
    case c ('d', 2):
        // Demo mode.
        // 1B: 'f for follow mode.
        // 1B: 00 for clockwise, 01 for anti-clockwise
        top_demo_follow (args[0] == 1 ? 1 : -1);
        break;
    case c ('m', 5):
        // Go to position.
        // 2H: x, y.
        // 1B: direction_consign.
        {
            vect_t pos = {
                (int16_t) ucoo::bytes_pack (args[0], args[1]),
                (int16_t) ucoo::bytes_pack (args[2], args[3]),
            };
            move.stop ();
            move.start (pos, Asserv::DirectionConsign (args[4]));
        }
        break;
      case c ('f', 2):
        // Set low pressure threshold.
        // 1w: sensor value, 4096 is full scale.
        pressure.set (ucoo::bytes_pack (args[0], args[1]));
        break;
      case c ('o', 5):
        // Set/clear/toggle outputs.
        // - 1d: mask.
        // - 1b: 00 to clear, 01 to set, 02 to toggle.
        {
            uint32_t mask =
                ucoo::bytes_pack (args[0], args[1], args[2], args[3]);
            if (args[4] < 3)
                outputs_set_.command (Outputs::Command (args[4]), mask);
            else
            {
                proto.send ('?');
                return;
            }
        }
        break;
      case c ('o', 6):
        // Toggle outputs for a short time.
        // - 1d: mask.
        // - 1w: duration.
        {
            uint32_t mask =
                ucoo::bytes_pack (args[0], args[1], args[2], args[3]);
            outputs_set_.command (Outputs::TOGGLE, mask);
            outputs_set_.command_later (Outputs::TOGGLE, mask,
                    ucoo::bytes_pack (args[4], args[5]));
        }
        break;
    case c ('p', 4):
        // Set potentiometer wiper.
        // - 1b: wiper index.
        // - 1h: wiper value.
        // - 1b: 01 for eeprom, 00 for volatile.
        pot_regul.set_wiper (args[0], ucoo::bytes_pack (args[1], args[2]),
                             args[3] ? true : false);
        break;
    case c ('A', 1):
        // Asserv position stats.
        // 1B: stat interval.
        stats_asserv_cpt_ = stats_asserv_ = args[0];
        stats_proto_ = &proto;
        break;
    case c ('B', 1):
        // Beacon stats.
        // 1B: stat interval.
        stats_beacon_cpt_ = stats_beacon_ = args[0];
        // TODO: remove this test hack:
        beacon.on (args[0] & 1);
        stats_proto_ = &proto;
        break;
    case c ('C', 1):
        // Chrono stats.
        // 1B: start chrono if non-zero.
        stats_chrono_ = true;
        if (args[0])
            chrono.start ();
        stats_proto_ = &proto;
        break;
    case c ('I', 1):
        // Input stats.
        // 1B: stat interval.
        stats_inputs_cpt_ = stats_inputs_ = args[0];
        stats_proto_ = &proto;
        break;
    case c ('U', 1):
        // US distance sensors stats.
        // 1B: stat interval.
        stats_usdist_cpt_ = stats_usdist_ = args[0];
        stats_proto_ = &proto;
        break;
    case c ('G', 1):
        // Cake distance sensors stats.
        // 1B: stat interval.
        stats_cake_cpt_ = stats_cake_ = args[0];
        stats_proto_ = &proto;
        break;
    case c ('F', 1):
        // Pressure stats.
        // 1B: stat interval.
        stats_pressure_cpt_ = stats_pressure_ = args[0];
        stats_proto_ = &proto;
        break;
#ifdef TARGET_stm32
    case c ('R', 0):
        // Activate RGB
        rgb.start();
        proto.send ('Z', "X", rgb.get_get());
        while (!rgb.poke());
        proto.send ('R', "X", rgb.get ());
        break;
#endif
    case c ('b', 2):
        // Candles arm manipulation.
        //   - 00: arm events
        //      - 00: deploy arm
        //      - 01: undeploy arm
        //   - 01: near
        //      - 00: push
        //      - 01: unpush
        //      - 02: event push
        //   - 02: far
        //      - 00: push
        //      - 01: unpush
        //      - 02: event push
        //   - 03: deploying
        //      - 00: deploy
        //      - 01: undeploy 1
        //      - 02: undeploy 2
        //   - 04: flamby
        //      - 00: arm
        //      - 01: far puncher
        //   - 05: arm
        //      - 00: out
        //      - 01: in
        //   - 06: crampe
        //      - 00: arm
        //      - 01: far
        //   - 07: test undeploy
        //      - bb: sleep 1
        //      - bb: sleep 2
        //   - 08: test sleep
        //      - 01: near
        //          - bb: sleep
        //      - 02: far
        //          - bb: sleep
        if (args[0] == 0 && args[1] == 0)
            FSM_HANDLE (AI, ai_candle_deploy);
        else if (args[0] == 0 && args[1] == 1)
            FSM_HANDLE (AI, ai_candle_undeploy);
        else if (args[0] == 1 && args[1] == 0)
            Candles::push_near ();
        else if (args[0] == 1 && args[1] == 1)
            Candles::unpush_near ();
        else if (args[0] == 1 && args[1] == 2)
            FSM_HANDLE (AI, ai_candle_near_punch);
        else if (args[0] == 2 && args[1] == 0)
            Candles::push_far ();
        else if (args[0] == 2 && args[1] == 1)
            Candles::unpush_far ();
        else if (args[0] == 2 && args[1] == 2)
            FSM_HANDLE (AI, ai_candle_far_punch);
        else if (args[0] == 3 && args[1] == 0)
            Candles::deploy_arm ();
        else if (args[0] == 3 && args[1] == 1)
            Candles::undeploy_arm_1 ();
        else if (args[0] == 3 && args[1] == 2)
            Candles::undeploy_arm_2 ();
        else if (args[0] == 4 && args[1] == 0)
            Candles::flamby_arm ();
        else if (args[0] == 4 && args[1] == 1)
            Candles::flamby_far ();
        else if (args[0] == 5 && args[1] == 0)
            Candles::arm_out ();
        else if (args[0] == 5 && args[1] == 1)
            Candles::arm_back ();
        else if (args[0] == 6 && args[1] == 0)
            Candles::crampe_arm ();
        else if (args[0] == 6 && args[1] == 1)
            Candles::crampe_far ();

         break;
    case c ('b', 3):
        if (args[0] == 7)
        {
            int a = 0;
            Candles::arm_back ();
            while (a++ != args[1])
                robot->hardware.wait ();
            Candles::crampe_far ();
            a = 0;
            while (a++ != args[2])
                robot->hardware.wait ();
            Candles::flamby_far ();
        }
        if (args[0] == 8)
        {
            int a = 0;
            if (args[1] == 1)
            {
                Candles::push_near ();
                while (a++ != args[2])
                    robot->hardware.wait ();
                Candles::unpush_near ();
            }
            if (args[1] == 2)
            {
                Candles::push_far ();
                while (a++ != args[2])
                    robot->hardware.wait ();
                Candles::unpush_far ();
            }
        }
        break;
    default:
        proto.send ('?');
        return;
    }
    // Acknowledge.
    proto.send_buf (cmd, args, size);
}

void
Robot::proto_stats ()
{
    if (!stats_proto_)
        return;
    if (stats_asserv_ && !--stats_asserv_cpt_)
    {
        Position pos = asserv.get_position ();
        stats_proto_->send ('A', "hhH", pos.v.x, pos.v.y, pos.a);
        stats_asserv_cpt_ = stats_asserv_;
    }
    if (stats_beacon_ && !--stats_beacon_cpt_)
    {
        for (int i = 0; i < beacon.pos_nb; i++)
        {
            vect_t pos;
            int trust;
            trust = beacon.get_position (i, pos);
            stats_proto_->send ('B', "BhhB", i, pos.x, pos.y, trust);
        }
        stats_beacon_cpt_ = stats_beacon_;
    }
    if (stats_chrono_)
    {
        int s = chrono.remaining_time_ms () / 1000;
        if (s != stats_chrono_last_s_)
        {
            stats_proto_->send ('C', "b", s);
            stats_chrono_last_s_ = s;
        }
    }
    if (stats_inputs_ && !--stats_inputs_cpt_)
    {
        uint32_t inputs = 0;
        for (unsigned int i = 0; i < lengthof (inputs_); i++)
        {
            if (inputs_[i]->get ())
                inputs |= 1 << i;
        }
        stats_proto_->send ('I', "L", inputs);
        stats_inputs_cpt_ = stats_inputs_;
    }
    if (stats_usdist_ && !--stats_usdist_cpt_)
    {
        stats_proto_->send ('U', "HHHH", usdist0_.get (), usdist1_.get (),
                            usdist2_.get (), usdist3_.get ());
        stats_usdist_cpt_ = stats_usdist_;
    }
    if (stats_cake_ && !--stats_cake_cpt_)
    {
        stats_proto_->send ('G', "HH", hardware.adc_cake_front.read (),
                            hardware.adc_cake_back.read ());
        stats_cake_cpt_ = stats_cake_;
    }
    if (stats_pressure_ && !--stats_pressure_cpt_)
    {
        stats_proto_->send ('F', "H", pressure.get ());
        stats_pressure_cpt_ = stats_pressure_;
    }
}

