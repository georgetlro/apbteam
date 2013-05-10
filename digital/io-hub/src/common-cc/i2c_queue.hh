#ifndef i2c_queue_hh
#define i2c_queue_hh
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
#include "ucoolib/common.hh"
#include "ucoolib/intf/i2c.hh"

/// Handle communication as a master to several slaves.
///
/// The communication protocol is always based on:
///  - a message sent to the slave with a command to execute,
///  - a status read from the slave containing the current slave state.
///
/// The first byte of all messages is a CRC of the following bytes.
///
/// The next byte is the command sequence number.  It is used by the master to
/// know if its command has been handled.  The last handled command sequence
/// number is available in the slave status.
///
/// All other bytes are the message payload.
///
/// As long as the slave last command sequence number is not equal to the last
/// sent sequence number, the master can not send any other command.  If the
/// slave do not acknowledge the command after a time out, the command is sent
/// again.
///
/// Several commands can be stored by this class, it will defer their
/// transmission until the first command is acknowledged.
///
/// This class also support unreliable transient command delivery.  There can
/// be only one transient command per slave and it is sent only if the slave
/// is synchronised.
/// 
/// There can be slaves which do not use this protocol.  In this case, the
/// message is sent without any CRC or acknowledgement support.
class I2cQueue : public ucoo::I2cMaster::FinishedHandler
{
  public:
    /// Maximum status size.
    static const int status_size_max = 14;
    /// Maximum command size.
    static const int command_size_max = 18;
    /// Message header size.
    static const int header_size = 2;
    /// Maximum messages in queue.
    static const int queue_size = 15;
    /// Number of update between retries.
    static const int retry_timeout = 3;
  public:
    class Slave;
    /// Command type.
    enum CommandType { RELIABLE, TRANSIENT, RAW };
    /// Command buffer.
    struct Command
    {
        /// Associated slave.
        Slave *slave;
        /// Command raw size, or 0 for invalid.
        int raw_size;
        /// Command type.
        CommandType type;
        /// Command header + payload.
        uint8_t raw[header_size + command_size_max];
    };
    /// Slave classes should inherit from this.
    class Slave
    {
      protected:
        /// Initialise I2cQueue parameters, use 0 status_size if no status.
        Slave (I2cQueue &queue, uint8_t address, int status_size);
        /// Send a command.
        void send (const uint8_t *command, int size,
                   CommandType type = RELIABLE)
        { queue_.send (*this, command, size, type); }
        /// Called when a status has been received and synchronisation is
        /// done.
        virtual void recv_status (const uint8_t *status) = 0;
      private:
        /// Increment and return seq.
        inline uint8_t seq_next ();
      private:
        friend class I2cQueue;
        /// Next slave in list.
        Slave *next_;
        /// Attached I2cQueue.
        I2cQueue &queue_;
        /// Size of status.
        int raw_status_size_;
        /// Slave address.
        uint8_t address_;
        /// Last command sequence number.
        uint8_t seq_;
        /// Is this slave present?
        bool present_;
        /// Whether last received status was valid.
        bool last_status_valid_;
        /// Last received status.
        uint8_t last_raw_status_[header_size + status_size_max];
        /// Transient command slots.
        Command transient_commands_[2];
        /// Current active transient command.
        unsigned int transient_commands_index_;
    };
  public:
    /// Constructor.
    I2cQueue (ucoo::I2cMaster &i2c, bool can_drop = false);
    /// Synchronise all slaves, return true if synchronised (no message in
    /// queue).
    bool sync ();
    /// Add a new slave in the list of slaves.
    void register_slave (Slave &slave);
    /// Send a command.
    void send (Slave &slave, const uint8_t *command, int size,
               CommandType type = RELIABLE);
    /// See I2cMaster::FinishedHandler::finished.
    void finished (int status);
  private:
    void start_update_status ();
    void end_update_status (int status);
    void start_send_queue ();
    void end_send_queue (int status);
    void start_send_transient ();
    void end_send_transient (int status);
    /// Return next index in queue.
    static int queue_next (int index)
    {
        return (static_cast<unsigned int> (index) + 1) % (queue_size + 1);
    }
  private:
    /// I2C interface.
    ucoo::I2cMaster &i2c_;
    /// Can drop reliable command in case of big problem.
    bool can_drop_;
    /// Chained list of slaves.
    Slave *slaves_;
    /// State of update FSM.
    enum UpdateState
    {
        /// Not started.
        IDLE,
        /// Receiving a status.
        RECV_STATUS,
        /// Sending top of command queue.
        SEND_QUEUE,
        /// Sending transient command.
        SEND_TRANSIENT,
    };
    UpdateState update_state_;
    /// Currently addressed slave.
    Slave *update_slave_;
    /// Currently sent command.
    Command *update_command_;
    /// Command queue circular buffer.
    Command queue_[queue_size + 1];
    /// Index to head command, next command to be sent.
    ucoo::int_atomic_t queue_head_;
    /// Index to tail of queue, free space for next command to enqueue.
    ucoo::int_atomic_t queue_tail_;
    /// Number of update before next retransmission from queue.
    int queue_timeout_;
};

#endif // i2c_queue_hh
