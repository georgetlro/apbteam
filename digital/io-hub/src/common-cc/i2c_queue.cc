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
#include "i2c_queue.hh"

#include "ucoolib/utils/crc.hh"

#include <algorithm>

/// Update is done asynchronously.  At each cycle, the result of the previous
/// update is given to slave classes to update their status, then the next
/// update is started.  This frees CPU while I2C messages are exchanged.
///
/// There are three stages in the update:
///  - receive status from each slave,
///  - send one command from the queue if any,
///  - send one transient command for each synchronised slave if any.
///
/// A synchronised slave is a slave for which there is no command pending,
/// which means that the seq read from its status is the same as the last sent
/// command and there is no command for this slave in the queue.

I2cQueue::Slave::Slave (I2cQueue &queue, uint8_t address, int status_size)
    : next_ (0), queue_ (queue),
      raw_status_size_ (status_size ? header_size + status_size : 0),
      address_ (address), seq_ (0), present_ (status_size ? false : true),
      last_status_valid_ (false), transient_commands_index_ (0)
{
    queue.register_slave (*this);
    ucoo::assert (status_size <= status_size_max);
    transient_commands_[0].raw_size = 0;
    transient_commands_[1].raw_size = 0;
}

inline uint8_t
I2cQueue::Slave::seq_next ()
{
    if (seq_ == 255)
        seq_ = 1;
    else
        seq_++;
    return seq_;
}

I2cQueue::I2cQueue (ucoo::I2cMaster &i2c, bool can_drop)
    : i2c_ (i2c), can_drop_ (can_drop), slaves_ (0), update_state_ (IDLE),
      queue_head_ (0), queue_tail_ (0), queue_timeout_ (0)
{
    i2c_.register_finished (*this);
}

bool
I2cQueue::sync ()
{
    // Wait for end of all transfers.
    i2c_.wait ();
    // Update status with received data.
    bool all_slave_sync = true;
    for (Slave *s = slaves_; s; s = s->next_)
    {
        if (s->raw_status_size_)
        {
            if (s->last_status_valid_)
            {
                s->present_ = true;
                s->recv_status (s->last_raw_status_ + header_size);
                // On initialisation, copy sequence number.
                if (!s->seq_)
                    s->seq_ = s->last_raw_status_[1];
                // Then, slave is synchronised if status sequence number match
                // the current one.
                else if (s->last_raw_status_[1] != s->seq_)
                    all_slave_sync = false;
            }
            else
                all_slave_sync = false;
        }
    }
    // Start next update cycle.
    update_slave_ = slaves_;
    start_update_status ();
    // Was sync?
    return all_slave_sync;
}

void
I2cQueue::register_slave (Slave &slave)
{
    slave.next_ = slaves_;
    slaves_ = &slave;
}

void
I2cQueue::send (Slave &slave, const uint8_t *command, int size,
                CommandType type)
{
    Command *c;
    ucoo::assert (size <= command_size_max);
    unsigned int transient_commands_index = 0;
    // If slave is not there, drop.
    if (!slave.present_)
        return;
    // Find command buffer.
    if (type == TRANSIENT)
    {
        transient_commands_index = (slave.transient_commands_index_ + 1) % 2;
        c = &slave.transient_commands_[transient_commands_index];
    }
    else
    {
        if (queue_head_ == queue_next (queue_tail_))
        {
            ucoo::assert (can_drop_);
            return;
        }
        c = &queue_[queue_tail_];
    }
    // Fill and copy data.
    // Warning: transient commands need double buffering so that it can be
    // updated even if being transmitted.  However, this can fail if two
    // updates are done in a really short interval (if both updates are done
    // during a transfer).  Don't do that!
    c->slave = &slave;
    c->type = type;
    if (type != RAW)
    {
        c->raw_size = header_size + size;
        std::copy (command, command + size, c->raw + header_size);
        if (type == RELIABLE)
            c->raw[1] = c->slave->seq_next ();
        else
            c->raw[1] = 0;
        c->raw[0] = ucoo::crc8_compute (c->raw + 1, c->raw_size - 1);
    }
    else
    {
        c->raw_size = size;
        std::copy (command, command + size, c->raw);
    }
    // Commit.
    ucoo::barrier ();
    if (type == TRANSIENT)
        slave.transient_commands_index_ = transient_commands_index;
    else
        queue_tail_ = queue_next (queue_tail_);
}

void
I2cQueue::finished (int status)
{
    switch (update_state_)
    {
    case RECV_STATUS:
        end_update_status (status);
        start_update_status ();
        break;
    case SEND_QUEUE:
        end_send_queue (status);
        start_send_queue ();
        break;
    case SEND_TRANSIENT:
        end_send_transient (status);
        start_send_transient ();
        break;
    default:
        ucoo::assert_unreachable ();
    }
}

void
I2cQueue::start_update_status ()
{
    // Skip over slaves with no status.
    while (update_slave_ && !update_slave_->raw_status_size_)
        update_slave_ = update_slave_->next_;
    // Update if found, else next step.
    if (update_slave_)
    {
        update_state_ = RECV_STATUS;
        i2c_.recv (update_slave_->address_, (char *) update_slave_->last_raw_status_,
                   update_slave_->raw_status_size_);
    }
    else
    {
        update_slave_ = slaves_;
        start_send_queue ();
    }
}

void
I2cQueue::end_update_status (int status)
{
    bool ok = false;
    if (status == update_slave_->raw_status_size_)
    {
        // Check CRC.
        Slave *s = update_slave_;
        uint8_t crc = ucoo::crc8_compute (s->last_raw_status_ + 1,
                                          s->raw_status_size_ - 1);
        if (crc == s->last_raw_status_[0])
        {
            ok = true;
            // Check pending command.
            if (queue_head_ != queue_tail_)
            {
                Command *c = &queue_[queue_head_];
                if (c->slave == update_slave_
                    && c->type == RELIABLE
                    && c->raw[1] == s->last_raw_status_[1])
                {
                    queue_head_ = queue_next (queue_head_);
                    queue_timeout_ = 0;
                }
            }
        }
    }
    update_slave_->last_status_valid_ = ok;
    update_slave_ = update_slave_->next_;
}

void
I2cQueue::start_send_queue ()
{
    if (!queue_timeout_
        && queue_head_ != queue_tail_)
    {
        update_state_ = SEND_QUEUE;
        update_command_ = &queue_[queue_head_];
        i2c_.send (update_command_->slave->address_,
                   (char *) update_command_->raw, update_command_->raw_size);
    }
    else
    {
        if (queue_timeout_)
            queue_timeout_--;
        start_send_transient ();
    }
}

void
I2cQueue::end_send_queue (int status)
{
    if (status == update_command_->raw_size)
    {
        if (update_command_->type == RAW)
            // OK, next.
            queue_head_ = queue_next (queue_head_);
        else
            // Wait for acknowledgement.
            queue_timeout_ = retry_timeout;
    }
    else
        // Big problem, retry at next cycle.
        queue_timeout_ = 1;
}

void
I2cQueue::start_send_transient ()
{
    // Skip over slaves with no transient command, or not synchronised.
    for (; update_slave_; update_slave_ = update_slave_->next_)
    {
        if (!update_slave_->last_status_valid_
            || update_slave_->last_raw_status_[1] != update_slave_->seq_)
            continue;
        update_command_ = &update_slave_->transient_commands_
            [update_slave_->transient_commands_index_];
        if (update_command_->raw_size != 0)
            break;
    }
    // If one found, send it, else done.
    if (update_slave_)
    {
        update_state_ = SEND_TRANSIENT;
        i2c_.send (update_slave_->address_, (char *) update_command_->raw,
                   update_command_->raw_size);
    }
    else
    {
        update_slave_ = slaves_;
        update_state_ = IDLE;
    }
}

void
I2cQueue::end_send_transient (int status)
{
    if (status == update_command_->raw_size)
    {
        // Transient commands are sent once.
        update_command_->raw_size = 0;
    }
}

