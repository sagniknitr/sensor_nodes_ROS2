/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <bondcpp/bond.hpp>
// // #include <boost/thread/thread_time.hpp>
// // #include <boost/date_time/posix_time/posix_time_types.hpp>

#ifdef _WIN32
#include <Rpc.h>
#else
#include <uuid/uuid.h>
#endif

#include <algorithm>
#include <string>
#include <vector>

namespace bond {

static std::string makeUUID()
{
#ifdef _WIN32
  UUID uuid;
  UuidCreate(&uuid);
  unsigned char *str;
  UuidToStringA(&uuid, &str);
  std::string return_string(reinterpret_cast<char *>(str));
  RpcStringFreeA(&str);
  return return_string;
#else
  uuid_t uuid;
  uuid_generate_random(uuid);
  const int UUID_ARRAY_BOUND = 40;
  char uuid_str[UUID_ARRAY_BOUND];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
#endif
}

Bond::Bond(const std::string &topic, const std::string &id,
           std::function<void(void)> on_broken,
           std::function<void(void)> on_formed)
  : 

  bondsm_(new BondSM(this)),
  sm_(*bondsm_),
  topic_(topic),
  id_(id),
  instance_id_(makeUUID()),
  on_broken_(on_broken),
  on_formed_(on_formed),
  sisterDiedFirst_(false),
  started_(false),

  connect_timer_(rclcpp::Duration(0, 0), std::bind(&Bond::onConnectTimeout, this)),
  heartbeat_timer_(rclcpp::Duration(0, 0), std::bind(&Bond::onHeartbeatTimeout, this)),
  disconnect_timer_(rclcpp::Duration(0, 0), std::bind(&Bond::onDisconnectTimeout, this))
{
  setConnectTimeout(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT);
  setDisconnectTimeout(bond::msg::Constants::DEFAULT_DISCONNECT_TIMEOUT);
  setHeartbeatTimeout(bond::msg::Constants::DEFAULT_HEARTBEAT_TIMEOUT);
  setHeartbeatPeriod(bond::msg::Constants::DEFAULT_HEARTBEAT_PERIOD);
}

Bond::~Bond()
{
  if (!started_) {
    return;
  }

  breakBond();
  if (!waitUntilBroken(rclcpp::Duration(1e9))) {
    RCUTILS_LOG_INFO_NAMED("bond","Bond failed to break on destruction %s (%s)", id_.c_str(), instance_id_.c_str());
  }

  // Must destroy the subscription before locking mutex_: shutdown()
  // will block until the status callback completes, and the status
  // callback locks mutex_ (in flushPendingCallbacks).
  // // sub_.shutdown();

  // Stops the timers before locking the mutex.  Makes sure none of
  // the callbacks are running when we aquire the mutex.
  // // publishingTimer_.stop();
  // // connect_timer_.cancel();
  // // heartbeat_timer_.cancel();
  // // disconnect_timer_.cancel();

  // // std::lock_guard<std::mutex> lock(mutex_);
  // // pub_.shutdown();
}


void Bond::setConnectTimeout(double dur)
{
  if (started_) {
    RCUTILS_LOG_ERROR_NAMED("bond", "Cannot set timeouts after calling start()");
    return;
  }

  connect_timeout_ = dur;
  connect_timer_.setDuration(rclcpp::Duration(connect_timeout_*1e9));
}

void Bond::setDisconnectTimeout(double dur)
{
  if (started_) {
    RCUTILS_LOG_ERROR_NAMED("bond","Cannot set timeouts after calling start()");
    return;
  }

  disconnect_timeout_ = dur;
  disconnect_timer_.setDuration(rclcpp::Duration(disconnect_timeout_*1e9));
}

void Bond::setHeartbeatTimeout(double dur)
{
  if (started_) {
    RCUTILS_LOG_ERROR_NAMED("bond","Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_timeout_ = dur;
  heartbeat_timer_.setDuration(rclcpp::Duration(heartbeat_timeout_*1e9));
}

void Bond::setHeartbeatPeriod(double dur)
{
  if (started_) {
    RCUTILS_LOG_ERROR_NAMED("bond", "Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_period_ = dur;
}

/*void Bond::setCallbackQueue(ros::CallbackQueueInterface *queue)
{
  if (started_) {
    ROS_ERROR("Cannot set callback queue after calling start()");
    return;
  }

  nh_.setCallbackQueue(queue);
}
*/

void Bond::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  connect_timer_.reset();
  pub_ = nh_->create_publisher<bond::msg::Status>(topic_);
  sub_ = nh_->create_subscription<bond::msg::Status>(topic_, std::bind(&Bond::bondStatusCB, this, std::placeholders::_1));

  publishingTimer_ = nh_->create_wall_timer(
    std::chrono::seconds(static_cast<int64_t>(heartbeat_period_)), std::bind(&Bond::doPublishing, this));
  started_ = true;
}

void Bond::setFormedCallback(std::function<void(void)> on_formed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  on_formed_ = on_formed;
}

void Bond::setBrokenCallback(std::function<void(void)> on_broken)
{
  std::lock_guard<std::mutex> lock(mutex_);
  on_broken_ = on_broken;
}

/*bool Bond::waitUntilFormed(rclcpp::Duration timeout)
{
  return waitUntilFormed(rclcpp::Duration(timeout));
}
*/
bool Bond::waitUntilFormed(rclcpp::Duration timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time deadline(clock.now() + timeout);

  while (sm_.getState().getId() == SM::WaitingForSister.getId()) {
    if (!rclcpp::ok()) {
      break;
    }

    rclcpp::Duration wait_time = rclcpp::Duration(1e8);
    if (timeout >= rclcpp::Duration(0, 0)) {
      wait_time = std::min(wait_time, deadline - clock.now());
    }

    if (wait_time <= rclcpp::Duration(0, 0)) {
      break;  // The deadline has expired
    }

   // // condition_.wait_for(mutex_, std::chrono::milliseconds(
      // //static_cast<int64_t>(wait_time.nanoseconds() / 1000000.0f)));
  }
  return sm_.getState().getId() != SM::WaitingForSister.getId();
}

/*bool Bond::waitUntilBroken(rclcpp::Duration timeout)
{
  return waitUntilBroken(rclcpp::Duration(timeout));
}*/
bool Bond::waitUntilBroken(rclcpp::Duration timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time deadline(clock.now() + timeout);

  while (sm_.getState().getId() != SM::Dead.getId()) {
    if (!rclcpp::ok()) {
      break;
    }

    rclcpp::Duration wait_time = rclcpp::Duration(1e8);
    if (timeout >= rclcpp::Duration(0, 0)) {
      wait_time = std::min(wait_time, deadline - clock.now());
    }

    if (wait_time <= rclcpp::Duration(0, 0)) {
      break;  // The deadline has expired
    }

    // // condition_.timed_wait(mutex_, boost::posix_time::milliseconds(
     // // static_cast<int64_t>(wait_time.toSec() * 1000.0f)));
  }
  return sm_.getState().getId() == SM::Dead.getId();
}

bool Bond::isBroken()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sm_.getState().getId() == SM::Dead.getId();
}

void Bond::breakBond()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sm_.getState().getId() != SM::Dead.getId()) {
      sm_.Die();
      publishStatus(false);
    }
  }
  flushPendingCallbacks();
}


void Bond::onConnectTimeout()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sm_.ConnectTimeout();
  }
  flushPendingCallbacks();
}
void Bond::onHeartbeatTimeout()
{
  // Checks that heartbeat timeouts haven't been disabled globally.
  bool disable_heartbeat_timeout;
  // //nh_.param(bond::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, disable_heartbeat_timeout, false);
  if (disable_heartbeat_timeout) {
    RCUTILS_LOG_INFO_NAMED("bond", "Heartbeat timeout is disabled.  Not breaking bond (topic: %s, id: %s)",
             topic_.c_str(), id_.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    sm_.HeartbeatTimeout();
  }
  flushPendingCallbacks();
}
void Bond::onDisconnectTimeout()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sm_.DisconnectTimeout();
  }
  flushPendingCallbacks();
}

void Bond::bondStatusCB(const bond::msg::Status::ConstSharedPtr msg)
{
  // Filters out messages from other bonds and messages from ourself
  if (msg->id == id_ && msg->instance_id != instance_id_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (sister_instance_id_.empty()) {
        sister_instance_id_ = msg->instance_id;
      }
      if (sister_instance_id_ != msg->instance_id) {
        RCUTILS_LOG_INFO_NAMED("bond",
          "More than two locations are trying to use a single bond (topic: %s, id: %s).  "
          "You should only instantiate at most two bond instances for each (topic, id) pair.",
          topic_.c_str(), id_.c_str());
        return;
      }

      if (msg->active) {
        sm_.SisterAlive();
      } else {
        sm_.SisterDead();

        // Immediate ack for sister's death notification
        if (sisterDiedFirst_) {
          publishStatus(false);
        }
      }
    }
    flushPendingCallbacks();
  }
}

void Bond::doPublishing()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (sm_.getState().getId() == SM::WaitingForSister.getId() ||
      sm_.getState().getId() == SM::Alive.getId()) {
    publishStatus(true);
  } else if (sm_.getState().getId() == SM::AwaitSisterDeath.getId()) {
    publishStatus(false);
  } else {
    publishingTimer_->cancel();
    RCUTILS_LOG_INFO_NAMED("bond","the timer should stop");
  }
}

void Bond::publishStatus(bool active)
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  bond::msg::Status::SharedPtr msg(new bond::msg::Status);
  msg->header.stamp = clock.now();
  msg->id = id_;
  msg->instance_id = instance_id_;
  msg->active = active;
  msg->heartbeat_timeout = heartbeat_timeout_;
  msg->heartbeat_period = heartbeat_period_;
  pub_->publish(msg);
}


void Bond::flushPendingCallbacks()
{
  std::vector<std::function<void(void)> > callbacks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks = pending_callbacks_;
    pending_callbacks_.clear();
  }

  for (size_t i = 0; i < callbacks.size(); ++i) {
    callbacks[i]();
  }
}

}  // namespace bond


void BondSM::Connected()
{
  b->connect_timer_.cancel();
  b->condition_.notify_all();
  if (b->on_formed_) {
    b->pending_callbacks_.push_back(b->on_formed_);
  }
}

void BondSM::SisterDied()
{
  b->sisterDiedFirst_ = true;
}

void BondSM::Death()
{
  b->condition_.notify_all();
  b->heartbeat_timer_.cancel();
  b->disconnect_timer_.cancel();
  if (b->on_broken_) {
    b->pending_callbacks_.push_back(b->on_broken_);
  }
}

void BondSM::Heartbeat()
{
  b->heartbeat_timer_.reset();
}

void BondSM::StartDying()
{
  b->heartbeat_timer_.cancel();
  b->disconnect_timer_.reset();
  // // b->publishingTimer_->setPeriod(rclcpp::Duration(bond::msg::Constants::DEAD_PUBLISH_PERIOD*1e9));
}
