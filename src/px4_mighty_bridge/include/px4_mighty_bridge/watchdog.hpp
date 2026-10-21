// Copyright 2026, MIGHTY Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#ifndef PX4_MIGHTY_BRIDGE__WATCHDOG_HPP_
#define PX4_MIGHTY_BRIDGE__WATCHDOG_HPP_

#include <chrono>
#include <mutex>

namespace px4_mighty_bridge
{

/// Monotonic setpoint-staleness detector.
///
/// The watchdog deliberately uses std::chrono::steady_clock rather than ROS
/// time: a paused Gazebo/clock topic must not silently disable the safety
/// fallback.  A watchdog that has never been fed is considered expired.
class Watchdog
{
public:
  explicit Watchdog(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(250));

  /// Mark that a valid setpoint source has just been observed.
  void feed();

  /// Forget the last feed (e.g. after a manual override or disarm).
  void reset();

  bool expired() const;

  bool hasBeenFed() const;

  std::chrono::milliseconds timeout() const;

  void setTimeout(std::chrono::milliseconds timeout);

  /// Time since the last feed.  Returns the timeout when never fed.
  std::chrono::milliseconds timeSinceLastFeed() const;

private:
  using Clock = std::chrono::steady_clock;

  mutable std::mutex mutex_;
  std::chrono::milliseconds timeout_;
  Clock::time_point last_feed_;
  bool fed_{false};
};

}  // namespace px4_mighty_bridge

#endif  // PX4_MIGHTY_BRIDGE__WATCHDOG_HPP_
