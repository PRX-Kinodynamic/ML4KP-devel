/**
 * @file timer.hpp
 * @brief <b>A clock for measuring time using system time.</b>
 * @authors Kostas Bekris
 */
#pragma once

#include <chrono>
// #include <sys/time.h>

namespace prx
{
/**
 * A class that can express current time as a single number and which can
 * also provide methods that measure elapsed time between consecutive calls.
 *
 * @brief <b>A clock for measuring time using system time.</b>
 * @authors Kostas Bekris, Edgar Granados
 *
 */
class timer_t
{
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double>;

protected:
  TimePoint _start;
  TimePoint _finish;

  /** @brief When the timer was started */
  // struct timeval start;
  /** @brief When the timer finished */
  // struct timeval finish;
  /** @brief How much time has elapsed since start */
  double elapsed;

public:
  timer_t()
  {
    reset();
  }

  virtual ~timer_t()
  {
  }

  /**
   * Gets the time in seconds
   *
   * @brief Gets the time in seconds
   * @return The time in seconds
   */
  double get_time_in_secs();

  /**
   * Resets the timer
   *
   * @brief Resets the timer
   */
  void reset()
  {
    _start = Clock::now();
    // const auto finish{ std::chrono::steady_clock::now() };
  }

  double operator()()
  {
    const TimePoint end{ Clock::now() };
    const Duration diff{ end - _start };
    return diff.count();
  }
  /**
   * Measures the timer and returns the value in seconds
   *
   * @brief Measures the timer
   * @return The elapsed time in seconds
   */
  double measure()
  {
    return this->operator()();
  }

  /**
   * Performs measure and reset
   *
   * @brief Calls measure and reset
   * @return The elapsed time in seconds
   */
  double measure_reset()
  {
    const double elapsed{ measure() };
    reset();
    return elapsed;
  }

  /**
   * Adds a delay to the clock
   *
   * @brief Adds a delay to the clock
   * @param delay Determines how much delay is added
   */
  // void add_delay_user_clock(double delay)
  // {
  //   _start = _start - Duration(delay);
  // }
};
}  // namespace prx