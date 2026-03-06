#include <fstream>
#include <memory>
#include "prx/utilities/general/timer.hpp"
// #include <ros/time.h>

namespace prx
{
class time_profiler_t
{
  time_profiler_t()
  {
  }

  void header(const std::string line)
  {
    _ofs << "# " << line << "\n";
  }

  void reset()
  {
    _ofs << "\n";
    // _timer.start
  }

  void measure()
  {
    _ofs << _timer() << " ";
    _timer.reset();
  }

public:
  using TimeProfilerPtr = std::shared_ptr<time_profiler_t>;
  time_profiler_t(const std::string filename) : _ofs(filename)
  {
  }

  static void reset(TimeProfilerPtr profiler)
  {
    if (profiler)
      profiler->reset();
  }

  static void measure(TimeProfilerPtr profiler)
  {
    if (profiler)
      profiler->measure();
  }

protected:
  // Only record when this is on. Could be faster by using compilation directives, but this is more usable...
  bool _record;

  timer_t _timer;

  std::ofstream _ofs;
};

// #define PRX_TIME_RESET(profiler)                                                                                       \
//   if (profiler)                                                                                                        \
//     profiler.reset();
// #define PRX_TIME_PROFILER(profiler)                                                                                    \
//   if (profiler)                                                                                                        \
//     timer();

}  // namespace prx