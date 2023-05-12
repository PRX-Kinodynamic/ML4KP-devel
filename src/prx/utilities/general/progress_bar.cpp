#include "prx/utilities/general/progress_bar.hpp"

namespace prx
{

progress_bar_t::progress_bar_t(std::size_t _total, std::string _label)
{
  total = _total;
  label = _label;
  next_current = -1;
}

void progress_bar_t::update(const std::size_t current_value)
{
  const double step{ 2.0 * total / 100.0 };
  if (next_current == -1)
  {
    std::cout << "0%   10   20   30   40   50   60   70   80   90  100%" << std::endl;
    std::cout << "[--------------------------------------------------]" << std::endl;
    std::cout << "[";
    next_current = static_cast<std::size_t>(step);
    // next_current = current_value;
    // return;
  }
  if (current_value < next_current)
    return;

  // int i = 0;

  // std::cout << label;
  // std::cout << "[";
  // PRX_DEBUG_VAR_2(total, step);
  // PRX_DEBUG_VAR_2(current_value, next_current);
  for (double i = next_current; i < next_current + step; i += step)
  {
    std::cout << "*" << std::flush;
  }
  next_current += step;
  if (next_current >= total)
  {
    std::cout << "]" << std::endl;
    return;
  }
  // for (; i < 100; i += 2)
  // {
  //   std::cout << " ";
  // }
}

}  // namespace prx