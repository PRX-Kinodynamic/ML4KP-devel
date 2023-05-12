#pragma once

#include <vector>
#include <numeric>
#include "prx/utilities/defs.hpp"

namespace prx
{
class progress_bar_t
{
public:
  progress_bar_t(std::size_t total, std::string label = "");
  ~progress_bar_t() = default;

  void update(const std::size_t current_value);

  class iterator
  {
    using iterator_category = std::output_iterator_tag;
    using value_type = std::size_t;  // crap
    using difference_type = std::size_t;
    using pointer = const std::size_t*;
    using reference = std::size_t;

    std::size_t _idx;
    progress_bar_t* _bar;

  public:
    explicit iterator(std::size_t idx, progress_bar_t* bar) : _idx(idx)
    {
      _bar = bar;
    }

    iterator& operator++()
    {
      _idx++;
      _bar->update(_idx);

      return *this;
    }
    iterator operator++(int)
    {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const
    {
      return _idx == other._idx;
    }
    bool operator!=(iterator other) const
    {
      return !(*this == other);
    }
    reference operator*() const
    {
      return _idx;
    }
  };

  iterator begin()
  {
    return iterator(0, this);
  }
  iterator end()
  {
    return iterator(total, nullptr);
  }

private:
  std::size_t total;
  double next_current;
  std::string label;
};
}  // namespace prx