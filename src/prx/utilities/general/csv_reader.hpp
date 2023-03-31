#pragma once
#include <fstream>

#include "prx/utilities/defs.hpp"

namespace prx
{
namespace utilities
{
class csv_reader_t
{
public:
  using Line = std::vector<std::string>;
  using Block = std::vector<std::vector<std::string>>;

  // member typedefs provided through inheriting from std::iterator
  class iterator
  {
    using iterator_category = std::output_iterator_tag;
    using value_type = Line;  // crap
    using difference_type = Line;
    using pointer = const Line*;
    using reference = Line;
    csv_reader_t* _ptr;
    Line _line;

  public:
    explicit iterator(csv_reader_t* ptr) : _ptr(ptr), _line()
    {
      if (_ptr != nullptr)
      {
        if (_ptr->has_next_line())
          _line = _ptr->next_line();
        PRX_DEBUG_VAR_1(_line.size());
      }
    }

    iterator& operator++()
    {
      if (_ptr != nullptr && _ptr->has_next_line())
      {
        _line = _ptr->next_line();
      }
      else
      {
        _ptr = nullptr;
        _line = Line();
      }
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
      return _ptr == other._ptr && _line == other._line;
    }
    bool operator!=(iterator other) const
    {
      return !(*this == other);
    }
    reference operator*() const
    {
      return _line;
    }
  };
  iterator begin()
  {
    return iterator(this);
  }
  iterator end()
  {
    return iterator(nullptr);
  }

  csv_reader_t() = delete;
  csv_reader_t(const std::string filename, const char separator = ' ')
    : _filename(filename), _sep(separator), file(filename.c_str())
  {
  }

  ~csv_reader_t()
  {
  }

  inline bool has_next_line() const
  {
    return !file.eof();
  }
  inline bool is_open() const
  {
    return file.is_open();
  }

  Line next_line()
  {
    std::string line;
    std::getline(file, line);
    return prx::split<std::string>(line, _sep);
  }

  template <typename T>
  std::vector<T> next_line()
  {
    std::string line;
    std::getline(file, line);
    return prx::split<T>(line, _sep);
  }

  template <typename Function>
  Line next_line(Function f)
  {
    Line line;
    do
    {
      line = next_line();
      if (f(line))
      {
        return line;
      }
    } while (has_next_line());
    return Line();
  }

  /**
   * @brief      Get the next line that satisfies line[idx] == value
   *
   * @param[in]  value     Value
   * @param[in]  idx   The index
   *
   * @return     Line that satisfies "line[idx] == value"
   */
  Line next_line(const std::string& value, const std::size_t idx)
  {
    return next_line([&](const Line& line) { return line.size() > idx && line[idx] == value; });
  }

  Block next_block()
  {
    Line line;
    Block block;

    while (has_next_line())
    {
      line = next_line();
      if (line.size() == 0)
        break;
      block.emplace_back(line);
    }
    return block;
  }

private:
  const char _sep;
  const std::string _filename;
  std::ifstream file;
};

}  // namespace utilities
}  // namespace prx