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

    do
    {
      line = next_line();
      block.emplace_back(line);
    } while (has_next_line() && line.size() > 0);
    return block;
  }

private:
  const char _sep;
  const std::string _filename;
  std::ifstream file;
};

}  // namespace utilities
}  // namespace prx