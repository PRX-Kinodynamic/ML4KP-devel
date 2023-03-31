#pragma once
#include <fstream>

#include "prx/utilities/general/constants.hpp"

namespace prx
{
// TODO: is this the best (faster) way of logging?
class logger_t
{
public:
  logger_t(const std::string& filename, char separator = prx::separating_value, std::string endline = "")
    : logger_t(filename, std::string(1, separator), endline)
  {
  }
  logger_t(const std::string& filename, std::string separator, std::string endline = "")
    : ofs_logger(filename.c_str(), std::ofstream::out | std::ofstream::trunc)
    , _filename(filename)
    , sep(separator)
    , _endline(endline)
  {
  }

  virtual ~logger_t()
  {
    ofs_logger.flush();
    ofs_logger.close();
  }

  template <typename T>
  void add_values(T values, const std::string& last_str = "\n")
  {
    for (auto v : values)
    {
      ofs_logger << v << sep;
    }
    ofs_logger << last_str;
  }

  template <class... Types>
  void log(Types... types)
  {
    log_<0>(types...);
  }

  const std::string get_filename()
  {
    return _filename;
  }

protected:
  template <std::size_t Ix, class... Types, std::enable_if_t<Ix == sizeof...(Types), bool> = true>
  inline void log_(Types... types)
  {
    ofs_logger << std::endl;
  }

  template <std::size_t Ix, class... Types, std::enable_if_t<(Ix < sizeof...(Types)), bool> = true>
  inline void log_(Types... types)
  {
    std::tuple<Types...> tuple{ types... };
    ofs_logger << std::get<Ix>(tuple) << sep;
    log_<Ix + 1>(types...);
  }

  std::ofstream ofs_logger;
  std::string _filename;
  std::string sep;
  std::string _endline;
};  // namespace prx

}  // namespace prx