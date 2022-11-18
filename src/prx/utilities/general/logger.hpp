#pragma once
#include <fstream>

namespace prx
{
// TODO: is this the best (faster) way of logging?
class logger_t
{
public:
  logger_t(const std::string& file_name, char separator = ' ')
    : ofs_logger(file_name.c_str(), std::ofstream::out | std::ofstream::trunc)
  {
    sep = separator;
  }

  virtual ~logger_t()
  {
    ofs_logger.flush();
    ofs_logger.close();
  }

  template <typename T>
  void add_values(T values)
  {
    for (auto v : values)
    {
      ofs_logger << v << sep;
    }
    ofs_logger << '\n';
  }

  template <class... Types>
  void log(Types... types)
  {
    log_<0>(types...);
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
  char sep;
};

}  // namespace prx