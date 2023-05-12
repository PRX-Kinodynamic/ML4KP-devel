#pragma once
#include <fstream>

#include "prx/utilities/general/constants.hpp"

namespace prx
{
// TODO: is this the best (faster) way of logging?
class logger_t
{
public:
  logger_t() = default;
  logger_t(const std::string& filename, char separator = prx::separating_value, std::string endline = "\n")
    : logger_t(filename, std::string(1, separator), endline)
  {
  }
  logger_t(const std::string& filename, std::string separator, std::string endline = "\n")
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

  void close()
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
  void operator()(Types... types)
  {
    log_<0, true>(types...);
  }

  template <bool Add_NL = true, class... Types>
  void log(Types... types)
  {
    log_<0, Add_NL>(types...);
  }

  const std::string get_filename()
  {
    return _filename;
  }

  inline void newline()
  {
    ofs_logger << _endline;
    ofs_logger.flush();
  }

protected:
  template <std::size_t Ix, bool Add_NL, class... Types,
            std::enable_if_t<Add_NL && Ix == sizeof...(Types), bool> = true>
  inline void log_(Types... types)
  {
    newline();
  }
  template <std::size_t Ix, bool Add_NL, class... Types,
            std::enable_if_t<!Add_NL && Ix == sizeof...(Types), bool> = true>
  inline void log_(Types... types)
  {
  }

  template <
      std::size_t Ix, bool Add_NL, class... Types, std::enable_if_t<(Ix < sizeof...(Types)), bool> = true,
      std::enable_if_t<prx::utils::is_iterable<typename std::tuple_element<Ix, std::tuple<Types...>>::type>{} &&
                           !prx::utils::is_streamable<typename std::tuple_element<Ix, std::tuple<Types...>>::type>{},
                       bool> = true>
  void log_(Types... types)
  {
    // std::cout << "is_iterable && !is_streamable: " << std::endl;
    std::tuple<Types...> tuple{ types... };
    for (auto ti : std::get<Ix>(tuple))
    {
      std::cout << ti << sep;
      ofs_logger << ti << sep;
    }
    log_<Ix + 1, Add_NL>(types...);
  }

  template <
      std::size_t Ix, bool Add_NL, class... Types, std::enable_if_t<(Ix < sizeof...(Types)), bool> = true,
      std::enable_if_t<!prx::utils::is_iterable<typename std::tuple_element<Ix, std::tuple<Types...>>::type>{} ||
                           prx::utils::is_streamable<typename std::tuple_element<Ix, std::tuple<Types...>>::type>{},
                       bool> = true>
  inline void log_(Types... types)
  {
    std::tuple<Types...> tuple{ types... };
    ofs_logger << std::get<Ix>(tuple) << sep;
    log_<Ix + 1, Add_NL>(types...);
    // std::cout << "!is_iterable || is_streamable: " << std::get<Ix>(tuple) << std::endl;
  }

  std::ofstream ofs_logger;
  std::string _filename;
  std::string sep;
  std::string _endline;
};  // namespace prx

}  // namespace prx