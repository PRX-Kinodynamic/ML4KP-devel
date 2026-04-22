#pragma once
#include <ostream>
#include <regex>
// #include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/template_utils.hpp"

namespace prx
{
namespace dbg
{
constexpr std::string_view normal = "\033[0m";
constexpr std::string_view red{ "\033[31m" };
constexpr std::string_view green{ "\033[32m" };
constexpr std::string_view yellow{ "\033[33m" };

inline void print_variables(std::ostream& stream, const std::string& name)
{
  stream << std::endl;
}

template <typename Value, std::enable_if_t<prx::utilities::is_streamable<Value>::value, bool> = true>
inline void print_value(std::ostream& stream, const Value& value)
{
  stream << value << " ";
}

template <typename Value, std::enable_if_t<prx::utilities::is_iterable<Value>::value and
                                               not prx::utilities::is_streamable<Value>::value,
                                           bool> = true>
inline void print_value(std::ostream& stream, const Value& value)
{
  for (auto e : value)
  {
    print_value(stream, e);
  }
}

inline void print_values(std::ostream& stream)
{
  stream << "\n";
}

template <typename Var0, class... Vars>
inline void print_values(std::ostream& stream, const Var0 var, Vars... vars)
{
  print_value(stream, var);
  print_values(stream, vars...);
}

template <typename Var0, class... Vars>
inline void print_variables(std::ostream& stream, const std::string& name, const Var0 var, Vars... vars)
{
  const std::regex regex(",(\\s*)+");
  std::string var_name{ name };
  std::string other_names{ "" };
  std::smatch match;                          // <-- need a match object
  if (std::regex_search(name, match, regex))  // <-- use it here to get the match
  {
    const int split_on = match.position();  // <-- use the match position
    var_name = name.substr(0, split_on);
    other_names = name.substr(split_on + match.length());  // <-- also, skip the whole math
  }

  stream << yellow << var_name << ": " << normal;
  print_value(stream, var);
  print_variables(stream, other_names, vars...);
}

}  // namespace dbg
}  // namespace prx
#define PRX_VALUES_TO_STREAM(ofs, ...) prx::dbg::print_values(ofs, __VA_ARGS__);
#define PRX_PRINT_VALUES(...) prx::dbg::print_values(std::cout, __VA_ARGS__);
#define PRX_DBG_VARS(...) prx::dbg::print_variables(std::cout, #__VA_ARGS__, __VA_ARGS__);
#define PRX_MSG(MSG)                                                                                                   \
  {                                                                                                                    \
    std::stringstream strstr;                                                                                          \
    strstr << MSG;                                                                                                     \
    std::string msg{ strstr.str() };                                                                                   \
    PRX_DBG_VARS(msg)                                                                                                  \
  };

#define PRX_WARNING(MSG)                                                                                               \
  {                                                                                                                    \
    std::stringstream strstr;                                                                                          \
    strstr << "WARNING - [" << __PRETTY_FUNCTION__ << "] ";                                                            \
    strstr << MSG;                                                                                                     \
    std::string msg{ strstr.str() };                                                                                   \
    PRX_DBG_VARS(msg)                                                                                                  \
  };

#define PRX_MSG_VARS(MSG, ...)                                                                                         \
  {                                                                                                                    \
    std::stringstream strstr;                                                                                          \
    strstr << MSG;                                                                                                     \
    std::string msg{ strstr.str() };                                                                                   \
    PRX_DBG_VARS(msg, __VA_ARGS__)                                                                                     \
  };
