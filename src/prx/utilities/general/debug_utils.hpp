#pragma once
#include <regex>
#include "prx/utilities/general/template_utils.hpp"

namespace prx
{
namespace dbg
{

inline void print_variables(std::string name)
{
  std::cout << std::endl;
}

template <typename Value, std::enable_if_t<prx::utilities::is_streamable<Value>::value, bool> = true>
inline void print_value(const Value& value)
{
  std::cout << value << " ";
}

template <typename Value, std::enable_if_t<prx::utilities::is_iterable<Value>::value and
                                               not prx::utilities::is_streamable<Value>::value,
                                           bool> = true>
inline void print_value(const Value& value)
{
  for (auto e : value)
  {
    print_value(e);
  }
}

template <typename Var0, class... Vars>
inline void print_variables(std::string name, Var0 var, Vars... vars)
{
  const std::regex regex(",(\\s*)+");
  std::string var_name{ name };
  std::string other_names{ "" };
  std::smatch match;  // <-- need a match object
  // std::cout << "name: " << name << std::endl;
  if (std::regex_search(name, match, regex))  // <-- use it here to get the match
  {
    const int split_on = match.position();  // <-- use the match position
    var_name = name.substr(0, split_on);
    other_names = name.substr(split_on + match.length());  // <-- also, skip the whole math
  }

  std::cout << var_name << ": ";
  print_value(var);
  print_variables(other_names, vars...);
}

}  // namespace dbg
}  // namespace prx
#define PRX_DBG_VARS(...) prx::dbg::print_variables(#__VA_ARGS__, __VA_ARGS__);
