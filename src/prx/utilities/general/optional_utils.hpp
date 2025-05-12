#pragma once
#include <fstream>

namespace prx
{
namespace utilities
{

// At least one value is not boost::optional
template <typename CheckType>
bool any_of(const CheckType& check)
{
  return static_cast<bool>(check);
}

template <typename CheckType, typename... CheckTypes>
bool any_of(const CheckType& check, CheckTypes... checks)
{
  return (check or any_of(checks...));
  // static const std::size_t value = sizeof...(checks);
}
}  // namespace utilities
}  // namespace prx