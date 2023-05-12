#pragma once
#include <type_traits>
#include "prx/utilities/general/template_utils.hpp"
namespace prx
{
namespace utilities
{

template <typename T, typename StringType,
          std::enable_if_t<std::is_integral<T>::value && std::is_same<StringType, std::string>::value, bool> = true>
inline T convert_to(const StringType& str)
{
  return std::stoi(str);
}

template <
    typename T, typename StringType,
    std::enable_if_t<std::is_floating_point<T>::value && std::is_same<StringType, std::string>::value, bool> = true>
inline T convert_to(const StringType& str)
{
  return std::stod(str);
}

template <typename To, typename From,
          std::enable_if_t<!std::is_same<To, From>::value && std::is_floating_point<To>::value &&
                               std::is_floating_point<From>::value,
                           bool> = true>
inline To convert_to(const From& value)
{
  return static_cast<To>(value);
}

template <typename To, typename From,
          std::enable_if_t<std::is_floating_point<To>::value && std::is_integral<From>::value, bool> = true>
inline To convert_to(const From& value)
{
  return static_cast<To>(value);
}

template <typename To, typename From, std::enable_if_t<std::is_same<To, From>::value, bool> = true>
inline To convert_to(const From& value)
{
  return value;
}

}  // namespace utilities
}  // namespace prx