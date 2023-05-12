#pragma once
#include <memory>
#include <type_traits>

namespace prx
{
namespace utils
{

// Taken from:
// https://stackoverflow.com/questions/41853159/how-to-detect-if-a-type-is-shared-ptr-at-compile-time
// This is though of an extension of std::is_pointer<T> to detect shared_ptr(s).
template <class T>
struct is_ptr_type : std::false_type
{
};

template <class T>
struct is_ptr_type<T*> : std::true_type
{
};

template <class T>
struct is_ptr_type<std::shared_ptr<T>> : std::true_type
{
};

template <class T>
struct is_ptr_type<std::unique_ptr<T>> : std::true_type
{
};

template <class T>
struct is_ptr_type<std::weak_ptr<T>> : std::true_type
{
};

template <typename T, typename = void>
struct is_iterable : std::false_type
{
};

// this gets used only when we can call std::begin() and std::end() on that type
template <typename T>
struct is_iterable<T, std::void_t<decltype(std::begin(std::declval<T>())), decltype(std::end(std::declval<T>()))>>
  : std::true_type
{
};

// Here is a helper:
template <typename T>
constexpr bool is_iterable_v = is_iterable<T>::value;

template <typename T, typename = void>
struct is_streamable : std::false_type
{
};

template <typename T>
struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<T>())>>
  // struct is_streamable<T, std::void_t<typename std::is_convertible<
  // decltype(std::declval<std::ostream&>() << std::declval<T>()), std::ostream&>::value>>
  : std::true_type
{
};
}  // namespace utils
}  // namespace prx