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

}  // namespace utils
}  // namespace prx