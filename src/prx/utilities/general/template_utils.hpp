#pragma once
#include <memory>
#include <type_traits>

namespace prx
{
namespace utils
{

// is_shared_ptr: Taken from:
// https://stackoverflow.com/questions/41853159/how-to-detect-if-a-type-is-shared-ptr-at-compile-time
// This is though of an extension of std::is_pointer<T> to detect shared_ptr(s).
template <class T>
struct is_shared_ptr : std::false_type
{
};

template <class T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
{
};

}  // namespace utils
}  // namespace prx