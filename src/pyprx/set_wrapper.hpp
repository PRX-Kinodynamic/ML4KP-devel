#pragma once
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

namespace pyprx
{
namespace stdlib
{
// Unordered set: https://en.cppreference.com/w/cpp/container/unordered_set

template <typename Set>
struct set_wrapper
{
  using value_type = typename Set::value_type;
  using key_type = typename Set::key_type;
  using index_type = typename Set::key_type;
  using size_type = typename Set::size_type;
  using difference_type = typename Set::difference_type;
  using iterator = typename Set::iterator;

  // function_N according to https://en.cppreference.com/w/cpp/container/unordered_set/function numbering
  static std::pair<iterator, bool> insert_1(Set& set, const value_type& value)
  {
    return set.emplace(value);
  };

  static iterator erase_1(Set& set, iterator pos)
  {
    return set.erase(pos);
  }

  static size_type erase_4(Set& set, const key_type& key)
  {
    return set.erase(key);
  }

  // static void merge_1(Set& set, Set& source) //Only available in c++17
  // {
  //   set.merge(source);
  // }

  static size_type count_1(Set& set, const key_type& key)
  {
    return set.count(key);
  }

  static iterator find_1(Set& set, const key_type& key)
  {
    return set.find(key);
  }

  static bool contains_1(Set& set, const key_type& key)
  {
    return set.count(key) > 0;  // To avoid dependency on c++20
  }

  static void bindings(const std::string name)
  {
    class_<Set>(name.c_str(), init<>())
        .def("__str__", &container_to_string<Set>)
        .def("empty", &Set::empty)
        .def("size", &Set::size)
        .def("clear", &Set::clear)
        .def("insert", &insert_1)
        .def("erase", &erase_1)
        .def("erase", &erase_4)
        // .def("merge", &merge_1)
        .def("count", &count_1)
        .def("find", &find_1)
        .def("contains", &contains_1)
        .def("__iter__", boost::python::iterator<Set>())
        // Comment to force ; to the next one
        ;
  }
};
}  // namespace stdlib
}  // namespace pyprx