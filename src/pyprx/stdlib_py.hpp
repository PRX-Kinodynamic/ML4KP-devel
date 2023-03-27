#include <iostream>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
// #include "prx/utilities/geometry/movable_object.hpp"

using namespace boost::python;

namespace pyprx
{
namespace stdlib
{

template <typename T>
std::vector<T> to_std_vector(boost::python::list& ns)
{
  std::vector<T> v;
  for (int i = 0; i < len(ns); ++i)
  {
    v.push_back(boost::python::extract<double>(ns[i]));
  }
  return v;
}

void bindings()
{
  // class_<std::vector<double> >("std_vectorXd")
  //        .def(vector_indexing_suite<std::vector<double> >())
  //        .def(init<std::vector<double>>())
  //        .def("__len__", &std::vector<double>::size)
  //        // .def("__getitem__", &std_item<std::vector<double>::at, return_value_policy<copy_non_const_reference>())
  //        // .def("__setitem__", &std_item<std::vector<double>::at, with_custodian_and_ward<1,2>())
  //        // .def("assign", &list_assign<double>)
  //        .def("__str__", &to_str<double>)
  //        .def("__repr__", &to_str<double>)
  //        // .def(str(self))
  //        ;
  // PRX_ITERABLE_WRAPPER(std::vector<std::string>, "vector_of_strings")
  // PRX_ITERABLE_WRAPPER(std::vector<double>, "vector_of_doubles")
  container_wrapper<std::vector<std::string>>("vector_of_strings");
  container_wrapper<std::vector<double>>("vector_of_doubles");
  container_wrapper<std::vector<std::vector<double>>>("vector_of_vector_of_doubles");
  container_wrapper<std::vector<std::size_t>>("vector_of_unsigned");
  // PRX_ITERABLE_WRAPPER_NONSTR(std::vector<std::vector<double>>, "vector_of_vector_of_doubles")
  // PRX_ITERABLE_WRAPPER_NONSTR(std::vector<double*>, "vector_of_doubles_ptrs")

  // PRX_ITERABLE_WRAPPER(std::vector<double>&, "vector_of_doubles")
  // class_<std::vector<std::string>>("vector_of_strings")
  // .def(vector_indexing_suite<std::vector<std::string>>())
  // ;

  iterable_converter()
      // Build-in type.
      .from_python<std::vector<long unsigned>>()
      .from_python<std::vector<int>>()
      .from_python<std::vector<double>>()
      .from_python<std::vector<double*>>()
      // .from_python<std::vector<double> >()
      // Each dimension needs to be convertable.
      .from_python<std::vector<std::string>>()
      .from_python<std::vector<std::vector<double>>>()
      // .from_python<std::vector<std::shared_ptr<prx::movable_object_t>>>()

      ;
}

}  // namespace stdlib
}  // namespace pyprx