#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/utilities/spaces/space.hpp"

using namespace boost::python;

int get_dim_wrapper(prx::space_point_t p)
{
  return p->get_dim();
}
double space_point_get_item(prx::space_point_t& pt, int i)
{
  INDEX_CHECK(i, pt->get_dim(), "space_point")

  return pt->at(i);
}
void space_point_set_item(prx::space_point_t& pt, int i, double val)
{
  INDEX_CHECK(i, pt->get_dim(), "space_point")

  pt->at(i) = val;
}

void space_set_item(prx::space_t& space, int i, double val)
{
  INDEX_CHECK(i, space.get_dimension(), "space_t")

  space.at(i) = val;
}

prx::distance_function_t init_distance_function()
{
  prx::distance_function_t default_df = [](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    return sqrt((s1->at(0) - s2->at(0)) * (s1->at(0) - s2->at(0)) + (s1->at(1) - s2->at(1)) * (s1->at(1) - s2->at(1)));
  };
  return default_df;
}

struct distance_function_wrapper : prx::distance_function_t, wrapper<prx::distance_function_t>
{
};

void set_distance_function(object self, PyObject* f)
{
  prx::distance_function_t df = extract<prx::distance_function_t>(self.attr("distance_function"));
  df = [f](const prx::space_point_t& s1, const prx::space_point_t& s2) { return call<double>(f, s1, s2); };
  // self = df;
}

prx::distance_function_t get_df(PyObject* x)
{
  std::function<double(const prx::space_point_t&, const prx::space_point_t&)> new_df =
      [x](const prx::space_point_t& s1, const prx::space_point_t& s2) { return call<double>(x, s1, s2); };
  return new_df;
}

boost::python::list space_point_to_pylist(const prx::space_point_t& v)
{
  boost::python::list l;
  for (int i = 0; i < v->get_dim(); ++i)
  {
    l.append((*v)[i]);
  }
  return l;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(space_t_print_point_overloads, prx::space_t::print_point, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(space_t_print_memory_overloads, prx::space_t::print_memory, 0, 1)
BOOST_PYTHON_FUNCTION_OVERLOADS(space_t_l1_norm_overloads, prx::space_t::l1_norm, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(space_t_l2_norm_overloads, prx::space_t::l2_norm, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(space_t_euclidean_2d_overloads, prx::space_t::euclidean_2d, 2, 4)

void (prx::space_t::*enforce_bounds0)() const = &prx::space_t::enforce_bounds;
void (prx::space_t::*enforce_bounds1)(const prx::space_point_t&) const = &prx::space_t::enforce_bounds;

void (prx::space_t::*integrate_0)(const prx::space_point_t&, const prx::space_t*, double) = &prx::space_t::integrate;
void (prx::space_t::*integrate_1)(const prx::space_t*, double) = &prx::space_t::integrate;

void (prx::space_t::*copy_std_vector_from_point)(
    std::vector<double>& destination, const prx::space_point_t& source) const = &prx::space_t::copy_vector_from_point;

void copy_eigen_vector_from_point(prx::space_t* s, Eigen::VectorXd destination, const prx::space_point_t& source)
{
  s->copy_vector_from_point(destination, source);
}

struct space_memory_py
{
  space_memory_py(int size) : mem(size, 0)
  {
    for (int i = 0; i < size; ++i)
    {
      mem_ptr.push_back(&(mem[i]));
    }
  }

  const std::vector<double*> get_addresses() const
  {
    return mem_ptr;
  }

private:
  std::vector<double> mem;
  std::vector<double*> mem_ptr;
};

prx::space_t* space_constructor_py(const std::string& topo, const space_memory_py& sm, const std::string& name)
{
  return new prx::space_t(topo, sm.get_addresses(), name);
}

void pyobject_to_vector(std::vector<double>& vec_to, const boost::python::list& py_list_from)
{
  for (int i = 0; i < vec_to.size(); ++i)
  {
    vec_to[i] = boost::python::extract<double>(py_list_from[i]);
  }
}

void vector_to_pyobject(boost::python::list& py_list_to, const std::vector<double>& vec_from)
{
  for (int i = 0; i < vec_from.size(); ++i)
  {
    py_list_to[i] = vec_from[i];
  }
}

template <typename T>
void py_copy_to_T(const prx::space_t* space, const T& pt)
{
  space->copy_to(pt);
}

void py_copy_to(const prx::space_t* space, boost::python::list& py_list)
{
  const std::size_t dim{ space->get_dimension() };
  std::vector<double> vec_aux(dim, 0);

  space->copy_to(vec_aux);
  if (len(py_list) > dim)
  {
    while (len(py_list) > dim)
    {
      py_list.pop();
    }
  }
  else if (len(py_list) < dim)
  {
    while (len(py_list) < dim)
    {
      py_list.append(0);
    }
  }

  vector_to_pyobject(py_list, vec_aux);
}

template <typename T>
void py_copy_from_T(const prx::space_t* space, const T& pt)
{
  space->copy_from(pt);
}

void py_copy_from(const prx::space_t* space, boost::python::list& py_list)
{
  const std::size_t dim{ space->get_dimension() };
  std::vector<double> vec_aux(dim, 0);
  prx_assert(len(py_list) >= dim, "space::copy_from expects list of size space.get_dimension() = " << dim << ".");
  prx_warn_cond(len(py_list) == dim, "Length of list (" << len(py_list)
                                                        << ") to copy from is greater than dimension of space (" << dim
                                                        << "), dropping the greater indexes.");

  pyobject_to_vector(vec_aux, py_list);
  space->copy_from(vec_aux);
}

template <typename To, typename From>
void py_copy_T(const prx::space_t* space, To& to, const From& from)
{
  space->copy(to, from);
}

void py_copy_0(const prx::space_t* space, boost::python::list& py_list_to, const boost::python::list& py_list_from)
{
  const std::size_t dim{ space->get_dimension() };
  std::vector<double> vec_aux_to(dim, 0);
  std::vector<double> vec_aux_from(dim, 0);
  prx_assert(len(py_list_to) == dim,
             "space::copy_from expects list to copy to to be of size space.get_dimension() = " << dim << ".");
  prx_assert(len(py_list_from) == dim,
             "space::copy_from expects list to copy from to be of size space.get_dimension() = " << dim << ".");

  pyobject_to_vector(vec_aux_from, py_list_from);

  space->copy(vec_aux_to, vec_aux_from);

  vector_to_pyobject(py_list_to, vec_aux_to);
}

template <typename To>
void py_copy_1(const prx::space_t* space, To& to, const boost::python::list& py_list_from)
{
  const std::size_t dim{ space->get_dimension() };
  std::vector<double> vec_aux_from(dim, 0);
  prx_assert(len(py_list_from) == dim,
             "space::copy_from expects list to copy from to be of size space.get_dimension() = " << dim << ".");

  pyobject_to_vector(vec_aux_from, py_list_from);

  space->copy(to, vec_aux_from);
}

template <typename From>
void py_copy_2(const prx::space_t* space, boost::python::list& py_list_to, const From& from)
{
  const std::size_t dim{ space->get_dimension() };
  std::vector<double> vec_aux_to(dim, 0);
  prx_assert(len(py_list_to) == dim,
             "space::copy_from expects list to copy to to be of size space.get_dimension() = " << dim << ".");

  space->copy(vec_aux_to, from);

  vector_to_pyobject(py_list_to, vec_aux_to);
}

void pyprx_utilities_spaces_space()
{
  // typedef std::shared_ptr<space_snapshot_t> space_point_t;
  class_<prx::space_point_t>("space_point", no_init)
      .def("__len__", &get_dim_wrapper)
      .def("get_dim", &get_dim_wrapper)
      .def("__getitem__", &space_point_get_item)
      .def("__setitem__", &space_point_set_item)
      // .def("assign", &list_assign<double>)
      .def("__str__", &prx_to_str<prx::space_point_t>)
      .def("__repr__", &prx_print<prx::space_point_t>)
      .def("to_list", &space_point_to_pylist)
      // .def(str(self))
      ;
  enum_<prx::space_t::topology_t>("topology")
      .value("EUCLIDEAN", prx::space_t::topology_t::EUCLIDEAN)
      .value("ROTATIONAL", prx::space_t::topology_t::ROTATIONAL)
      .value("DISCRETE", prx::space_t::topology_t::DISCRETE)
      .export_values();

  class_<prx::distance_function_t>("distance_function")
      .def("__call__", &prx::distance_function_t::operator())
      .def("default", make_function(&init_distance_function, default_call_policies()))
      .staticmethod("default")
      // .def("set_df", &get_df).staticmethod("set_df")
      .def("set_df", &create_function<prx::distance_function_t, double, prx::space_point_t, prx::space_point_t>)
      .staticmethod("set_df")
      .def("wrap", &create_function<prx::distance_function_t, double, prx::space_point_t, prx::space_point_t>)
      .staticmethod("wrap")
      // .def("wrap", &get_df).staticmethod("wrap")
      // .def("__setattr__", &set_distance_function).staticmethod("__setattr__")
      ;

  class_<space_memory_py>("space_memory", init<int>())
      // .def("get_addresses", &space_memory_py::get_addresses)
      ;

  class_<prx::space_t, std::shared_ptr<prx::space_t>>("space_t", init<std::string, std::vector<double*>, std::string>())
      .def("__init__", make_constructor(&init_as_ptr<prx::space_t, std::string, std::vector<double*>&>,
                                        default_call_policies(), (args("topology"), args("addresses"))))
      .def("__init__", make_constructor(&init_as_ptr<prx::space_t, const std::vector<const prx::space_t*>&>,
                                        default_call_policies(), (args("spaces"))))
      .def("__init__", make_constructor(&space_constructor_py, default_call_policies()))
      .def("set_bounds", &prx::space_t::set_bounds)
      .def("make_point", &prx::space_t::make_point)
      .def("clone_point", &prx::space_t::clone_point)
      .def("enforce_bounds", enforce_bounds0)
      .def("enforce_bounds", enforce_bounds1)
      .def("at", &prx::space_t::at, return_value_policy<copy_non_const_reference>())
      .def("__getitem__", &prx::space_t::at, return_value_policy<copy_non_const_reference>())
      .def("__setitem__", &space_set_item)
      .def("get_dimension", &prx::space_t::get_dimension)
      .def("copy_to", py_copy_to)
      .def("copy_to", py_copy_to_T<prx::space_point_t>)
      .def("copy_to_point", py_copy_to_T<prx::space_point_t>)
      .def("copy_from", py_copy_from)
      .def("copy_from", py_copy_from_T<prx::space_point_t>)
      .def("copy_from_point", py_copy_from_T<prx::space_point_t>)
      .def("copy_from_vector", py_copy_from_T<std::vector<double>>)
      .def("copy", py_copy_0)
      .def("copy", py_copy_1<prx::space_point_t>)
      .def("copy", py_copy_2<prx::space_point_t>)
      .def("copy", py_copy_T<prx::space_point_t, prx::space_point_t>)
      .def("copy_point", py_copy_T<prx::space_point_t, prx::space_point_t>)
      .def("copy_point_from_vector", py_copy_T<prx::space_point_t, std::vector<double>>)
      .def("copy_vector_from_point", py_copy_T<std::vector<double>, prx::space_point_t>)
      .def("copy_vector_from_point", py_copy_T<Eigen::VectorXd, prx::space_point_t>)
      .def("is_point_in_space", &prx::space_t::is_point_in_space)
      .def("split_point", &prx::space_t::split_point)
      .def("print_memory", &prx::space_t::print_memory, space_t_print_memory_overloads())
      .def("print_point", &prx::space_t::print_point, space_t_print_point_overloads())
      .def("equal_points", &prx::space_t::equal_points)
      .def("get_dimension", &prx::space_t::get_dimension)
      .def("satisfies_bounds", &prx::space_t::satisfies_bounds)
      .def("sample", &prx::space_t::sample)
      .def("get_space_name", &prx::space_t::get_space_name)
      .def("get_lower_bounds", &prx::space_t::get_lower_bounds)
      .def("get_upper_bounds", &prx::space_t::get_upper_bounds)
      .def("get_lower_bound", &prx::space_t::get_lower_bound)
      .def("get_upper_bound", &prx::space_t::get_upper_bound)
      .def("get_bounds", &prx::space_t::get_bounds)
      .def("integrate", integrate_0)
      .def("integrate", integrate_1)
      .def("interpolate", &prx::space_t::interpolate)
      .def("l1_norm", (double (*)(const prx::space_point_t& p1, const prx::space_point_t& p2))1,
           space_t_l1_norm_overloads())
      .staticmethod("l1_norm")
      .def("l2_norm", (double (*)(const prx::space_point_t& p1, const prx::space_point_t& p2))1,
           space_t_l2_norm_overloads())
      .staticmethod("l2_norm")
      .def("euclidean_2d",
           (double (*)(const prx::space_point_t& p1, const prx::space_point_t& p2, int start, int end))2,
           space_t_euclidean_2d_overloads())
      .staticmethod("euclidean_2d")
      .def("print_bounds", &prx::space_t::print_bounds)
      // .def("lp_norm", lp_norm_2)
      // .def("l1_norm", (double (prx::space_t::*)(const
      // prx::space_point_t&))&prx::space_t::l1_norm).staticmethod("l1_norm") .def("l1_norm", &prx::space_t::l1_norm,
      // space_t_l1_norm_overloads(args("p1", "p2"), "l1 norm")) .def<double (prx::space_t::*)(const prx::space_point_t&
      // p1)>("l1_norm", prx::space_t::l1_norm)//.staticmethod("l1_norm") .def("l1_norm",
      // l1_norm_2).staticmethod("l1_norm") .def("", &prx::space_t::) .def("", &prx::space_t::) .def("",
      // &prx::space_t::) .def("", &prx::space_t::)
      ;
}
