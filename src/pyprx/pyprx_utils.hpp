#pragma once
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "prx/utilities/general/template_utils.hpp"
// Lets put python translating functions globally accesible here
//
//
//
using namespace boost::python;

#define PRX_GETTER(obj, var)                                                                                           \
  template <class T>                                                                                                   \
  T get_##obj##_##var(obj& o)                                                                                          \
  {                                                                                                                    \
    return o.var;                                                                                                      \
  }

#define PRX_SETTER(obj, var)                                                                                           \
  template <class T>                                                                                                   \
  void set_##obj##_##var(obj& o, T val)                                                                                \
  {                                                                                                                    \
    o.var = val;                                                                                                       \
  }
#define PRX_PTR_GETTER(obj, var)                                                                                       \
  template <class T>                                                                                                   \
  T get_ptr_##obj##_##var(obj& o)                                                                                      \
  {                                                                                                                    \
    return o->var;                                                                                                     \
  }
#define PRX_GETTER_TEMPLATE(obj, var)                                                                                  \
  template <class T, typename... Targs>                                                                                \
  T get_##obj##_##var(obj<Targs...>& o)                                                                                \
  {                                                                                                                    \
    return o.var;                                                                                                      \
  }

#define PRX_SETTER_TEMPLATE(obj, var)                                                                                  \
  template <class T, typename... Targs>                                                                                \
  void set_##obj##_##var(obj<Targs...>& o, T val)                                                                      \
  {                                                                                                                    \
    o.var = val;                                                                                                       \
  }

#define PRX_PTR_GETTER_TEMPLATE(obj, var)                                                                              \
  template <class T, typename... Targs>                                                                                \
  T get_ptr_##obj##_##var(obj<Targs...>& o)                                                                            \
  {                                                                                                                    \
    return o->var;                                                                                                     \
  }
/*
 * Safety checking functions
 *
 */
#define INDEX_CHECK(i, MAX, MSG)                                                                                       \
  if (i < 0 || i >= MAX)                                                                                               \
  {                                                                                                                    \
    PyErr_SetString(PyExc_IndexError, ("[ PRX::" + std::string(MSG) + " ] Index '" + std::to_string(i) +               \
                                       "' Out Of Range: [0, " + std::to_string(MAX) + ")")                             \
                                          .c_str());                                                                   \
    throw_error_already_set();                                                                                         \
  }

/*
 * Function wrappers to interface python with raw/smart ptrs
 */
#define PRX_FUNC_WRAPPER(OBJ, PTR, FUNC)                                                                               \
  void OBJ##_##FUNC##_wrapper(prx::OBJ& o, std::shared_ptr<prx::PTR> smart_p)                                          \
  {                                                                                                                    \
    o.FUNC(smart_p.get());                                                                                             \
  }

#define PRX_ITERABLE_WRAPPER(CLASS, NAME)                                                                              \
  class_<CLASS>(NAME).def(vector_indexing_suite<CLASS>()).def("__str__", &iter_to_str<CLASS>);

#define PRX_ITERABLE_WRAPPER_NONSTR(CLASS, NAME) class_<CLASS>(NAME).def(vector_indexing_suite<CLASS>());

#define SINGLE_ARG(...) __VA_ARGS__

// TODO: put everything inside these namespaces
namespace pyprx
{
template <typename PtrType>
static void register_ptr_with_check()
{
  boost::python::type_info info = boost::python::type_id<PtrType>();
  const boost::python::converter::registration* reg = boost::python::converter::registry::query(info);
  if (reg == NULL)
  {
    register_ptr_to_python<PtrType>();
  }
  else if ((*reg).m_to_python == NULL)
  {
    register_ptr_to_python<PtrType>();
  }
}

template <std::size_t Counter, typename ContainerElement,
          std::enable_if_t<!prx::utilities::is_iterable<ContainerElement>{}, bool> = true>
inline std::string container_to_string_1(const ContainerElement& container_element, std::string& separator)
{
  std::ostringstream os;
  os << container_element;
  separator = " ";
  return os.str();
}

template <std::size_t Counter, typename Container,
          std::enable_if_t<prx::utilities::is_iterable<Container>{}, bool> = true>
std::string container_to_string_1(const Container& container, std::string& separator)
{
  std::ostringstream os;
  for (auto& element : container)
  {
    os << container_to_string_1<Counter + 1>(element, separator) << separator;
  }
  std::string result{ os.str() };
  result.erase(result.size() - 1, 1);
  separator = "\n";
  return result;
}
template <typename Container, std::enable_if_t<prx::utilities::is_iterable<Container>{}, bool> = true>
std::string container_to_string(const Container& container)
{
  std::string separator = " ";
  return container_to_string_1<0>(container, separator);
}

template <typename T>
std::string iter_to_str(T& v)
{
  // using namespace std;
  std::ostringstream os;
  // copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
  os << "[";
  for (int i = 0; i < v.size(); ++i)
  {
    if (i != 0)
      os << ", ";
    os << v[i];
  }
  os << "]";
  return os.str();
}
template <typename Vector>
void vector_wrapper(const std::string name)
{
  class_<Vector>(name.c_str(), init<>())
      .def(vector_indexing_suite<Vector>())
      .def("__str__", &container_to_string<Vector>)
      // Comment to force ; to the next one
      ;
}

template <typename T>
std::string stream_to_str(T& obj)
{
  std::stringstream ss;
  ss << obj;
  return ss.str();
}

// Haven't been able to bind operator[] directly, so this makes it almost the same
template <typename T, typename R>
R& wrapper_subscript_oper_to_get_item(T& obj, const std::size_t& idx)
{
  return obj[idx];
}

template <typename T, typename R>
void wrapper_subscript_oper_to_set_item(T& obj, const std::size_t& idx, R data)
{
  obj[idx] = data;
}
template <typename T>
void pylist_to_vector(std::vector<T>& vec_to, const boost::python::list& py_list_from)
{
  for (int i = 0; i < boost::python::len(py_list_from); ++i)
  {
    vec_to.push_back(boost::python::extract<T>(py_list_from[i]));
  }
}
template <typename Container>
void container_to_pyobject(boost::python::list& py_list_to, const Container& from)
{
  for (int i = 0; i < from.size(); ++i)
  {
    py_list_to[i] = from[i];
  }
}

template <typename Owner, typename... Args, void (Owner::*F)(Args...)>
void overload_member_function(Owner* owner, Args... args){
  // F(args...);
};
/*
 * Iterating functions
 */
template <typename T>
std::string to_str(const std::vector<T>& v)
{
  // using namespace std;
  std::ostringstream os;
  copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
  return os.str();
}

/*
 * Template functions to create smart pointers
 */
template <class T, class R>
std::shared_ptr<R> create_ptr(T* obj)
{
  return std::shared_ptr<R>(obj);
  // std::shared_ptr<R> new_ptr;
  // new_ptr.reset(&new_obstacle);
  // return new_ptr;
}

template <class T>
std::shared_ptr<T> create_system_ptr(std::string name)
{
  return std::make_shared<T>(name);
}

template <class T, typename... Targs>
std::shared_ptr<T> init_as_ptr(Targs... Fargs)
{
  return std::make_shared<T>(Fargs...);
}

/*
 * High order functions!
 * Lipsy functions: take functions, create functions and return functions
 *
 * T - Type of the function to create
 * R - What the function will return when called (could be void)
 * Targs - The arguments that the function takes
 */
template <typename T, typename R, typename... Targs>
T create_function(PyObject* x)
{
  T new_f = [x](Targs... Fargs) { return call<R>(x, Fargs...); };
  return new_f;
}

/*
 * String-related functions
 */
template <typename T>
std::string prx_to_str(T obj)
{
  std::ostringstream iss;
  iss << obj;
  return iss.str();
}

template <typename T>
void prx_print(T obj)
{
  std::cout << obj;
}

template <class T>
struct is_pyobject : std::false_type
{
};
template <>
struct is_pyobject<boost::python::list> : std::true_type
{
};
/// @brief Type that allows for registration of conversions from
///        python iterable types.
struct iterable_converter
{
  /// @note Registers converter from a python interable type to the
  ///       provided type.
  template <typename Container>
  iterable_converter& from_python()
  {
    boost::python::converter::registry::push_back(&iterable_converter::convertible,
                                                  &iterable_converter::construct<Container>,
                                                  boost::python::type_id<Container>());

    // Support chaining.
    return *this;
  }

  /// @brief Check if PyObject is iterable.
  static void* convertible(PyObject* object)
  {
    return PyObject_GetIter(object) ? object : NULL;
  }

  /// @brief Convert iterable PyObject to C++ container type.
  ///
  /// Container Concept requirements:
  ///
  ///   * Container::value_type is CopyConstructable.
  ///   * Container can be constructed and populated with two iterators.
  ///     I.e. Container(begin, end)
  template <typename Container>
  static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    namespace python = boost::python;
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting.
    python::handle<> handle(python::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    typedef python::converter::rvalue_from_python_storage<Container> storage_type;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    typedef python::stl_input_iterator<typename Container::value_type> iterator;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor.
    new (storage) Container(iterator(python::object(handle)),  // begin
                            iterator());                       // end
    data->convertible = storage;
  }
};

template <class T>
boost::python::list returning_a_pylist(const T& v)
{
  boost::python::object get_iter = boost::python::iterator<T>();
  boost::python::object iter = get_iter(v);
  boost::python::list l(iter);
  return l;
}
}  // namespace pyprx
