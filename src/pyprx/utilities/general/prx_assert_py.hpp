#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/general/constants.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace general
{
namespace prx_assert
{

void pyprx_assert(const bool expression, const std::string& message)
{
  if (!(expression))
  {
    throw prx::prx_assert_t("[PYPRX] Assertion failed: " + message);
  }
}

void bindings()
{
  def("assert", pyprx_assert);
}
}  // namespace prx_assert
}  // namespace general
}  // namespace utilities
}  // namespace pyprx