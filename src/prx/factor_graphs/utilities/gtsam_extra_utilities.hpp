#pragma once
#include <fstream>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{

template <typename Type, typename CheckType>
boost::optional<Type&> check_optional(Type& matrix, const CheckType& check)
{
  if (check)
    return matrix;
  return boost::none;
}

template <typename Type, typename CheckType, typename... CheckTypes>
boost::optional<Type&> check_optional(Type& matrix, const CheckType& check, CheckTypes... checks)
{
  if (check)
    return check_optional(matrix, checks...);
  return boost::none;
}

}  // namespace fg
}  // namespace prx