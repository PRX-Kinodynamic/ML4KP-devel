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

template <typename MatrixType, typename CheckType>
gtsam::OptionalJacobian<MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime>
init_optional_jacobian(MatrixType& matrix, const CheckType& check)
{
  const Eigen::Index DimOut{ MatrixType::RowsAtCompileTime };
  const Eigen::Index DimIn{ MatrixType::ColsAtCompileTime };
  if (check)
  {
    return gtsam::OptionalJacobian<DimOut, DimIn>(matrix);
  }
  return gtsam::OptionalJacobian<DimOut, DimIn>(boost::none);
}

template <typename MatrixType, typename CheckType, typename... CheckTypes>
gtsam::OptionalJacobian<MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime>
init_optional_jacobian(MatrixType& matrix, const CheckType& check, CheckTypes... checks)
{
  if (check)
    return init_optional_jacobian(matrix, checks...);
  return boost::none;
}

}  // namespace fg
}  // namespace prx