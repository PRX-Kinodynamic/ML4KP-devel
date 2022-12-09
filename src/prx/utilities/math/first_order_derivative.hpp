#pragma once
#include <tuple>

#include <Eigen/Core>
#include "prx/utilities/defs.hpp"

namespace prx
{
namespace math
{
// Based on: https://geometrictools.com/Documentation/FiniteDifferences.pdf

// The error of the derivative is: O(h^{s-1}).
using S = uint8_t;
using I_min = int8_t;
using D = int8_t;
using N_i = int8_t;
using approximation_row_t = std::tuple<D, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i>;
struct first_order_derivative_table_t
{
  // using approximation_table_t =
  //     std::map<std::pair<S, I_min>, std::tuple<D, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i>>;
  static constexpr approximation_row_t approximation_table(S s, I_min i_min)
  {
    // clang-format off
  	//		  	     								                     D  -4  -3  -2  -1   0   1   2   3   4
    if (s == 2 && i_min ==  0) return std::make_tuple( 1,  0,  0,  0,  0, -1,  1,  0,  0,  0);
    if (s == 2 && i_min == -1) return std::make_tuple( 1,  0,  0,  0, -1,  1,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 3 && i_min ==  0) return std::make_tuple( 2,  0,  0,  0,  0, -3,  4, -1,  0,  0);
    if (s == 3 && i_min == -1) return std::make_tuple( 2,  0,  0,  0, -1,  0,  1,  0,  0,  0);
    if (s == 3 && i_min == -2) return std::make_tuple( 2,  0,  0,  1, -4,  3,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 4 && i_min ==  0) return std::make_tuple( 6,  0,  0,  0,  0,-11, 18, -9,  2,  0);
    if (s == 4 && i_min == -1) return std::make_tuple( 6,  0,  0,  0, -2, -3,  6, -1,  0,  0);
    if (s == 4 && i_min == -2) return std::make_tuple( 6,  0,  0,  1, -6,  3,  2,  0,  0,  0);
    if (s == 4 && i_min == -3) return std::make_tuple( 6,  0, -2,  9,-18, 11,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 5 && i_min ==  0) return std::make_tuple(12,  0,  0,  0,  0,-25, 48,-36, 16, -3);
    if (s == 5 && i_min == -1) return std::make_tuple(12,  0,  0,  0, -3,-10, 18, -6,  1,  0);
    if (s == 5 && i_min == -2) return std::make_tuple(12,  0,  0,  1, -8,  0,  8, -1,  0,  0);
    if (s == 5 && i_min == -3) return std::make_tuple(12,  0, -1,  6,-18, 10,  3,  0,  0,  0);
    if (s == 5 && i_min == -4) return std::make_tuple(12,  3,-16, 36,-48, 25,  0,  0,  0,  0);
    return std::make_tuple(0,  0,  0,  0,  0, 0,  0,  0,  0,  0); // <== Should never get here
  }
  // clang-format on
};  // namespace math

template <class Function, typename InputState, S Evaluations, I_min MinDifference = 0>
class first_order_derivative_t
{
  using Scalar = typename InputState::Scalar;
  using OutputState = typename std::invoke_result<Function, InputState>::type;

  static constexpr Eigen::Index NInputs{ InputState::RowsAtCompileTime };
  static constexpr Eigen::Index NOutputs{ OutputState::RowsAtCompileTime };

  using output_matrix_t = Eigen::Matrix<Scalar, NOutputs, NInputs>;
  using epsilon_matrix_t = Eigen::Matrix<Scalar, NInputs, NInputs>;

public:
  first_order_derivative_t() = delete;

  first_order_derivative_t(const double _h = 0.01) : h(_h), epsilon_matrix(_h * epsilon_matrix_t::Identity())
  {
  }

  first_order_derivative_t(Function& _model, const double _h = 0.01)
    : model(_model), h(_h), epsilon_matrix(_h * epsilon_matrix_t::Identity())
  {
  }

  output_matrix_t operator()(const InputState& state) const
  {
    const D d{ std::get<0>(approximation_row) };
    const N_i n1{ std::get<1>(approximation_row) };
    const N_i n2{ std::get<2>(approximation_row) };
    const N_i n3{ std::get<3>(approximation_row) };
    const N_i n4{ std::get<4>(approximation_row) };
    const N_i n5{ std::get<5>(approximation_row) };
    const N_i n6{ std::get<6>(approximation_row) };
    const N_i n7{ std::get<7>(approximation_row) };
    const N_i n8{ std::get<8>(approximation_row) };
    const N_i n9{ std::get<9>(approximation_row) };
    static_assert(d != 0, "Invalid approximation_table for first_order_derivative_t");

    output_matrix_t derivative{};
    for (int i = 0; i < NInputs; ++i)
    {
      const OutputState F1 = evaluate<n1, 1 - 5>(state, i, OutputState::Zero());
      const OutputState F2 = evaluate<n2, 2 - 5>(state, i, F1);
      const OutputState F3 = evaluate<n3, 3 - 5>(state, i, F2);
      const OutputState F4 = evaluate<n4, 4 - 5>(state, i, F3);
      const OutputState F5 = evaluate<n5, 5 - 5>(state, i, F4);
      const OutputState F6 = evaluate<n6, 6 - 5>(state, i, F5);
      const OutputState F7 = evaluate<n7, 7 - 5>(state, i, F6);
      const OutputState F8 = evaluate<n8, 8 - 5>(state, i, F7);
      const OutputState F9 = evaluate<n9, 9 - 5>(state, i, F8);

      derivative.col(i) = F9 / (d * h);
    }
    return derivative;
  }

  Function model;

private:
  static constexpr approximation_row_t approximation_row{ first_order_derivative_table_t::approximation_table(
      Evaluations, MinDifference) };

  template <N_i n_i, int hi, std::enable_if_t<(n_i != 0), bool> = true>
  inline OutputState evaluate(const InputState& input, const int col_i, const OutputState& AccumSum) const
  {
    return AccumSum + n_i * model(input + hi * epsilon_matrix.col(col_i));
  }
  template <N_i n_i, int hi, std::enable_if_t<(n_i == 0), bool> = true>
  inline OutputState evaluate(const InputState& input, const int col_i, const OutputState& AccumSum) const
  {
    return AccumSum;
  }

  double h;
  epsilon_matrix_t epsilon_matrix;
};
template <class Function, typename InputState, S Evaluations, I_min MinDifference>
constexpr approximation_row_t
    first_order_derivative_t<Function, InputState, Evaluations, MinDifference>::approximation_row;
}  // namespace math
}  // namespace prx
