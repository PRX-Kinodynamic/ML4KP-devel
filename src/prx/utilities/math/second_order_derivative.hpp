#pragma once
#include <tuple>

#include <Eigen/Core>
#include "prx/utilities/defs.hpp"

namespace prx
{
namespace math
{
// Based on: https://geometrictools.com/Documentation/FiniteDifferences.pdf
using S = uint8_t;
using I_min = int8_t;
using D = int8_t;
using N_i = int8_t;
using second_order_approximation_row_t = std::tuple<D, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i>;
struct second_order_derivative_table_t
{
  static constexpr second_order_approximation_row_t approximation_table(S s, I_min i_min)
  {
    // clang-format off
    //                                                 D  -5  -4   -3  -2  -1   0   1   2   3   4   5
    if (s == 3 && i_min ==  0) return std::make_tuple( 1,  0,  0,   0,  0,  0,  1, -2,  1,  0,  0,  0);
    if (s == 3 && i_min == -1) return std::make_tuple( 1,  0,  0,   0,  0,  1, -2,  1,  0,  0,  0,  0);
    if (s == 3 && i_min == -2) return std::make_tuple( 1,  0,  0,   0,  1, -2,  1,  0,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 4 && i_min ==  0) return std::make_tuple( 1,  0,  0,   0,  0,  0,  2,  -5,  4, -1,  0,  0);
    if (s == 4 && i_min == -1) return std::make_tuple( 1,  0,  0,   0,  0,  1, -2,   1,  0,  0,  0,  0);
    if (s == 4 && i_min == -2) return std::make_tuple( 1,  0,  0,   0,  0,  1, -2,   1,  0,  0,  0,  0);
    if (s == 4 && i_min == -3) return std::make_tuple( 1,  0,  0,  -1,  4, -5,  2,   0,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 5 && i_min ==  0) return std::make_tuple(12,  0,  0,   0,  0,  0, 35,-104,114,-56,11,  0);
    if (s == 5 && i_min == -1) return std::make_tuple(12,  0,  0,   0,  0, 11,-20,   6,  4, -1,  0,  0);
    if (s == 5 && i_min == -2) return std::make_tuple(12,  0,  0,   0, -1, 16,-30,  16, -1,  0,  0,  0);
    if (s == 5 && i_min == -3) return std::make_tuple(12,  0,  0,  -1,  4,  6,-20,  11,  0,  0,  0,  0);
    if (s == 5 && i_min == -4) return std::make_tuple(12,  0, 11, -56,114,-104,35,   0,  0,  0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 6 && i_min ==  0) return std::make_tuple(12,  0,  0,   0,  0,  0, 45,-154,214,-156,61,-10);
    if (s == 6 && i_min == -1) return std::make_tuple(12,  0,  0,   0,  0, 10,-15,  -4, 14, -6,  1,  0);
    if (s == 6 && i_min == -2) return std::make_tuple(12,  0,  0,   0, -1, 16,-30,  16,  1,  0,  0,  0);
    if (s == 6 && i_min == -3) return std::make_tuple(12,  0,  0,   0, -1, 16,-30,  16,  1,  0,  0,  0);
    if (s == 6 && i_min == -4) return std::make_tuple(12,  0,  1,  -6, 14, -4,-15,  10,  0,  0,  0,  0);
    if (s == 6 && i_min == -5) return std::make_tuple(12,-10, 61,-156,214,-154, 45,  0,  0,  0,  0,  0);
    return std::make_tuple(0,  0,  0,  0,  0, 0,  0,  0,  0,  0, 0, 0); // <== Error case
  }
  // clang-format on
};

template <class Function, typename InputState, S Evaluations, I_min MinDifference>
class second_order_derivative_t
{
  using Scalar = typename InputState::Scalar;
  using OutputState = typename std::result_of<Function(InputState)>::type;

  static constexpr Eigen::Index NInputs{ InputState::RowsAtCompileTime };
  static constexpr Eigen::Index NOutputs{ OutputState::RowsAtCompileTime };

  using output_matrix_t = Eigen::Matrix<Scalar, NOutputs, NInputs>;
  using epsilon_matrix_t = Eigen::Matrix<Scalar, NInputs, NInputs>;

public:
  second_order_derivative_t() = delete;
  second_order_derivative_t(const Function& _model, const double _h = 0.01)
    : model(_model), h(_h), epsilon_matrix(_h * epsilon_matrix_t::Identity())
  {
  }
  output_matrix_t operator()(const InputState& state)
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
    const N_i n10{ std::get<10>(approximation_row) };
    const N_i n11{ std::get<11>(approximation_row) };
    static_assert(d != 0, "Invalid approximation_table for second_order_derivative_t");

    output_matrix_t derivative{};
    for (int i = 0; i < NInputs; ++i)
    {
      const InputState F1 = evaluate<n1, 1 - 6>(state, i, InputState::Zero());
      const InputState F2 = evaluate<n2, 2 - 6>(state, i, F1);
      const InputState F3 = evaluate<n3, 3 - 6>(state, i, F2);
      const InputState F4 = evaluate<n4, 4 - 6>(state, i, F3);
      const InputState F5 = evaluate<n5, 5 - 6>(state, i, F4);
      const InputState F6 = evaluate<n6, 6 - 6>(state, i, F5);
      const InputState F7 = evaluate<n7, 7 - 6>(state, i, F6);
      const InputState F8 = evaluate<n8, 8 - 6>(state, i, F7);
      const InputState F9 = evaluate<n9, 9 - 6>(state, i, F8);
      const InputState F10 = evaluate<n10, 10 - 6>(state, i, F9);
      const InputState F11 = evaluate<n11, 11 - 6>(state, i, F10);

      derivative.col(i) = F11 / (d * h * h);
    }
    return derivative;
  }

private:
  static constexpr second_order_approximation_row_t approximation_row{
    second_order_derivative_table_t::approximation_table(Evaluations, MinDifference)
  };

  template <N_i n_i, int i, std::enable_if_t<(n_i != 0), bool> = true>
  inline InputState evaluate(InputState input, int h_i, const InputState& AccumSum)
  {
    return AccumSum + n_i * model(input + i * epsilon_matrix.col(h_i));
  }
  template <N_i n_i, int i, std::enable_if_t<(n_i == 0), bool> = true>
  inline InputState evaluate(InputState input, int h_i, const InputState& AccumSum)
  {
    return AccumSum;
  }

  Function model;
  double h;
  epsilon_matrix_t epsilon_matrix;
};
template <class Function, typename InputState, S Evaluations, I_min MinDifference>
constexpr second_order_approximation_row_t
    second_order_derivative_t<Function, InputState, Evaluations, MinDifference>::approximation_row;
}  // namespace math
}  // namespace prx
