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
using N_i = int16_t;
using second_order_approximation_row_t = std::tuple<D, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i, N_i>;
struct second_order_derivative_table_t
{
  static constexpr second_order_approximation_row_t approximation_table(S s, I_min i_min)
  {
    // clang-format off
    //                                                 D  -5  -4   -3  -2   -1    0   1   2   3   4   5
    if (s == 3 && i_min ==  0) return std::make_tuple( 1,  0,  0,   0,  0,   0,   1, -2,   1,   0,  0,  0);
    if (s == 3 && i_min == -1) return std::make_tuple( 1,  0,  0,   0,  0,   1,  -2,  1,   0,   0,  0,  0);
    if (s == 3 && i_min == -2) return std::make_tuple( 1,  0,  0,   0,  1,  -2,   1,  0,   0,   0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 4 && i_min ==  0) return std::make_tuple( 1,  0,  0,   0,  0,   0,   2,  -5,  4,  -1,  0,  0);
    if (s == 4 && i_min == -1) return std::make_tuple( 1,  0,  0,   0,  0,   1,  -2,   1,  0,   0,  0,  0);
    if (s == 4 && i_min == -2) return std::make_tuple( 1,  0,  0,   0,  0,   1,  -2,   1,  0,   0,  0,  0);
    if (s == 4 && i_min == -3) return std::make_tuple( 1,  0,  0,  -1,  4,  -5,   2,   0,  0,   0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 5 && i_min ==  0) return std::make_tuple(12,  0,  0,   0,  0,   0,  35,-104,114, -56, 11,  0);
    if (s == 5 && i_min == -1) return std::make_tuple(12,  0,  0,   0,  0,  11, -20,   6,  4,  -1,  0,  0);
    if (s == 5 && i_min == -2) return std::make_tuple(12,  0,  0,   0, -1,  16, -30,  16, -1,   0,  0,  0);
    if (s == 5 && i_min == -3) return std::make_tuple(12,  0,  0,  -1,  4,   6, -20,  11,  0,   0,  0,  0);
    if (s == 5 && i_min == -4) return std::make_tuple(12,  0, 11, -56,114,-104,  35,   0,  0,   0,  0,  0);
    // --------------------------------------------------------------------------------------
    if (s == 6 && i_min ==  0) return std::make_tuple(12,  0,  0,   0,  0,   0,  45,-154,214,-156, 61,-10);
    if (s == 6 && i_min == -1) return std::make_tuple(12,  0,  0,   0,  0,  10, -15,  -4, 14,  -6,  1,  0);
    if (s == 6 && i_min == -2) return std::make_tuple(0,  0,  0,   0, -1,  16, -30,  16,  1,   0,  0,  0); // Not working
    if (s == 6 && i_min == -3) return std::make_tuple(0,  0,  0,   0, -1,  16, -30,  16,  1,   0,  0,  0); // Not working
    if (s == 6 && i_min == -4) return std::make_tuple(12,  0,  1,  -6, 14,  -4, -15,  10,  0,   0,  0,  0);
    if (s == 6 && i_min == -5) return std::make_tuple(12,-10, 61,-156,214,-154,  45,   0,  0,   0,  0,  0);
    return std::make_tuple(0,  0,  0,  0,  0, 0,  0,  0,  0,  0, 0, 0); // <== Error case
  }
  // clang-format on
};

template <class Function, typename InputState, S Evaluations, I_min MinDifference>
class second_order_derivative_t
{
  using Scalar = typename InputState::Scalar;
  using OutputState = typename std::invoke_result<Function, InputState>::type;

  static constexpr Eigen::Index NInputs{ InputState::RowsAtCompileTime };
  static constexpr Eigen::Index NOutputs{ OutputState::RowsAtCompileTime };

  using output_matrix_t = Eigen::Matrix<Scalar, NOutputs, NInputs>;
  using epsilon_matrix_t = Eigen::Matrix<Scalar, NInputs, NInputs>;

public:
  second_order_derivative_t(const double h, const Eigen::Index n_inputs, const Eigen::Index n_outputs)
    : _h(h)
    , _n_inputs(n_inputs)
    , _n_outputs(n_outputs)
    , _epsilon_matrix(_h * epsilon_matrix_t::Identity(_n_inputs, _n_inputs))
    , _zero_matrix(output_matrix_t::Zero(_n_outputs, _n_inputs))
    , _dh(_d * _h * _h)
  {
    static_assert(_d != 0, "Invalid approximation_table for second_order_derivative_t");
  }

  template <Eigen::Index InputDim = NInputs, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  second_order_derivative_t(const double h = 0.01) : second_order_derivative_t(h, NInputs, NOutputs)
  {
  }

  second_order_derivative_t(Function& model, const double h, const Eigen::Index n_inputs, const Eigen::Index n_outputs)
    : second_order_derivative_t(h, n_inputs, n_outputs)
  {
    _model = model;
  }

  template <Eigen::Index InputDim = NInputs, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  second_order_derivative_t(Function& model, const double h = 0.01)
    : second_order_derivative_t(model, h, NInputs, NOutputs)
  {
  }

  output_matrix_t operator()(const InputState& state) const
  {
    output_matrix_t derivative{ _zero_matrix };

    iterate_columns<0>(state, derivative);

    return derivative;
  }

  Function _model;

private:
  static constexpr second_order_approximation_row_t approximation_row{
    second_order_derivative_table_t::approximation_table(Evaluations, MinDifference)
  };

  template <N_i n_i, int i, std::enable_if_t<(n_i != 0), bool> = true>
  inline void evaluate(const InputState& input, const int col_i, output_matrix_t& derivative) const
  {
    derivative.col(col_i) += n_i * _model(input + i * _epsilon_matrix.col(col_i));
  }
  template <N_i n_i, int i, std::enable_if_t<(n_i == 0), bool> = true>
  inline void evaluate(const InputState& input, const int col_i, output_matrix_t& derivative) const
  {
  }

  template <Eigen::Index I, std::enable_if_t<(NInputs != Eigen::Dynamic) && (I == NInputs), bool> = true>
  inline void iterate_columns(const InputState& state, output_matrix_t& derivative) const
  {
  }
  // If the vector size is known at compile time, avoid a for and rely on compiler/templates optimization
  template <Eigen::Index I, std::enable_if_t<(NInputs != Eigen::Dynamic) && (I < NInputs), bool> = true>
  inline void iterate_columns(const InputState& state, output_matrix_t& derivative) const
  {
    compute_column(I, state, derivative);
    iterate_columns<I + 1>(state, derivative);
  }

  // When the vector size is not known at compilation time. Compiler won't be able to optimize the for (unwrap)
  template <Eigen::Index I, std::enable_if_t<(NInputs == Eigen::Dynamic) && (I >= 0), bool> = true>
  void iterate_columns(const InputState& state, output_matrix_t& derivative) const
  {
    for (std::size_t i = 0; i < _n_inputs; ++i)
    {
      compute_column(i, state, derivative);
    }
  }

  inline void compute_column(const std::size_t col_i, const InputState& state, output_matrix_t& derivative) const
  {
    // OutputState Fi{ OutputState::Zero(_n_outputs) };
    // auto& Fi =
    evaluate<std::get<1>(approximation_row), 1 - 6>(state, col_i, derivative);
    evaluate<std::get<2>(approximation_row), 2 - 6>(state, col_i, derivative);
    evaluate<std::get<3>(approximation_row), 3 - 6>(state, col_i, derivative);
    evaluate<std::get<4>(approximation_row), 4 - 6>(state, col_i, derivative);
    evaluate<std::get<5>(approximation_row), 5 - 6>(state, col_i, derivative);
    evaluate<std::get<6>(approximation_row), 6 - 6>(state, col_i, derivative);
    evaluate<std::get<7>(approximation_row), 7 - 6>(state, col_i, derivative);
    evaluate<std::get<8>(approximation_row), 8 - 6>(state, col_i, derivative);
    evaluate<std::get<9>(approximation_row), 9 - 6>(state, col_i, derivative);
    evaluate<std::get<10>(approximation_row), 10 - 6>(state, col_i, derivative);
    evaluate<std::get<11>(approximation_row), 11 - 6>(state, col_i, derivative);

    derivative.col(col_i) /= _dh;
  }

  static constexpr D _d{ std::get<0>(approximation_row) };
  const double _h;
  const double _dh;

  const Eigen::Index _n_inputs;
  const Eigen::Index _n_outputs;

  const output_matrix_t _zero_matrix;
  const epsilon_matrix_t _epsilon_matrix;
};
template <class Function, typename InputState, S Evaluations, I_min MinDifference>
constexpr second_order_approximation_row_t
    second_order_derivative_t<Function, InputState, Evaluations, MinDifference>::approximation_row;
}  // namespace math
}  // namespace prx
