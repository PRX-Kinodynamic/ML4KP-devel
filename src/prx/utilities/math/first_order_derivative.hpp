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
  static constexpr approximation_row_t approximation_table(S s, I_min i_min)
  {
    // clang-format off
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
};

template <typename InputState, typename T = void>
struct derivative_input_types
{
  template <typename State, typename DeltaVec>
  static State Plus(const State&, const DeltaVec&) {};
  template <typename State>
  static State Log(const State&, const State&) {};
};
template <typename OutputState, typename T = void>
struct derivative_output_types
{
};

// Eigen input
template <typename InputState>
struct derivative_input_types<InputState, std::enable_if_t<not std::is_floating_point_v<InputState>>>
{
  using Scalar = typename InputState::Scalar;
  static constexpr Eigen::Index NInputs{ InputState::RowsAtCompileTime };
  template <typename DeltaVec>
  static InputState Plus(const InputState& state, const DeltaVec& delta)
  {
    return state + delta;
  };
  // F(x) (-) F(x')
  template <typename State>
  static State Log(const State& x)
  {
    return x;
  };
};

template <typename InputState>
struct derivative_input_types<InputState, std::enable_if_t<std::is_floating_point_v<InputState>>>
{
  using Scalar = InputState;
  static constexpr Eigen::Index NInputs{ 1 };
  template <typename DeltaVec>
  static InputState Plus(const InputState& state, const DeltaVec& delta)
  {
    return state + delta(0, 0);
  };
  // F(x) (-) F(x')
  template <typename State>
  static State Log(const State& x)
  {
    return x;
  };
};

// Eigen output
template <typename OutputState>
struct derivative_output_types<OutputState, std::enable_if_t<not std::is_floating_point_v<OutputState>>>
{
  static constexpr Eigen::Index NOutputs{ OutputState::RowsAtCompileTime };
};

template <typename OutputState>
struct derivative_output_types<OutputState, std::enable_if_t<std::is_floating_point_v<OutputState>>>
{
  static constexpr Eigen::Index NOutputs{ 1 };
};

template <class Function, typename InputState, S Evaluations, I_min MinDifference = -1, class... OtherInputs>
class first_order_derivative_t
{
  using Delta = derivative_input_types<InputState>;
  using Scalar = typename derivative_input_types<InputState>::Scalar;
  using OutputState = typename std::invoke_result<Function, InputState, OtherInputs...>::type;

  static constexpr Eigen::Index NInputs{ derivative_input_types<InputState>::NInputs };
  static constexpr Eigen::Index NOutputs{ derivative_output_types<OutputState>::NOutputs };

  using output_matrix_t = Eigen::Matrix<Scalar, NOutputs, NInputs>;
  using epsilon_matrix_t = Eigen::Matrix<Scalar, NInputs, NInputs>;

  // using FirstOrderDerivative = first_order_derivative_t<Function, InputState, Evaluations, MinDifference>;

  // using Operations = derivative_input_types<InputState>;

public:
  first_order_derivative_t(const double h, const Eigen::Index n_inputs, const Eigen::Index n_outputs)
    : _h(h)
    , _n_inputs(n_inputs)
    , _n_outputs(n_outputs)
    , _epsilon_matrix(_h * epsilon_matrix_t::Identity(_n_inputs, _n_inputs))
    , _zero_matrix(output_matrix_t::Zero(_n_outputs, _n_inputs))
    , _dh(_d * _h)
  {
    static_assert(_d != 0, "Invalid approximation_table for first_order_derivative_t");
  }

  template <Eigen::Index InputDim = NInputs, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  first_order_derivative_t(const double h = 0.01) : first_order_derivative_t(h, NInputs, NOutputs)
  {
  }

  first_order_derivative_t(const Function& model, const double h, const Eigen::Index n_inputs,
                           const Eigen::Index n_outputs)
    : first_order_derivative_t(h, n_inputs, n_outputs)
  {
    _model = model;
  }

  template <Eigen::Index InputDim = NInputs, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  first_order_derivative_t(const Function& model, const double h = 0.01)
    : first_order_derivative_t(model, h, NInputs, NOutputs)
  {
  }

  output_matrix_t operator()(const InputState& state, const OtherInputs&... other_inputs) const
  {
    output_matrix_t derivative{ _zero_matrix };
    iterate_columns<0>(state, derivative, other_inputs...);

    return derivative;
  }

  Function _model;

private:
  static constexpr approximation_row_t approximation_row{ first_order_derivative_table_t::approximation_table(
      Evaluations, MinDifference) };

  template <N_i n_i, int i, std::enable_if_t<(n_i != 0), bool> = true>
  inline void evaluate(const InputState& input, const int col_i, output_matrix_t& derivative,
                       const OtherInputs&... other_inputs) const
  {
    const auto epsilon_column{ i * _epsilon_matrix.col(col_i) };
    const InputState delta{ Delta::Plus(input, epsilon_column) };
    const OutputState Fi{ _model(delta, other_inputs...) };
    // const OutputState Fnew{ Delta::plus(f1, derivative.col(col_i)) };
    derivative.col(col_i) = derivative.col(col_i) + n_i * Delta::Log(Fi);
    // derivative.col(col_i) = Delta::plus(n_i * _model(delta), derivative.col(col_i));
  }

  template <N_i n_i, int i, std::enable_if_t<(n_i == 0), bool> = true>
  inline void evaluate(const InputState& input, const int col_i, output_matrix_t& derivative,
                       const OtherInputs&... other_inputs) const
  {
  }

  template <Eigen::Index I, std::enable_if_t<(NInputs != Eigen::Dynamic) && (I == NInputs), bool> = true>
  inline void iterate_columns(const InputState& state, output_matrix_t& derivative,
                              const OtherInputs&... other_inputs) const
  {
  }
  // If the vector size is known at compile time, avoid a for and rely on compiler/templates optimization
  template <Eigen::Index I, std::enable_if_t<(NInputs != Eigen::Dynamic) && (I < NInputs), bool> = true>
  inline void iterate_columns(const InputState& state, output_matrix_t& derivative,
                              const OtherInputs&... other_inputs) const
  {
    compute_column(I, state, derivative, other_inputs...);
    iterate_columns<I + 1>(state, derivative, other_inputs...);
  }

  // When the vector size is not known at compilation time. Compiler won't be able to optimize the for (unwrap)
  template <Eigen::Index I, std::enable_if_t<(NInputs == Eigen::Dynamic) && (I >= 0), bool> = true>
  void iterate_columns(const InputState& state, output_matrix_t& derivative, const OtherInputs&... other_inputs) const
  {
    for (std::size_t i = 0; i < _n_inputs; ++i)
    {
      compute_column(i, state, derivative, other_inputs...);
    }
  }

  inline void compute_column(const std::size_t col_i, const InputState& state, output_matrix_t& derivative,
                             const OtherInputs&... other_inputs) const
  {
    evaluate<_n1, 1 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n2, 2 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n3, 3 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n4, 4 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n5, 5 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n6, 6 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n7, 7 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n8, 8 - 5>(state, col_i, derivative, other_inputs...);
    evaluate<_n9, 9 - 5>(state, col_i, derivative, other_inputs...);

    derivative.col(col_i) /= _dh;
  }

  static constexpr D _d{ std::get<0>(approximation_row) };
  static constexpr N_i _n1{ std::get<1>(approximation_row) };
  static constexpr N_i _n2{ std::get<2>(approximation_row) };
  static constexpr N_i _n3{ std::get<3>(approximation_row) };
  static constexpr N_i _n4{ std::get<4>(approximation_row) };
  static constexpr N_i _n5{ std::get<5>(approximation_row) };
  static constexpr N_i _n6{ std::get<6>(approximation_row) };
  static constexpr N_i _n7{ std::get<7>(approximation_row) };
  static constexpr N_i _n8{ std::get<8>(approximation_row) };
  static constexpr N_i _n9{ std::get<9>(approximation_row) };

  const double _h;
  const double _dh;

  const Eigen::Index _n_inputs;
  const Eigen::Index _n_outputs;

public:
  const output_matrix_t _zero_matrix;
  const epsilon_matrix_t _epsilon_matrix;
};

template <class Function, typename InputState, S Evaluations, I_min MinDifference, class... OtherInputs>
constexpr approximation_row_t
    first_order_derivative_t<Function, InputState, Evaluations, MinDifference, OtherInputs...>::approximation_row;

}  // namespace math
}  // namespace prx
