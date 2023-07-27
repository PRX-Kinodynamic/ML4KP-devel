#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
namespace estimation
{
using DerivativeEvaluations = uint8_t;

template <typename State, typename Observation>
struct EkfState
{
  static constexpr Eigen::Index StateDim{ State::RowsAtCompileTime };
  static constexpr Eigen::Index ObservationDim{ Observation::RowsAtCompileTime };

  using Step = std::size_t;

  using StateCovariance = Eigen::Matrix<double, StateDim, StateDim>;
  using ProcessNoiseCovariance = Eigen::Matrix<double, StateDim, StateDim>;
  using MeasurementNoiseCovariance = Eigen::Matrix<double, ObservationDim, ObservationDim>;
  using KallmanGain = Eigen::Matrix<double, StateDim, ObservationDim>;

  Step k;

  State estimation;
  Observation observation;

  // State/Observation noise {w,v}_k ~ N(0,{Q_k,R_k} )
  State w_k;
  Observation v_k;

  // Covariance
  StateCovariance P_k;

  // Process noise covariance
  ProcessNoiseCovariance Q_k;

  // Measurement noise covariance
  MeasurementNoiseCovariance R_k;

  // Kallman Gain
  KallmanGain K_k;
};

template <typename Model, typename MeasurementFunction, typename State, typename Observation,
          DerivativeEvaluations Evaluations = 4>
class extended_kalman_filter_t
{
  static constexpr Eigen::Index StateDim{ State::RowsAtCompileTime };
  static constexpr Eigen::Index ObservationDim{ Observation::RowsAtCompileTime };

  using EkfState = EkfState<State, Observation>;
  using StateJacobian = prx::math::first_order_derivative_t<Model, State, Evaluations>;
  using MeasurementJacobian = prx::math::first_order_derivative_t<MeasurementFunction, State, Evaluations>;

public:
  using MatrixNN = Eigen::Matrix<double, StateDim, StateDim>;
  using MatrixMM = Eigen::Matrix<double, ObservationDim, ObservationDim>;
  using MatrixNM = Eigen::Matrix<double, StateDim, ObservationDim>;
  using MatrixMN = Eigen::Matrix<double, ObservationDim, StateDim>;

  using Step = typename EkfState::Step;
  using StateCovariance = typename EkfState::StateCovariance;
  using ProcessNoiseCovariance = typename EkfState::ProcessNoiseCovariance;
  using MeasurementNoiseCovariance = typename EkfState::MeasurementNoiseCovariance;
  using KallmanGain = typename EkfState::KallmanGain;

  extended_kalman_filter_t(const Model& model, const MeasurementFunction& measure, const double& epsilon = 0.01)
    : _model(model), _measure(measure), _Im(MatrixMM::Identity()), _In(MatrixNN::Identity()), _epsilon(epsilon)
  {
  }

  void init(const State x_0, const StateCovariance& P_0, const ProcessNoiseCovariance& Q_0,
            const MeasurementNoiseCovariance& R_0)
  {
    _ekf_state.k = 0;
    _ekf_state.P_k = P_0;
    _ekf_state.Q_k = Q_0;
    _ekf_state.R_k = R_0;
    _ekf_state.K_k = KallmanGain::Ones();
    _ekf_state.estimation = x_0;
  }

  State operator()(const State& expected, const Observation& observation)
  {
    return this->operator()(expected, observation, _ekf_state.P_k, _ekf_state.Q_k, _ekf_state.R_k);
  }

  State operator()(const State& expected, const Observation& observation, const StateCovariance& P_k,
                   const ProcessNoiseCovariance& Q_k, const MeasurementNoiseCovariance& R_k)
  {
    _ekf_state.k++;
    _ekf_state.Q_k = Q_k;
    _ekf_state.R_k = R_k;
    _ekf_state.P_k = P_k;
    _ekf_state.estimation = expected;
    _ekf_state.observation = observation;

    predict();
    correct();

    return _ekf_state.estimation;
  }

protected:
  void predict()
  {
    const State x_a{ _ekf_state.estimation };

    const Step k{ _ekf_state.k };
    const ProcessNoiseCovariance Q_k{ _ekf_state.Q_k };
    const StateCovariance P_k{ _ekf_state.P_k };

    const StateJacobian f_partial_derivative{ _model, _epsilon };

    const MatrixNN F_kx = f_partial_derivative(x_a);

    _ekf_state.P_k = F_kx * P_k * F_kx.transpose() + Q_k;
  }

  void correct()
  {
    // Variables at k-1
    // const State z_fk{ _ekf_state.observation };  // z(k-1)

    // Variables at k
    const Step k{ _ekf_state.k };                     // k
    const State x_f{ _ekf_state.estimation };         // x_f(k)
    const Observation z_k{ _ekf_state.observation };  // z(k)
    const MeasurementNoiseCovariance R_k{ _ekf_state.R_k };
    const StateCovariance P_k{ _ekf_state.P_k };

    const MeasurementJacobian h_partial_derivative{ _measure, _epsilon };

    const MatrixMN H_k1x = h_partial_derivative(x_f);

    // Inverse could cause numerical issues. This method is recomended by Eigen, but there could be better for
    // specific cases.
    const MatrixMM S_k1 = H_k1x * P_k * H_k1x.transpose() + R_k;
    const MatrixMM S_k1_inverse = S_k1.colPivHouseholderQr().solve(_Im);

    _ekf_state.K_k = P_k * H_k1x.transpose() * S_k1_inverse;

    const KallmanGain K_k{ _ekf_state.K_k };

    _ekf_state.estimation = x_f + K_k * (z_k - _measure(x_f));

    _ekf_state.P_k = (_In - K_k * H_k1x) * P_k;
  }

private:
  EkfState _ekf_state;

  Model _model;
  MeasurementFunction _measure;

  const double _epsilon;
  const MatrixMM _Im;  // identity
  const MatrixNN _In;  // identity
};
}  // namespace estimation
}  // namespace prx