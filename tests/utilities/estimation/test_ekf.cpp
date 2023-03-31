#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/estimation/extended_kalman_filter.hpp"

BOOST_AUTO_TEST_CASE(paths_test)
{
  using State = Eigen::Vector<double, 2>;
  using Observation = Eigen::Vector<double, 2>;
  using Model = std::function<State(State)>;
  using MeasurementFunction = std::function<Observation(Observation)>;

  using Ekf = prx::estimation::extended_kalman_filter_t<Model, MeasurementFunction, State, Observation>;

  std::random_device rd{};
  std::mt19937 gen{ rd() };
  std::normal_distribution<double> w{ 0, 0.01 };
  std::normal_distribution<double> v{ 0, 0.01 };

  Model f = [](const State& s) { return s; };
  MeasurementFunction h = [](const Observation& z) { return z + Observation(0.1, -0.1); };

  Ekf ekf(f, h);
  ekf.init(State::Ones(), Ekf::StateCovariance::Identity() * 0.1, Ekf::ProcessNoiseCovariance::Identity() * 0.1,
           Ekf::MeasurementNoiseCovariance::Identity() * 0.1);

  State estimation{};
  for (int i = 0; i < 100; ++i)
  {
    Observation observation{ Observation::Ones() + Observation::Ones() * v(gen) };
    estimation = ekf(State::Ones(), observation);
    // std::cout << "estimation: " << estimation.transpose() << std::endl;
  }
  State expected_estimation{ State::Ones() };

  State result{ expected_estimation - estimation };
  BOOST_CHECK_SMALL(result[0], 1e-1);
  BOOST_CHECK_SMALL(result[1], 1e-1);
}