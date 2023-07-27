
#include "prx/utilities/estimation/extended_kalman_filter.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"

using State = Eigen::Vector<double, 5>;
using Observation = Eigen::Vector<double, 3>;
using Control = Eigen::Vector<double, 2>;
using Model = std::function<State(State)>;
using MeasurementFunction = std::function<Observation(const State&)>;

using Ekf = prx::estimation::extended_kalman_filter_t<Model, MeasurementFunction, State, Observation>;
const double WHEEL_DISTANCE{ 0.115 * 2.0 };

int main(int argc, char* argv[])
{
  auto params = prx::param_loader("executables/factor_graphs/ackermann_fg.yaml", argc, argv);
  std::random_device rd{};
  std::mt19937 gen{ rd() };
  std::normal_distribution<double> w{ 0, 0.01 };
  std::normal_distribution<double> v{ 0, 0.01 };

  const double dt{ 0.01 };

  Model f = [&](const State& s) {
    State next{ State::Zero() };
    const double x{ s[0] };
    const double y{ s[1] };
    const double theta{ s[2] };
    const double V{ s[3] };
    const double gamma{ s[4] };
    next[0] = V * std::cos(theta);
    next[1] = V * std::sin(theta);
    next[2] = (V / WHEEL_DISTANCE) * std::tan(gamma);
    return s + next * dt;
  };
  MeasurementFunction h = [](const State& x_hat) { return x_hat.head(3); };

  Ekf ekf(f, h);
  ekf.init(State::Zero(), Ekf::StateCovariance::Identity() * 0.01, Ekf::ProcessNoiseCovariance::Identity() * 0.01,
           Ekf::MeasurementNoiseCovariance::Identity() * 0.01);

  std::vector<Control> ctrls = {
    { 0.933466, -0.001465 }, { 0.944210, 0.010173 }, { 0.942091, -0.062425 }, { 0.804396, 0.042345 }
  };

  Control u{};
  State estimation{};
  const std::string traj_file{ params["traj_file"].as<>() };
  prx::utilities::csv_reader_t reader(traj_file);

  std::size_t idx{ 0 };
  std::vector<double> line;
  Observation observation{ Observation::Zero() };
  // Observation observation{ line[0] + v(gen), line[1] + v(gen), line[2] + v(gen) };
  while (reader.has_next_line())
  {
    if (idx % 10 == 0)
    {
      line = reader.next_line<double>();
      if (line.size() == 0)
        continue;
      observation = Observation{ line[0] + v(gen), line[1] + v(gen), line[2] + v(gen) };
      std::cout << "GT: " << line[0] << " " << line[1] << " " << line[2] << std::endl;
      std::cout << "observation: " << observation.transpose() << std::endl;
    }
    if (idx % 1'000 == 0)
    {
      u = ctrls[0];
      estimation[3] = u[0];
      estimation[4] = u[1];
      ctrls.erase(ctrls.begin());
    }
    estimation = ekf(estimation, observation);
    std::cout << "estimation: " << estimation.transpose() << std::endl;
    idx += 10;
  }

  return 0;
}