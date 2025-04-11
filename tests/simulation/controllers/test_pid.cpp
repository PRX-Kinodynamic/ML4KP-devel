#define BOOST_AUTO_TEST_MAIN lqr_controller_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/pid.hpp"
#include "prx/utilities/general/debug_utils.hpp"

using PID = prx::simulation::pid_t<1>;
using Array = Eigen::Array<double, 1, 1>;
using Vector = Eigen::Vector<double, 1>;
namespace mock
{
// Model adapted from http://www.inf.fu-berlin.de/lehre/SS06/Robotik/motors_Ue7.pdf
// Models for DC Motors by Raul Rojas
struct dc_motor_t
{
  dc_motor_t() : kp(10.0), R(1.0), J(10.0), w(0.0), wdot(0.0), wf(10.0), dt(0.01)
  {
  }

  void operator()(double Vin)
  {
    const double Ts{ kp * Vin / R };
    wdot = (Ts / J) * (1.0 - w / wf);
    w = w + wdot * dt;
  }
  double kp, R;
  double J;
  double w, wdot;
  double wf;
  double dt;
};
struct open_loop_t
{
  open_loop_t() : Vref(10) {};
  inline Vector operator()(const Vector& x, const Vector& ref) const
  {
    return Vref;
  }

  const Vector Vref;
};

template <typename Controller>
std::vector<double> simulate(const double Tf, Controller& ctrl)
{
  dc_motor_t motor;
  std::vector<double> x;
  x.push_back(motor.w);
  for (double ti = 0; ti < Tf; ti += motor.dt)
  {
    const double u{ ctrl(Vector(motor.w), Vector(motor.wf))[0] };
    motor(u);
    x.push_back(motor.w);
    // PRX_DBG_VARS(ti, motor.w);
  }
  return x;
}

}  // namespace mock

BOOST_AUTO_TEST_CASE(proportional_only_test)
{
  // A P controller should reach the desired ref faster but may not exactly
  mock::open_loop_t open_loop;

  PID pid(Array(10), Array(0), Array(0), Vector(0));

  std::vector<double> x_open_loop{ mock::simulate(10, open_loop) };
  std::vector<double> x_pid{ mock::simulate(10, pid) };

  const double ref{ 9.0 };
  int idx_ol{ 100000 }, idx_pid{ 100000 };
  for (int i = 0; i < x_pid.size(); ++i)
  {
    const double& xol{ x_open_loop[i] };
    const double& xpid{ x_pid[i] };

    if (xol > 9.0)
    {
      idx_ol = std::min(idx_ol, i);
    }
    if (xpid > 9.0)
    {
      idx_pid = std::min(idx_pid, i);
    }
  }
  BOOST_CHECK(idx_pid < idx_ol);
}

BOOST_AUTO_TEST_CASE(integral_only_test)
{
  // An I controller should reach the desired ref (numerically) 'exactly'
  mock::open_loop_t open_loop;

  PID pid(Array(0), Array(0.1), Array(0), Vector(0));

  std::vector<double> x_open_loop{ mock::simulate(10, open_loop) };
  std::vector<double> x_pid{ mock::simulate(10, pid) };

  const double ref{ 9.9 };
  int idx_ol{ 100000 }, idx_pid{ 100000 };
  for (int i = 0; i < x_pid.size(); ++i)
  {
    const double& xol{ x_open_loop[i] };
    const double& xpid{ x_pid[i] };

    if (xol > 9.9)
    {
      idx_ol = std::min(idx_ol, i);
    }
    if (xpid > 9.9)
    {
      idx_pid = std::min(idx_pid, i);
    }
  }
  BOOST_CHECK(idx_pid < idx_ol);
  BOOST_REQUIRE_SMALL(x_pid.back() - 10, 1e-9);
}

BOOST_AUTO_TEST_CASE(pid_test)
{
  // A D controller should reach the desired ref (numerically) 'exactly'
  mock::open_loop_t open_loop;

  PID pid(Array(5), Array(0.1), Array(0.1), Vector(0));

  std::vector<double> x_open_loop{ mock::simulate(10, open_loop) };
  std::vector<double> x_pid{ mock::simulate(10, pid) };

  const double ref{ 9.9 };
  int idx_ol{ 100000 }, idx_pid{ 100000 };
  for (int i = 0; i < x_pid.size(); ++i)
  {
    const double& xol{ x_open_loop[i] };
    const double& xpid{ x_pid[i] };

    // PRX_DBG_VARS(xol, xpid);
    if (xol > ref)
    {
      idx_ol = std::min(idx_ol, i);
    }
    if (xpid > ref)
    {
      idx_pid = std::min(idx_pid, i);
    }
  }

  BOOST_CHECK(idx_pid < idx_ol);
  BOOST_REQUIRE_SMALL(x_pid.back() - 10, 1e-9);
}
