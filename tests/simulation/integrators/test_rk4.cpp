#define BOOST_AUTO_TEST_MAIN rk4_integrator
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/integrators/runge_kutta4.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"

#include "prx/planning/world_model.hpp"

namespace mock
{
// Example taken from this Matlab function
// function stated = MassSpring(t, state)
//   x  = state(1); % unpack the state vector
//   xd = state(2); % unpack the state vector
//   k = 2.5; % Newtons per metre;
//   m = 1.5; % Kilograms
//   g = 9.8; % metres per second
//   xdd = ((-k*x)/m) + g;  % compute acceleration xdd
//   stated = [xd; xdd];  % output vector
struct mass_spring_system_t
{
  mass_spring_system_t()
    : _x(0)
    , _dx(0)
    , state_memory({ &_x, &_dx })
    , derivative_memory({ &_dx, &_ddx })
    , state_space("EE", state_memory, "space_test")
    , derivative_space("EE", derivative_memory, "deriv_space_test")
  {
  }
  //  ((-k*x)/m) + g;
  std::function<void()> compute_derivative = [&]() { _ddx = (-_k * _x / _m) + _g; };

  const double _k{ 2.5 };  // Newtons per metre;
  const double _m{ 1.5 };  // Kilograms
  const double _g{ 9.8 };  // metres per second

  double _x;
  double _dx;
  double _ddx;

  std::vector<double*> state_memory;
  std::vector<double*> derivative_memory;
  prx::space_t state_space;
  prx::space_t derivative_space;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(rk4_mass_spring_system_test)
{
  mock::mass_spring_system_t test{};
  prx::space_t* state_space = &test.state_space;
  prx::space_t* derivative_space = &test.derivative_space;
  prx::runge_kutta4_t rk4(state_space, derivative_space, test.compute_derivative);

  // Matlab code to replicate the 'solution' output
  // > t = 0:.1:10;
  // > state0 = [0.0, 0.0];
  // > [t,state] = ode78('MassSpring', t, state0);
  const std::vector<Eigen::Vector2d> solution{ { 0, 0 },
                                               { 0.0489319822415636, 0.977280045394589 },
                                               { 0.194913842726653, 1.93829447299627 },
                                               { 0.435514973957407, 2.86704922920342 },
                                               { 0.766731579589919, 3.74808625086592 },
                                               { 1.18305166737316, 4.56674147559824 },
                                               { 1.67754526887117, 5.30939008681156 },
                                               { 2.24198324417463, 5.96366311839264 },
                                               { 2.86697287275196, 6.51862422027501 },
                                               { 3.54209948035214, 6.96516777252598 },
                                               { 4.25613476696172, 7.29584374592318 },
                                               { 4.99720559107103, 7.50507336101271 },
                                               { 5.75297932542076, 7.58933845832332 },
                                               { 6.51086969581917, 7.54725496246047 },
                                               { 7.25825356378987, 7.37956497490661 },
                                               { 7.98268911388876, 7.08908203028035 },
                                               { 8.67212590629354, 6.68062405100644 },
                                               { 9.31509101590773, 6.16096400558957 },
                                               { 9.90084487517664, 5.53874537227144 },
                                               { 10.4197309117181, 4.8243762176332 },
                                               { 10.8631019151437, 4.02973104196137 },
                                               { 11.2235256867636, 3.16800123129815 },
                                               { 11.4949772093766, 2.25351736773185 },
                                               { 11.6729501848196, 1.30151411531051 },
                                               { 11.7545110150978, 0.327859054999536 },
                                               { 11.7383183029326, -0.651233157757591 },
                                               { 11.6246309475571, -1.61947993940807 },
                                               { 11.4153228163115, -2.56077536391042 },
                                               { 11.1138628479953, -3.45940687662782 },
                                               { 10.7253327550332, -4.30051837230025 },
                                               { 10.2561847205547, -5.07010117183899 },
                                               { 9.71418685731393, -5.75529048995365 },
                                               { 9.10834399315517, -6.34465294883748 },
                                               { 8.44875343777426, -6.82838990296097 },
                                               { 7.74641801544108, -7.19847930457515 },
                                               { 7.01303864845883, -7.44877883951029 },
                                               { 6.26080877606315, -7.57511306287019 },
                                               { 5.50223631885884, -7.57535914941865 },
                                               { 4.74997684648208, -7.4494820549124 },
                                               { 4.01649371737431, -7.19968436678793 },
                                               { 3.31399788065765, -6.83010316571051 },
                                               { 2.65421210170676, -6.34682422572104 },
                                               { 2.04813503973324, -5.75786312708993 },
                                               { 1.50585004579783, -5.07304244468483 },
                                               { 1.03636791394604, -4.30379893956978 },
                                               { 0.647492818158956, -3.4629546815397 },
                                               { 0.345700667833226, -2.56448603040428 },
                                               { 0.136026101800188, -1.62332641313386 },
                                               { 0.0219902381921709, -0.655169030101606 },
                                               { 0.00539525728083684, 0.323913455308282 },
                                               { 0.0865334550816817, 1.29762732638012 },
                                               { 0.26411155904077, 2.24974643750055 },
                                               { 0.5351996354272, 3.16441015983089 },
                                               { 0.895269715895607, 4.02639453873117 },
                                               { 1.33829591144182, 4.82136065326294 },
                                               { 1.85688778183416, 5.53608416796305 },
                                               { 2.44242872884427, 6.15867006730801 },
                                               { 3.08519161495839, 6.67874733828593 },
                                               { 3.77446569011114, 7.08761310303765 },
                                               { 4.49878299350238, 7.37859252669041 },
                                               { 5.24609401854218, 7.54682317332288 },
                                               { 6.00396168453053, 7.589428622206 },
                                               { 6.75976903909306, 7.50566414514285 },
                                               { 7.50093179481878, 7.29694366335523 },
                                               { 8.21511087413772, 6.96678411557042 },
                                               { 8.89042013641428, 6.52070336901627 },
                                               { 9.51562446123107, 5.96610780505326 },
                                               { 10.0803166142214, 5.31220262533365 },
                                               { 10.575059602607, 4.56987416712433 },
                                               { 10.991717141109, 3.75150838068677 },
                                               { 11.3233416233916, 2.87071331292918 },
                                               { 11.5643569428675, 1.94212637646132 },
                                               { 11.7107243057083, 0.981196865925681 },
                                               { 11.7600209947491, 0.00392968965366514 },
                                               { 11.7114576505554, -0.973393717062706 },
                                               { 11.5658596359451, -1.93450133068552 },
                                               { 11.3256380502736, -2.86340736373055 },
                                               { 10.9947717990094, -3.74465648809148 },
                                               { 10.5787592943488, -4.5635329684916 },
                                               { 10.0845745797727, -5.30652083085218 },
                                               { 9.52042978828182, -5.96124149898568 },
                                               { 8.89568214821052, -6.51673386204275 },
                                               { 8.22071786434901, -6.96372007473392 },
                                               { 7.50678308499454, -7.29477405905039 },
                                               { 6.76578090188946, -7.50442002045967 },
                                               { 6.01005333019723, -7.58918829157553 },
                                               { 5.25216721546139, -7.54765581627999 },
                                               { 4.5047274642496, -7.38049134787074 },
                                               { 3.78020801137112, -7.09045327515813 },
                                               { 3.09059712017782, -6.68246226486161 },
                                               { 2.44737820956857, -6.16328930006634 },
                                               { 1.86129566963314, -5.5415176390565 },
                                               { 1.34212491262115, -4.82747214119678 },
                                               { 0.898500246968766, -4.03305606720466 },
                                               { 0.537785473107449, -3.17152640586647 },
                                               { 0.265972099814929, -2.25723877949652 },
                                               { 0.0875900798276472, -1.30539298017665 },
                                               { 0.00561670383017509, -0.331809472254451 },
                                               { 0.0214130808171497, 0.647298296258097 },
                                               { 0.134717048006129, 1.6156327349394 },
                                               { 0.343642806721063, 2.55707733847426 } };

  prx::constants::precision = 10;
  prx::space_point_t xt{ state_space->make_point() };
  prx::space_point_t dxt{ derivative_space->make_point() };
  const double dt{ 0.0001 };
  std::vector<Eigen::Vector2d> result{};
  for (double i = 0.0; i < 10; i += dt)
  {
    state_space->copy_to(xt);
    result.emplace_back(xt->as<Eigen::Vector2d>());
    rk4.integrate(dt);
  }
  state_space->copy_to(xt);
  result.emplace_back(xt->as<Eigen::Vector2d>());
  const double epsilon{ 0.01 };
  const std::size_t step{ static_cast<std::size_t>(0.1 / dt) };
  PRX_DEBUG_VAR_2(result.size(), step);
  for (std::size_t i = 0; i < result.size(); i += step)
  {
    BOOST_REQUIRE_SMALL(result[i][0] - solution[i / step][0], epsilon);
    BOOST_REQUIRE_SMALL(result[i][1] - solution[i / step][1], epsilon);
  }
}

BOOST_AUTO_TEST_CASE(euler_rk4_test)
{
  auto acrobot_euler = prx::system_factory_t::create_system("Acrobot", "Acrobot_euler");
  auto acrobot_rk4 = prx::system_factory_t::create_system("Acrobot", "Acrobot_rk4");

  prx::simulation_step = 0.000001;
  std::dynamic_pointer_cast<prx::plant_t>(acrobot_euler)->set_integrator(prx::integrator_t::kEULER);
  prx::simulation_step = 0.001;
  std::dynamic_pointer_cast<prx::plant_t>(acrobot_rk4)->set_integrator(prx::integrator_t::kRK4);

  std::vector<std::vector<double> > ctrls{ { 6.94678, 0.12, 0.12 },  { 6.70279, 0.28, 0.4 },   { 6.22159, 0.33, 0.73 },
                                           { -3.61730, 0.36, 1.09 }, { -5.02736, 0.23, 1.32 }, { -1.41110, 0.2, 1.52 },
                                           { -6.98014, 0.43, 1.95 }, { -6.73070, 0.2, 2.15 },  { -3.74701, 0.1, 2.25 },
                                           { 2.47204, 0.29, 2.54 },  { 3.82880, 0.37, 2.91 },  { -4.03203, 0.01, 2.92 },
                                           { -2.47204, 0.43, 3.35 } };

  prx::distance_function_t dist = [](prx::space_point_t a, prx::space_point_t b) {
    double res = 0;
    for (int i = 0; i < a->get_dim(); ++i)
    {
      res += std::pow(a->at(i) - b->at(i), 2);
    }
    return std::sqrt(res);
  };

  prx::world_model_t world_model({ acrobot_euler }, {});
  world_model.create_context("disk_context", { "Acrobot_euler" }, {});
  auto context = world_model.get_context("disk_context");
  auto sg = context.first;
  auto state_space = context.first->get_state_space();
  auto control_space = context.first->get_control_space();

  prx::space_point_t curr_state;
  prx::plan_t plan_euler(control_space);
  prx::trajectory_t euler_traj(state_space);

  prx::world_model_t world_model_rk4({ acrobot_rk4 }, {});
  world_model_rk4.create_context("context_rk4", { "Acrobot_rk4" }, {});
  auto context_rk4 = world_model_rk4.get_context("context_rk4");
  auto sg_rk4 = context.first;
  auto state_space_rk4 = context.first->get_state_space();
  auto control_space_rk4 = context.first->get_control_space();

  prx::plan_t plan_rk4(control_space_rk4);
  prx::trajectory_t traj_rk4(state_space_rk4);

  for (auto ctrl : ctrls)
  {
    plan_euler.append_onto_back(ctrl[1]);
    plan_euler.back().control->at(0) = ctrl[0];

    plan_rk4.append_onto_back(ctrl[1]);
    plan_rk4.back().control->at(0) = ctrl[0];
  }

  curr_state = state_space->make_point();
  curr_state->at(0) = 0.0;
  curr_state->at(1) = 0.0;
  curr_state->at(2) = 0.0;
  curr_state->at(3) = 0.0;

  prx::simulation_step = 0.000001;
  sg->propagate(curr_state, plan_euler, euler_traj);
  prx::simulation_step = 0.001;
  sg_rk4->propagate(curr_state, plan_rk4, traj_rk4);

  for (double i = 0.01; i <= 1; i += 0.01)
  {
    auto pt_euler = euler_traj.at(i);
    auto pt_rk4 = traj_rk4.at(i);

    BOOST_REQUIRE_SMALL(dist(pt_rk4, pt_euler), 0.1);

    // Check to ensure that the trajectories are different than zeros
    BOOST_REQUIRE(Vec(pt_rk4).norm() > 0.0);
    BOOST_REQUIRE(Vec(pt_euler).norm() > 0.0);
  }
}