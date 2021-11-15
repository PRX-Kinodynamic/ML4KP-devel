#define BOOST_AUTO_TEST_MAIN lqr_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/plants/plants.hpp"

BOOST_AUTO_TEST_CASE( lqr_test )
{   
	prx::simulation_step = 0.01;
    prx::init_random(112392);

    std::string plant_name = "pendulum";
    std::string plant_path = "pendulum";
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

    const auto ss = plant -> get_state_space();
    const auto cs = plant -> get_control_space();
    const auto ps = plant -> get_parameter_space();

    double length = 0.5;
    double friction = 0.1;
    double mass = 0.15;
    double normalize = 1;
    std::vector<double> v = {length,friction,mass,normalize};
    // ps -> copy_from_vector(v);

    auto Q = Eigen::MatrixXd::Identity(2,2);
    auto R = Eigen::MatrixXd::Identity(1,1);
	auto pendulum = std::dynamic_pointer_cast<prx::pendulum_t>(plant);
	
    pendulum -> linearize();

    prx::lqr_t lqr(pendulum, Q, R, "LQR");
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    std::cout << "A: " << pendulum -> get_A() << std::endl;
    std::cout << "B: " << pendulum -> get_B() << std::endl;
    std::cout << "K: " << K << std::endl;

    BOOST_CHECK(prx::are_approx_equal(K(0,0), 7.39050619, 1e-5));
    BOOST_CHECK(prx::are_approx_equal(K(0,1), 2.60611851, 1e-5));

}