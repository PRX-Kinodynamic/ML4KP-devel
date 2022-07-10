#define BOOST_AUTO_TEST_MAIN lqr_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/general/transforms.hpp"

void pendulum_check()
{
    std::cout << "Checking pendulum..." << std::endl;
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
    ps -> copy_from_vector(v);
    
    auto Q = Eigen::MatrixXd::Identity(2,2);
    auto R = Eigen::MatrixXd::Identity(1,1);
    // auto pendulum = std::dynamic_pointer_cast<prx::pendulum_t>(plant);
    // auto pendulum = std::make_shared<prx::lti_t>(plant);
    // pendulum -> linearize();

    prx::lqr_t lqr(plant, Q, R, "LQR");
    lqr.set_goal(Eigen::VectorXd::Zero(2));
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    // std::cout << "A: " << lqr.get_linearized_plant -> get_A() << std::endl;
    // std::cout << "B: " << lqr.get_linearized_plant -> get_B() << std::endl;
    // std::cout << "K: " << K << std::endl;

    BOOST_CHECK(prx::are_approx_equal(K(0,0), 7.39050619, 1e-5));
    BOOST_CHECK(prx::are_approx_equal(K(0,1), 2.60611851, 1e-5));

    std::cout << "Pendulum OK" << std::endl;
}

void acrobot_check()
{
    std::cout << "Checking acrobot..." << std::endl;
    std::string plant_name = "Acrobot";
    std::string plant_path = "Acrobot";
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

    const auto ss = plant -> get_state_space();
    const auto cs = plant -> get_control_space();
    const auto ps = plant -> get_parameter_space();

    double mass = 1.0;
    double g = 9.81;
    double l1 = 1.0;
    double l2 = 1.0;
    double I1 = 0.2;
    double I2 = 1.0;
    double d1 = 1.0; // Damping
    double d2 = 1.0; 
    double viz_length = 20;
    std::vector<double> v = {mass,g,l1,l2,I1,I2,d1,d2,viz_length};
    ps -> copy_from_vector(v);
    
    // Eigen::Vector4d diagonal;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << 10, 10, 1, 1;
    // auto Q = diagonal.asDiagonal(); //Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1,1);
    // auto acrobot = std::dynamic_pointer_cast<prx::two_link_acrobot_t>(plant);
    // auto acrobot = std::make_shared<prx::lti_t>(plant);
    auto goal_pt = ss -> make_point();
    (*goal_pt)[0] = PRX_PI;
    // std::cout << "Q: " << Q << std::endl;
    // acrobot -> linearize();
    // std::cout << "A: " << acrobot -> get_A() << std::endl;
    // std::cout << "B: " << acrobot -> get_B() << std::endl;
    prx::lqr_t lqr(plant, Q, R, "LQR");
    lqr.set_goal(goal_pt);
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    
    std::cout << "K: " << K << std::endl;

    Eigen::Matrix4d A_from_matlab;
    A_from_matlab <<
                    0,                   0,    1.000000000000000,       0,
                    0,                   0,    0,                       1.000000000000000,
   12.907894736841627,  -2.581578947368598,    0,                       0,
  -14.456842105262695,   8.777368421053325,    0,                       0;

    PRX_DEBUG_PRINT
    Eigen::MatrixXd B_from_matlab;
    B_from_matlab.resize(4,1);
    B_from_matlab <<
                     0,
                     0,
    -1.578947368421053,
     3.368421052631580;

     Eigen::MatrixXd K_from_matlab;
     K_from_matlab.resize(1,4);
     K_from_matlab << 
        -1.263938740391128 * 1.0e+02,
        -0.328348178538952 * 1.0e+02,
        -0.492746311271306 * 1.0e+02,
        -0.189862747615557 * 1.0e+02;

    std::cout << "A from matlab: " << A_from_matlab << std::endl;
    std::cout << "B from matlab: " << B_from_matlab << std::endl;
    PRX_DEBUG_PRINT 
    BOOST_CHECK(prx::are_matrices_approx_equal(lqr.get_linearized_plant() -> get_A(), A_from_matlab, 1e-5));
   BOOST_CHECK(prx::are_matrices_approx_equal(lqr.get_linearized_plant() -> get_B(), B_from_matlab, 1e-5));
   BOOST_CHECK(prx::are_matrices_approx_equal(K, K_from_matlab, 1e-5));

    // BOOST_CHECK(prx::are_approx_equal(K(0,0), 7.39050619, 1e-5));
    // BOOST_CHECK(prx::are_approx_equal(K(0,1), 2.60611851, 1e-5));

    std::cout << "Acrobot OK" << std::endl;
}

BOOST_AUTO_TEST_CASE( lqr_test )
{   
	prx::simulation_step = 0.01;
    prx::init_random(112392);
    
    pendulum_check();
    acrobot_check();
}