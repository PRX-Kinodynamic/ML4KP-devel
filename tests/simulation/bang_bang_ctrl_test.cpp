#define BOOST_TEST_MODULE bang_bang_ctrls
#include <string>
#include <boost/test/included/unit_test.hpp>

#include "prx/simulation/controllers/bang_bang.hpp"

#include "prx/simulation/plants/plants.hpp"

BOOST_AUTO_TEST_CASE(one_dimension_bang_bang_ctrl_build_correct)
{
  prx::system_ptr_t plant = prx::system_factory_t::create_system("pendulum", "pendulum");

  prx::space_t* cs = plant->get_control_space();
  double u_min = cs->get_lower_bound(0);
  double u_equ = 0;
  double u_max = cs->get_upper_bound(0);

  std::vector<std::vector<double>> set_of_ctrls = { { u_min, u_equ, u_max } };
  prx::bang_bang_t bb_ctrl(plant, set_of_ctrls);

  BOOST_CHECK(bb_ctrl.get_num_ctrls() == 3);
  BOOST_CHECK(bb_ctrl.get_control_at(0)->at(0) == u_min);
  BOOST_CHECK(bb_ctrl.get_control_at(1)->at(0) == u_equ);
  BOOST_CHECK(bb_ctrl.get_control_at(2)->at(0) == u_max);
}

BOOST_AUTO_TEST_CASE(two_dimension_bang_bang_ctrl_build_correct)
{
  prx::system_ptr_t plant = prx::system_factory_t::create_system("2D_Point", "2D_Point");

  prx::space_t* cs = plant->get_control_space();
  double u0_min = cs->get_lower_bound(0);
  double u0_max = cs->get_upper_bound(0);
  double u0_equ = u0_max - u0_min;

  double u1_min = cs->get_lower_bound(1);
  double u1_max = cs->get_upper_bound(1);
  double u1_equ = u1_max - u1_min;

  std::vector<std::vector<double>> set_of_ctrls = { { u0_min, u0_equ, u0_max }, { u1_min, u1_equ, u1_max } };
  prx::bang_bang_t bb_ctrl(plant, set_of_ctrls);

  BOOST_CHECK(bb_ctrl.get_num_ctrls() == 9);

  // Testing ctrls = {u_min, *}
  BOOST_CHECK(bb_ctrl.get_control_at(0)->at(0) == u0_min);
  BOOST_CHECK(bb_ctrl.get_control_at(0)->at(1) == u1_min);

  BOOST_CHECK(bb_ctrl.get_control_at(1)->at(0) == u0_min);
  BOOST_CHECK(bb_ctrl.get_control_at(1)->at(1) == u1_equ);

  BOOST_CHECK(bb_ctrl.get_control_at(2)->at(0) == u0_min);
  BOOST_CHECK(bb_ctrl.get_control_at(2)->at(1) == u1_max);

  // Testing ctrls = {u0_equ, *}
  BOOST_CHECK(bb_ctrl.get_control_at(3)->at(0) == u0_equ);
  BOOST_CHECK(bb_ctrl.get_control_at(3)->at(1) == u1_min);

  BOOST_CHECK(bb_ctrl.get_control_at(4)->at(0) == u0_equ);
  BOOST_CHECK(bb_ctrl.get_control_at(4)->at(1) == u1_equ);

  BOOST_CHECK(bb_ctrl.get_control_at(5)->at(0) == u0_equ);
  BOOST_CHECK(bb_ctrl.get_control_at(5)->at(1) == u1_max);

  // Testing ctrls = {u0_max, *}
  BOOST_CHECK(bb_ctrl.get_control_at(6)->at(0) == u0_max);
  BOOST_CHECK(bb_ctrl.get_control_at(6)->at(1) == u1_min);

  BOOST_CHECK(bb_ctrl.get_control_at(7)->at(0) == u0_max);
  BOOST_CHECK(bb_ctrl.get_control_at(7)->at(1) == u1_equ);

  BOOST_CHECK(bb_ctrl.get_control_at(8)->at(0) == u0_max);
  BOOST_CHECK(bb_ctrl.get_control_at(8)->at(1) == u1_max);
}