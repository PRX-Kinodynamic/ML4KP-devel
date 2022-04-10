#define BOOST_AUTO_TEST_MAIN bang_bang_ctrls
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/controllers/bang_bang.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"

void check_2dpt()
{
	double epsilon = 0.01;
	auto plant = prx::system_factory_t::create_system("2D_Point", "2D_Point-bang_bang");
    prx::bang_bang_t bb_ctrl(plant);

    auto cs = plant -> get_control_space();
	auto lb = cs -> get_lower_bounds();
	auto ub = cs -> get_upper_bounds();
	auto ctrls_dim = bb_ctrl.get_num_ctrls();
    auto pt_bb_check = cs -> make_point();
    BOOST_CHECK(ctrls_dim == bb_ctrl.get_num_ctrls());
    
    auto pt_bb = bb_ctrl.get_control_at(0);
    cs -> copy_point_from_vector(pt_bb_check, {lb[0], lb[1]});
    BOOST_CHECK(prx::space_t::euclidean_2d(pt_bb, pt_bb_check, 0, cs -> get_dimension()) < epsilon);

    pt_bb = bb_ctrl.get_control_at(1);
    cs -> copy_point_from_vector(pt_bb_check, {ub[0], lb[1]});
    BOOST_CHECK(prx::space_t::euclidean_2d(pt_bb, pt_bb_check, 0, cs -> get_dimension()) < epsilon);

    pt_bb = bb_ctrl.get_control_at(2);
    cs -> copy_point_from_vector(pt_bb_check, {lb[0], ub[1]});
    BOOST_CHECK(prx::space_t::euclidean_2d(pt_bb, pt_bb_check, 0, cs -> get_dimension()) < epsilon);

    pt_bb = bb_ctrl.get_control_at(3);
    cs -> copy_point_from_vector(pt_bb_check, {ub[0], ub[1]});
    BOOST_CHECK(prx::space_t::euclidean_2d(pt_bb, pt_bb_check, 0, cs -> get_dimension()) < epsilon);

}

BOOST_AUTO_TEST_CASE( bang_bang_ctrls )
{
    
	check_2dpt();
}