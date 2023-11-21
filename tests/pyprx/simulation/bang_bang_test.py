import os
import math
import pytest
import PyML4KP as prx

def test_one_dimension_bang_bang_ctrl_build_correct():
	plant = prx.system_factory.create_system("pendulum", "pendulum-noisy")
	cs = plant.get_control_space();
	u_min = cs.get_lower_bound(0);
	u_equ = 0;
	u_max = cs.get_upper_bound(0);

	set_of_ctrls = [[u_min, u_equ, u_max]];
	bb_ctrl = prx.bang_bang(plant, set_of_ctrls, "bang_bang");

	assert(bb_ctrl.get_num_ctrls() == 3);
	assert(bb_ctrl.get_control_at(0)[0] == u_min);
	assert(bb_ctrl.get_control_at(1)[0] == u_equ);
	assert(bb_ctrl.get_control_at(2)[0] == u_max);

	print(os.path.basename(__file__), "DONE")

def test_two_dimension_bang_bang_ctrl_build_correct():
    plant = prx.system_factory.create_system("2D_Point", "2D_Point");

    cs = plant.get_control_space();
    u0_min = cs.get_lower_bound(0);
    u0_max = cs.get_upper_bound(0);
    u0_equ = u0_max - u0_min;

    u1_min = cs.get_lower_bound(1);
    u1_max = cs.get_upper_bound(1);
    u1_equ = u1_max - u1_min;

    set_of_ctrls = [[u0_min, u0_equ, u0_max], [u1_min, u1_equ, u1_max]];
    bb_ctrl = prx.bang_bang(plant, set_of_ctrls, "bang_bang");

    assert(bb_ctrl.get_num_ctrls() == 9);

    # Testing ctrls = {u_min, *}
    assert(bb_ctrl.get_control_at(0)[0] == u0_min);
    assert(bb_ctrl.get_control_at(0)[1] == u1_min);

    assert(bb_ctrl.get_control_at(1)[0] == u0_min);
    assert(bb_ctrl.get_control_at(1)[1] == u1_equ);

    assert(bb_ctrl.get_control_at(2)[0] == u0_min);
    assert(bb_ctrl.get_control_at(2)[1] == u1_max);

    # Testing ctrls = {u0_equ, *}
    assert(bb_ctrl.get_control_at(3)[0] == u0_equ);
    assert(bb_ctrl.get_control_at(3)[1] == u1_min);

    assert(bb_ctrl.get_control_at(4)[0] == u0_equ);
    assert(bb_ctrl.get_control_at(4)[1] == u1_equ);

    assert(bb_ctrl.get_control_at(5)[0] == u0_equ);
    assert(bb_ctrl.get_control_at(5)[1] == u1_max);

    # Testing ctrls = {u0_max, *}
    assert(bb_ctrl.get_control_at(6)[0] == u0_max);
    assert(bb_ctrl.get_control_at(6)[1] == u1_min);

    assert(bb_ctrl.get_control_at(7)[0] == u0_max);
    assert(bb_ctrl.get_control_at(7)[1] == u1_equ);

    assert(bb_ctrl.get_control_at(8)[0] == u0_max);
    assert(bb_ctrl.get_control_at(8)[1] == u1_max);
    
