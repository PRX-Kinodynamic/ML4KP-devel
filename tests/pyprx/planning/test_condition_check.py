import os
import math
import pytest
import libpyDirtMP as prx

def test_1():
	iters = 1e3;
	check_1 = prx.condition_check("iterations" , iters );
	# prx::simulation_step = 0.01;
	prx.set_simulation_step( 0.01)

	iters_test = 0;
	while True:
		iters_test += 1;
		if check_1.check():
			break
	
	assert iters_test == iters


	smp = prx.space_memory(3)
	space_1 = prx.space_t("EEE", smp, "space_1");

	goal = space_1.make_point();
	pt = space_1.make_point();
	pt_aux = space_1.make_point();
	space_1.copy_point_from_vector(goal, [1,1,1]);

	cc = prx.create_default_goal_check(space_1, goal, 0.1 );

	space_1.copy_point_from_vector(pt_aux, [0,0,0]);
	check_2 = prx.condition_check( cc );
	check_1.reset();
	check_1.add_condition(check_2);
	iters_test = 0;

	while True:
		iters_test += 1;
		pt_aux[0] += 0.1;
		pt_aux[1] += 0.1;
		pt_aux[2] += 0.1;
		space_1.copy_from_point(pt_aux);

		if check_1.check():
			break

	assert(iters_test < iters);

	print(os.path.basename(__file__), "DONE")