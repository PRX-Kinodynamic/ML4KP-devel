import os
import math
import pytest
import libpyDirtMP as prx

# plant = prx.system_factory.create_system("pendulum", "pendulum-noisy")
def check(plant):
	n_plant = prx.uniform_noisy_plant(plant, -0.5, 0.5);

	ns = n_plant.get_state_space();
	ss = plant.get_state_space();

	pt = ss.make_point();
	pt_n = ns.make_point();


	for i in range(1000):
		ss.sample(pt);
		ss.copy_from(pt);
		ns.copy_to(pt_n);

		for j in range(ss.get_dimension()):
			assert ( not ss.equal_points( pt_n, pt ) );
			assert ( pt_n[j] - pt[j]  <= 0.5 );
	

def test_pendulum():
	pendulum = prx.system_factory.create_system("pendulum", "pendulum-noisy")
	check(pendulum);
	print(os.path.basename(__file__), "DONE")
