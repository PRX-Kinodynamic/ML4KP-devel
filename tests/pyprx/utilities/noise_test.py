import os
import math
import pytest
import libpyDirtMP as prx


def test_pendulum():
	plant_name = "pendulum"
	plant_path = "pendulum"
	plant = prx.system_factory.create_system(plant_name, plant_path);
	ss = plant.get_state_space()
	pt = ss.make_point()

	unif_noise = prx.uniform_noise(-1,1);
	for i in range(100):
		ss.copy_point_from_vector(pt, [0,1]);
		unif_noise.add_noise(pt);
		# unif_noise.add_noise(pt, 0, 2);
		print("Point: ", pt)
	print(os.path.basename(__file__), "DONE")