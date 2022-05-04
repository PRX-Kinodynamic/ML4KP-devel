import os
import math
import pytest
import libpyDirtMP as prx

def test_1():
	smp = prx.space_memory(3)
	space_1 = prx.space_t("EEE", smp, "space_1");
	
	pt_sp1 = space_1.make_point();
	pt_ns1 = space_1.make_point();
	
	space_1.copy_to_point(pt_sp1);
	
	noisy_space_1 = prx.uniform_noisy_space(space_1, -0.5, 0.5);
	
	for i in range(1000):
		noisy_space_1.copy_to_point(pt_ns1);
	
		for j in range(space_1.get_dimension()):
			assert ( pt_sp1[j] - pt_ns1[j]  <= 0.5 );

	print(os.path.basename(__file__), "DONE")
