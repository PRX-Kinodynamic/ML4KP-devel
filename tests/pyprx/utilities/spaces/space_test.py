import os
import math
import pytest
import libpyDirtMP as prx

def test_space_copy_to_pyobject():
	smp = prx.space_memory(3)
	space = prx.space_t("EEE", smp, "space_1");
	
	space[0] = 1.0
	space[1] = 2.0
	space[2] = 3.0
	list_0 = [0.0]*3
	space.copy_to(list_0)
	
	assert (space[0] == list_0[0] );
	assert (space[1] == list_0[1] );
	assert (space[2] == list_0[2] );

def test_space_copy_to_prxobject():
	smp = prx.space_memory(3)
	space = prx.space_t("EEE", smp, "space_1");

	space[0] = 1.0
	space[1] = 2.0
	space[2] = 3.0
	pt = space.make_point();
	space.copy_to(pt)

	assert (space[0] == pt[0] );
	assert (space[1] == pt[1] );
	assert (space[2] == pt[2] );


def test_space_copy_from_pyobject():
	smp = prx.space_memory(3) # Space Memory Pointer
	space = prx.space_t("EEE", smp, "space_1");
	
	list_0 = [1,2,3]
	space.copy_from(list_0)
	
	assert (space[0] == list_0[0] );
	assert (space[1] == list_0[1] );
	assert (space[2] == list_0[2] );

def test_space_copy_from_prxobject():
	smp = prx.space_memory(3) # Space Memory Pointer
	space = prx.space_t("EEE", smp, "space_1");
	
	pt = space.make_point();
	pt[0] = 4
	pt[1] = 5
	pt[2] = 6
	space.copy_from(pt)
	
	assert (space[0] == pt[0] );
	assert (space[1] == pt[1] );
	assert (space[2] == pt[2] );


def test_space_copy():
	smp = prx.space_memory(3) # Space Memory Pointer
	space = prx.space_t("EEE", smp, "space_1");
	
	list_0 = [0]*3
	list_1 = [1,2,3]
	space.copy(list_0, list_1)
	
	assert (list_1[0] == list_0[0] );
	assert (list_1[1] == list_0[1] );
	assert (list_1[2] == list_0[2] );

