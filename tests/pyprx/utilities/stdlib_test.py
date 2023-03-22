import os
import math
import pytest
import libpyDirtMP as prx

def test_std_vector_doubles():
	vec_doubles = prx.vector_of_doubles();
	vec_doubles.append(0)
	vec_doubles.append(1)
	assert str(vec_doubles) == "0 1"
	vec_doubles[0] = -1
	assert str(vec_doubles) == "-1 1"

def test_std_vector_vector_doubles():
	vec_doubles_0 = prx.vector_of_doubles();
	vec_doubles_1 = prx.vector_of_doubles();
	vec_doubles_0.append(0)
	vec_doubles_0.append(1)
	vec_doubles_1.append(2)
	vec_doubles_1.append(3)
	vec_vec_doubles = prx.vector_of_vector_of_doubles();
	vec_vec_doubles.append(vec_doubles_0)
	vec_vec_doubles.append(vec_doubles_1)
	expected_str = "0 1\n2 3"
	assert str(vec_vec_doubles) == expected_str, print("Expected: ", expected_str, "Got: ",vec_vec_doubles) 
if __name__ == "__main__":
  test_std_vector_doubles();
  test_std_vector_vector_doubles();