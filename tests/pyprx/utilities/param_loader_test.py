import os
import math
import pytest
import libpyDirtMP as prx

def test_param_loader_loads_from_file():

  params = prx.param_loader();
  params.set_input_path(prx.lib_path + "/tests/utilities/");
  params.add_file("param_loader_test.yaml");

  assert params != None

def test_param_loader_check_existance_correctly():

  params = prx.param_loader();
  params.set_input_path(prx.lib_path + "/tests/utilities/");
  params.add_file("param_loader_test.yaml");

  assert params.exists("str_test")
  print(params.exists("garbage"))
  assert not params.exists("garbage")

def test_param_loader_converts_correctly():

  params = prx.param_loader();
  params.set_input_path(prx.lib_path + "/tests/utilities/");
  params.add_file("param_loader_test.yaml");

  str_param = str(params["str_test"])
  int_param = int(params["int_test"])
  float_param = float(params["dbl_test"])
  bool_param = bool(params["dbl_test"])
  vec_param = params["vec_test"].as_float_vector()

  assert isinstance(str_param, str) and  str_param == "dirtmp"
  assert isinstance(int_param, int) and  int_param == 231192
  assert isinstance(float_param, float) and  float_param == 3.1415926535897932385
  assert isinstance(bool_param, bool) and  bool_param == True
  assert len(vec_param) == 5 
  assert isinstance(vec_param[0], float) and vec_param[0] == 0 
  assert isinstance(vec_param[1], float) and vec_param[1] == 1 
  assert isinstance(vec_param[2], float) and vec_param[2] == 2 
  assert isinstance(vec_param[3], float) and vec_param[3] == 3 
  assert isinstance(vec_param[4], float) and vec_param[4] == 4 

if __name__ == "__main__":
  param_loader_loads_from_file_test()