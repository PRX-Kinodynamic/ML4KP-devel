#define BOOST_AUTO_TEST_MAIN param_loader_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"

BOOST_AUTO_TEST_CASE(param_loader_test)
{
  auto params = prx::param_loader();
  params.set_input_path(prx::lib_path + "/tests/utilities/general/");
  params.add_file("test_param_loader.yaml");

  std::vector<std::string> names = { "str_test", "int_test", "dbl_test", "bool_test", "vec_test" };
  // char** args = (char**) malloc(names.size() * sizeof(char**));
  char** args = new char*[names.size()];

  int argc_ = 0;
  args[argc_++] = (char*)"--str_test=dirtmp";
  args[argc_++] = (char*)"--int_test=231192";
  // args[argc_++] = (char*) "--dbl_test=3.1415926535897932385";
  args[argc_++] = (char*)"--bool_test";
  args[argc_++] = (char*)"--vec_test=[0,1,2,3,4]";
  // char** args = args_n;
  auto params_2 = prx::param_loader(argc_, args);
  params_2["dbl_test"].set(3.1415926535897932385);
  params_2.print();
  int i_n = 0;
  std::cout << "Checking: " << names[i_n++] << "\t";
  BOOST_CHECK(params.exists(names[0]));
  std::cout << "[ OK ] ";

  BOOST_CHECK(params[names[0]].as<std::string>() == params[names[0]].as<std::string>());
  std::cout << "[ OK ] ";
  BOOST_CHECK(params_2[names[0]].as<>() == params_2[names[0]].as<>());
  std::cout << "[ OK ] ";
  BOOST_CHECK(params[names[0]].as<std::string>() == params_2[names[0]].as<>());
  // BOOST_CHECK(params[names[0]].as<std::string>() == params_2[names[0]].as<>());
  std::cout << "[ OK ] " << std::endl;
  std::cout << "Checking: " << names[i_n++] << "\t";
  BOOST_CHECK(params[names[1]].as<int>() == params_2[names[1]].as<int>());
  std::cout << "[ OK ] " << std::endl;
  std::cout << "Checking: " << names[i_n++] << "\t";
  BOOST_CHECK(params[names[2]].as<double>() == PRX_PI);
  BOOST_CHECK(params_2[names[2]].as<double>() == PRX_PI);
  std::cout << "[ OK ] " << std::endl;
  std::cout << "Checking: " << names[i_n++] << "\t";
  BOOST_CHECK(params[names[3]].as<bool>() == params_2[names[3]].as<bool>());
  std::cout << "[ OK ] " << std::endl;
  std::cout << "Checking: " << names[i_n++] << "\t";
  auto v = params[names[4]].as<std::vector<int>>();
  auto v2 = params_2[names[4]].as<std::vector<int>>();
  for (int i = 0; i < v.size(); ++i)
  {
    BOOST_CHECK(v[i] == v2[i]);
  }
  std::cout << "[ OK ] " << std::endl;
  // TODO: add checks for nested params
}

BOOST_AUTO_TEST_CASE(param_loader_save_load_test)
{
  prx::param_loader params;
  params["a"].set(1);
  params["b"].set("B");
  params["c"].set(std::vector<double>{ 0.0, 1.0 });

  const std::string filename{ "/tmp/param_loader_save_load_test" };
  params.save(filename);
  prx::param_loader params_in(filename);

  BOOST_CHECK(params["a"].as<int>() == params_in["a"].as<int>());
  BOOST_CHECK(params["b"].as<>() == params_in["b"].as<>());

  const std::vector<double> c_original{ params["c"].as<std::vector<double>>() };
  const std::vector<double> c_from_file{ params_in["c"].as<std::vector<double>>() };

  BOOST_CHECK(c_original.size() == 2);
  BOOST_CHECK(c_from_file.size() == 2);
  BOOST_CHECK(c_original[0] == c_from_file[0]);
  BOOST_CHECK(c_original[1] == c_from_file[1]);
}