#include "general/debug_utils.hpp"
#include "general/param_loader.hpp"
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

BOOST_AUTO_TEST_CASE(param_loader_from_string)
{
  prx::param_loader pl{};
  const std::string input =  // no-lint
      "p0: \"zero\"\n"       // no-lint
      "p1: 1\n"              // no-lint
      "p_multi:\n"           // no-lint
      "    zero: 0\n"        // no-lint
      "    vec: [1,2]\n"     // no-lint
      ;                      // no-lint
  // pl.from_string("p0: zero\np1: 1\np_multi:\n\tzero: 0\n\tone: 1\n");
  pl.from_string(input);
  // PRX_DBG_VARS(pl)
  BOOST_CHECK(pl["p0"].as<std::string>() == "zero");
  BOOST_CHECK(pl["p1"].as<int>() == 1);
  BOOST_CHECK(pl["p_multi/zero"].as<int>() == 0);
  BOOST_CHECK(pl["p_multi/vec"].as<std::vector<int>>()[0] == 1);
  BOOST_CHECK(pl["p_multi/vec"].as<std::vector<int>>()[1] == 2);
}

BOOST_AUTO_TEST_CASE(param_loader_merge)
{
  prx::param_loader pl0{};
  prx::param_loader pl1{};

  const std::string t0{ "test_str_0" };
  const int t1{ 1 };
  const std::string t2{ "test_str_2" };
  const double t3{ 3.14 };
  pl0["test0"].set(t0);
  pl0["test1"].set(t1);
  pl1["test2"].set(t2);
  pl1["test3"].set(t3);

  pl0.merge(pl1);

  BOOST_CHECK(pl0["test0"].as<std::string>() == t0);
  BOOST_CHECK(pl0["test1"].as<int>() == t1);
  BOOST_CHECK(pl0["test2"].as<std::string>() == t2);
  BOOST_CHECK(pl0["test3"].as<double>() == t3);
  // BOOST_CHECK(pl["p1"].as<int>() == 1);
  // BOOST_CHECK(pl["p_multi/zero"].as<int>() == 0);
  // BOOST_CHECK(pl["p_multi/vec"].as<std::vector<int>>()[0] == 1);
  // BOOST_CHECK(pl["p_multi/vec"].as<std::vector<int>>()[1] == 2);
}

BOOST_AUTO_TEST_CASE(param_loader_replace_env_vars)
{
  prx::param_loader pl0{};

  const std::string t0{ "${DIRTMP_PATH}" };
  pl0["test0"].set(t0);

  // PRX_DBG_VARS(pl0)
  pl0.replace_environment_variables();
  // PRX_DBG_VARS(pl0)
  // pl0.merge(pl1);

  BOOST_CHECK(pl0["test0"].as<std::string>() != t0);
  // BOOST_CHECK(pl0["test1"].as<int>() == t1);
  // BOOST_CHECK(pl0["test2"].as<std::string>() == t2);
  // BOOST_CHECK(pl0["test3"].as<double>() == t3);
}

BOOST_AUTO_TEST_CASE(param_loader_list_array)
{
  std::string yaml =
      "bounds:\n"
      "  - \n"
      "    min: [0.0, 0.0]\n"
      "    max: [1.0, 1.0]\n"
      "  - \n"
      "    min: [1.0, 1.0]\n"
      "    max: [2.0, 2.0]\n"
      "  - \n"
      "    min: [2.0, 2.0]\n"
      "    max: [3.0, 3.0]\n";

  prx::param_loader pl0;
  pl0.from_string(yaml);

  prx::param_loader pl1(pl0["bounds"].begin(), pl0["bounds"].end());
  // auto iter = pl0["bounds"].begin();
  // // auto second = *(pl0["bounds"].begin()++);
  PRX_DBG_VARS(pl1)
  // iter++;
  // PRX_DBG_VARS(*iter)
  // pl0.print();
  // BOOST_CHECK(pl0["test0"].as<std::string>() != t0);
}