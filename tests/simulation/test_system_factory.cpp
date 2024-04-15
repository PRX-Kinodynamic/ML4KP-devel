#define BOOST_AUTO_TEST_MAIN system_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/plants/plants.hpp"

const std::vector<std::string> expected_system_names{ "2D_Point",    "rally_car",    "treaded_vehicle",
                                                      "3D_Point",    "Acrobot",      "fixed_wing",
                                                      "koules",      "racecar_mini", "FO_treaded_vehicle",
                                                      "Ackermann_FO" };

const std::vector<std::pair<std::string, std::string>> expected_fn_names{ { "2D_Point", "2D_Point" },
                                                                          { "rally_car", "rally_car" },
                                                                          { "treaded_vehicle", "treaded_vehicle" },
                                                                          { "FO_treaded_vehicle",
                                                                            "FO_treaded_vehicle" } };

namespace mock
{
struct dummy_t : public prx::plant_t
{
  dummy_t(const std::string& path) : plant_t(path){};
  virtual ~dummy_t(){};
  virtual void propagate(const double simulation_step) override final{};
  virtual void update_configuration() override{};

  virtual void compute_derivative() override final{};
};

}  // namespace mock
PRX_REGISTER_SYSTEM(mock::dummy_t, mock_dummy)

BOOST_AUTO_TEST_CASE(test_mock_dummy_is_registred)
{
  const std::string name{ "mock_dummy" };
  auto available_systems = prx::system_factory_t::available_systems();
  BOOST_CHECK(std::find(available_systems.begin(), available_systems.end(), name) != available_systems.end());
}

BOOST_AUTO_TEST_CASE(test_mock_dummy_create_system_as)
{
  const std::string name{ "mock_dummy" };
  std::shared_ptr<mock::dummy_t> sys_ptr{ prx::system_factory_t::create_system_as<mock::dummy_t>(name, name) };
  BOOST_CHECK(sys_ptr != nullptr);
  BOOST_CHECK(sys_ptr->get_pathname() == name);
}

BOOST_AUTO_TEST_CASE(test_available_systems_as_expected)
{
  auto available_systems = prx::system_factory_t::available_systems();
  for (auto name : expected_system_names)
  {
    printf("\tChecking available system: %s...", name.c_str());
    BOOST_CHECK(std::find(available_systems.begin(), available_systems.end(), name) != available_systems.end());
    printf("\t[ OK ]\n");
  }
}

BOOST_AUTO_TEST_CASE(testing_system_factory_builds_systems_correctly)
{
  printf("Checking systems in Factory...\n");
  for (auto name : expected_system_names)
  {
    printf("\tChecking systems: %s...", name.c_str());
    std::string path = name + "_path";
    auto sys_ptr = prx::system_factory_t::create_system(name, path);
    BOOST_CHECK(sys_ptr != nullptr);
    BOOST_CHECK(sys_ptr->get_pathname() == path);
    printf("\t[ OK ]\n");
  }
}
BOOST_AUTO_TEST_CASE(testing_system_factory_builds_functions_correctly)
{
  printf("\nChecking functions in Factory...\n");
  for (auto p : expected_fn_names)
  {
    printf("\tChecking function: %s...", p.first.c_str());
    BOOST_CHECK(prx::system_factory_t::get_system_max_velocity(
                    p.first, prx::system_factory_t::create_system(p.second, p.second)) <
                std::numeric_limits<double>::infinity());
    printf("\t[ OK ]\n");
  }
}