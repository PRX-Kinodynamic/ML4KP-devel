#define BOOST_AUTO_TEST_MAIN racecar_mini_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plants/racecar_mini.hpp"

struct car_test : prx::racecar_mini_t
{
  car_test(const std::string& path) : prx::racecar_mini_t(path)
  {
  }
  Eigen::Matrix3d mass_matrix()
  {
    M();
    return _M;
  }
};

BOOST_AUTO_TEST_CASE(racecar_mini_mass_matrix_updates_correctly)
{
  car_test car("car");
  car.get_parameter_space()->copy_from({ 1, 3 });

  const Eigen::Matrix3d expected_matrix(Eigen::DiagonalMatrix<double, 3>(1, 1, 3));

  Eigen::Matrix3d mass_matrix{ car.mass_matrix() };

  BOOST_CHECK_MESSAGE(mass_matrix == expected_matrix, "Got: \n" << mass_matrix << "\n Expected: \n" << expected_matrix);
}