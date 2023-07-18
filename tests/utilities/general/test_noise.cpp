#define BOOST_AUTO_TEST_MAIN noise_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/noise.hpp"
// TODO: Implement a correct test, possibly using this:
//		 https://www.boost.org/doc/libs/1_76_0/libs/math/doc/html/math_toolkit/dist_ref/dists/kolmogorov_smirnov_dist.html
BOOST_AUTO_TEST_CASE(noise_test)
{
  double x, y, z;
  x = y = z = 0;
  std::vector<double*> state_memory = { &x, &y, &z };
  prx::space_t* state_space = new prx::space_t("EEE", state_memory, "XYZ");
  state_space->set_bounds(
      { -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max() },
      { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() });

  auto pt = state_space->make_point();

  std::vector<double> v = { 0, -10, 10 };
  state_space->copy_point_from_vector(pt, v);

  // Using Normal(0,1)
  // prx::gaussian_noise_t g_noise;

  // std::cout << "Original point: " << pt << std::endl;
  // for (int i = 0; i < 10000; ++i)
  // {
  // 	state_space -> copy_point_from_vector(pt, v);
  // 	g_noise.add_noise(pt);
  // 	std::cout << pt << std::endl;
  // }

  // Using Normal(0,0.1)
  // prx::gaussian_noise_t g_noise_2(0,0.1);
  prx::uniform_noise_t g_noise_2(-1, 1);

  std::cout << "\nOriginal point: " << pt << std::endl;
  for (int i = 0; i < 10000; ++i)
  {
    state_space->copy_point_from_vector(pt, v);
    g_noise_2.add_noise(pt);
    std::cout << pt << std::endl;
  }
}