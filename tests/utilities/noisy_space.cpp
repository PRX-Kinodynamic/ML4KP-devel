#define BOOST_TEST_MODULE noisy_space_test
#include <boost/test/included/unit_test.hpp>
#include <string>
#include "prx/utilities/general/random.hpp"
#include "prx/utilities/spaces/noisy_space.hpp"

BOOST_AUTO_TEST_CASE(noisy_space_test)
{
  double x, y, z;
  x = y = z = 1;
  std::vector<double*> address_1 = { &x, &y, &z };
  prx::space_t* space_1 = new prx::space_t("EEE", address_1, "space_1");
  prx::space_point_t pt_sp1 = space_1->make_point();
  prx::space_point_t pt_ns1 = space_1->make_point();

  space_1->copy_to_point(pt_sp1);

  auto noisy_space_1 = new prx::noisy_space_t<prx::uniform_noise_t>(space_1, -0.5, 0.5);

  // std::cout << "point: " << pt_sp1 << std::endl;
  // std::cout << "noisy point: " << pt_ns1 << std::endl;

  for (int i = 0; i < 1000; ++i)
  {
    noisy_space_1->copy_to_point(pt_ns1);
    for (int j = 0; j < space_1->get_dimension(); ++j)
    {
      BOOST_CHECK(std::fabs((*pt_sp1)[j] - (*pt_ns1)[j]) <= 0.5);
    }
  }
  printf("%s:\t[ OK ]\n", __PRETTY_FUNCTION__);
}