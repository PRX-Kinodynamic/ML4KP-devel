#define BOOST_AUTO_TEST_MAIN care_test
#include <string>
#include <gtsam/geometry/Rot2.h>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/math/multivariate_gaussian_distribution.hpp"

BOOST_AUTO_TEST_CASE(multivariate_gaussian_test)
{
  prx::multivariate_gaussian_t<1> gaussian;
  using Sample = prx::multivariate_gaussian_t<1>::Sample;

  int in_66{ 0 };
  for (int i = 0; i < 1000; ++i)
  {
    const Sample mu{ gaussian() };
    // TODO: This is too simple and only upper bounds... Need a more statistical test
    in_66 += std::fabs(mu[0]) < 0.67 ? 1 : 0;
  }
  BOOST_CHECK(in_66 < 667);
}

BOOST_AUTO_TEST_CASE(lie_multivariate_gaussian_test)
{
  using Cov = prx::lie_group_gaussian_noise_t<gtsam::Rot2>::Covariance;
  Cov cov;
  cov << 0.1;
  prx::lie_group_gaussian_noise_t<gtsam::Rot2> lie_gaussian(cov);

  int sample_66{ 0 };
  for (int i = 0; i < 1000; ++i)
  {
    gtsam::Rot2 rot;
    const gtsam::Rot2 mu{ lie_gaussian(rot) };
    std::cout << "rot: " << rot.theta() << " ";
    std::cout << "mu: " << mu.theta() << "\n";

    // sample_66 += std::fabs(mu.theta()) < 0.0667 ? 1 : 0;
  }
  PRX_DBG_VARS(sample_66)
}
