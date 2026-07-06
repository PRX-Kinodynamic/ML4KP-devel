#include <gtsam/geometry/Pose2.h>
#include "general/param_loader.hpp"
#define BOOST_AUTO_TEST_MAIN sampler_test

#include <chrono>
#include <string>

#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/sampler.hpp"

BOOST_AUTO_TEST_CASE(test_eigen_vector_sampler)
{
  prx::sampler_t<Eigen::Vector2d> sampler_vec2d;
  prx::sampler_t<Eigen::Vector3d> sampler_vec3d(Eigen::Vector3d(0, 0.5, -2.0), Eigen::Vector3d(1.0, 0.75, 0.0));

  prx::param_loader param_loader;
  param_loader["min"].set(Eigen::Vector<double, 1>(1));
  param_loader["max"].set(Eigen::Vector<double, 1>(2));

  prx::sampler_t<Eigen::Vector<double, 1>> sampler_vec1d(param_loader);

  for (int i = 0; i < 100; ++i)
  {
    const Eigen::Vector<double, 1> vec1{ sampler_vec1d() };
    BOOST_REQUIRE_MESSAGE(1.0 < vec1[0] and vec1[0] < 2.0, vec1[0]);

    const Eigen::Vector2d vec2{ sampler_vec2d() };

    BOOST_REQUIRE_MESSAGE(-1.0 < vec2[0] and vec2[0] < 1.0, vec2[0]);
    BOOST_REQUIRE_MESSAGE(-1.0 < vec2[1] and vec2[1] < 1.0, vec2[1]);

    const Eigen::Vector3d vec3{ sampler_vec3d() };
    BOOST_REQUIRE_MESSAGE(0.0 < vec3[0] and vec3[0] < 1.0, vec3[0]);
    BOOST_REQUIRE_MESSAGE(0.5 < vec3[1] and vec3[1] < 0.75, vec3[1]);
    BOOST_REQUIRE_MESSAGE(-2.0 < vec3[2] and vec3[2] < 0.0, vec3[2]);
  }
}

BOOST_AUTO_TEST_CASE(test_gtsam_Pose2_sampler)
{
  prx::sampler_t<gtsam::Pose2> sampler0;
  prx::sampler_t<gtsam::Pose2> sampler1({ -10, -1, -prx::constants::pi }, { 5, 10, prx::constants::pi });

  for (int i = 0; i < 100; ++i)
  {
    const gtsam::Pose2 p0{ sampler0() };
    const gtsam::Pose2 p1{ sampler1() };

    BOOST_REQUIRE_MESSAGE(-1.0 < p0.x() and p0.x() < 1.0, p0.x());
    BOOST_REQUIRE_MESSAGE(-1.0 < p0.y() and p0.y() < 1.0, p0.y());
    BOOST_REQUIRE_MESSAGE(-prx::constants::pi < p0.theta() and p0.theta() < prx::constants::pi, p0.theta());

    BOOST_REQUIRE_MESSAGE(-1.0 < p1.x() and p1.x() < 1.0, p1.x());
    BOOST_REQUIRE_MESSAGE(-1.0 < p1.y() and p1.y() < 1.0, p1.y());
    BOOST_REQUIRE_MESSAGE(-prx::constants::pi < p1.theta() and p1.theta() < prx::constants::pi, p1.theta());
  }
}

BOOST_AUTO_TEST_CASE(test_gtsam_product_lie_group_sampler)
{
  using ProductLie = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
  prx::sampler_t<ProductLie> sampler0;

  for (int i = 0; i < 100; ++i)
  {
    const ProductLie p0{ sampler0() };

    BOOST_REQUIRE_MESSAGE(-1.0 < p0.first.x() and p0.first.x() < 1.0, p0.first.x());
    BOOST_REQUIRE_MESSAGE(-1.0 < p0.first.y() and p0.first.y() < 1.0, p0.first.y());
    BOOST_REQUIRE_MESSAGE(-prx::constants::pi < p0.first.theta() and p0.first.theta() < prx::constants::pi,
                          p0.first.theta());

    BOOST_REQUIRE_MESSAGE(-1. < p0.second[0] and p0.second[0] < 1.0, p0.second[0]);
    BOOST_REQUIRE_MESSAGE(-1. < p0.second[1] and p0.second[1] < 1.0, p0.second[1]);
    BOOST_REQUIRE_MESSAGE(-1. < p0.second[2] and p0.second[2] < 1.0, p0.second[2]);
  }
}

BOOST_AUTO_TEST_CASE(test_gtsam_product_lie_group_sampler_from_params)
{
  using ProductLie = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
  prx::param_loader params;
  std::string yaml =
      "bounds:\n"
      "  -\n"
      "    min: [-10, -10, -3.14159]\n"
      "    max: [+10, +10, +3.14159]\n"
      "  -\n"
      "    min: [-0.5, -0.5, -0.1]\n"
      "    max: [+0.5, +0.5, +0.1]\n";
  params.from_string(yaml);
  prx::sampler_t<ProductLie> sampler0(params["bounds"]);

  for (int i = 0; i < 1000; ++i)
  {
    const ProductLie p0{ sampler0() };

    BOOST_REQUIRE_MESSAGE(-10. < p0.first.x() and p0.first.x() < 10., p0.first.x());
    BOOST_REQUIRE_MESSAGE(-10. < p0.first.y() and p0.first.y() < 10., p0.first.y());
    BOOST_REQUIRE_MESSAGE(-prx::constants::pi < p0.first.theta() and p0.first.theta() < prx::constants::pi,
                          p0.first.theta());

    BOOST_REQUIRE_MESSAGE(-0.5 < p0.second[0] and p0.second[0] < 0.5, p0.second[0]);
    BOOST_REQUIRE_MESSAGE(-0.5 < p0.second[1] and p0.second[1] < 0.5, p0.second[1]);
    BOOST_REQUIRE_MESSAGE(-0.1 < p0.second[2] and p0.second[2] < 0.1, p0.second[2]);
  }
}

BOOST_AUTO_TEST_CASE(test_std_vector_sampler)
{
  const std::vector<double> bound_min = { -1.0, 0.0, 1.0 };
  const std::vector<double> bound_max = { +1.0, 1.0, 2.0 };
  prx::sampler_t<std::vector<double>> sampler(bound_min, bound_max);

  for (int i = 0; i < 1000; ++i)
  {
    std::vector<double> sample{ sampler() };
    BOOST_REQUIRE(sample.size() == bound_max.size());
    BOOST_REQUIRE(-1.0 < sample[0] and sample[0] <= 1.0);
    BOOST_REQUIRE(+0.0 < sample[1] and sample[1] <= 1.0);
    BOOST_REQUIRE(+1.0 < sample[2] and sample[2] <= 2.0);
  }
}

BOOST_AUTO_TEST_CASE(test_Rot2_sampler)
{
  prx::param_loader pl;
  const double _pi{ prx::constants::pi };
  prx::sampler_t<gtsam::Rot2> s0{};
  prx::sampler_t<gtsam::Rot2> s1(-1., 1.);
  prx::sampler_t<gtsam::Rot2> s2(pl);
  pl["min"].set(0.1);
  pl["max"].set(0.2);
  prx::sampler_t<gtsam::Rot2> s3(pl);
  for (int i = 0; i < 1000; ++i)
  {
    gtsam::Rot2 sample0{ s0() };
    gtsam::Rot2 sample1{ s1() };
    gtsam::Rot2 sample2{ s2() };
    gtsam::Rot2 sample3{ s3() };

    BOOST_REQUIRE(-_pi <= sample0.theta() and sample0.theta() <= _pi);
    BOOST_REQUIRE(-1. <= sample1.theta() and sample1.theta() <= 1.);
    BOOST_REQUIRE(-_pi <= sample2.theta() and sample2.theta() <= _pi);
    BOOST_REQUIRE(0.1 <= sample3.theta() and sample3.theta() <= 0.2);
  }
}

BOOST_AUTO_TEST_CASE(test_Rot2Double_sampler)
{
  using Type = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  const double _pi{ prx::constants::pi };
  prx::sampler_t<Type> s0{};
  // for (int i = 0; i < 1000; ++i)
  // {
  Type sample0{ s0() };

  BOOST_REQUIRE(-_pi <= sample0.first.theta() and sample0.first.theta() <= _pi);
  // }
}
