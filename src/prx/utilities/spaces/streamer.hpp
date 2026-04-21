#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/debug_utils.hpp"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/ProductLieGroup.h>
#include <type_traits>

namespace prx
{

template <typename T, typename = void>  // primary template
struct streamer_t : std::false_type     //: std::false_type
{
  // static void to_stream(std::ostream& os, const T& t)
  // {
  //   os << t << " ";
  // }
};

template <typename T>  // primary template
struct streamer_t<T, std::enable_if_t<std::is_arithmetic_v<T>>> : std::true_type
{
  static void to_stream(std::ostream& os, const T& t)
  {
    os << t << " ";
  }
};

template <int Dim>  // primary template
struct streamer_t<Eigen::Vector<double, Dim>> : std::true_type
{
public:
  using Vector = Eigen::Vector<double, Dim>;

  static void to_stream(std::ostream& os, const Vector& v)
  {
    os << v.transpose() << " ";
  }
};

template <>  // explicit specialization for T = void
struct streamer_t<gtsam::Pose2> : std::true_type
{
public:
  static void to_stream(std::ostream& os, const gtsam::Pose2& v)
  {
    os << v.x() << " ";
    os << v.y() << " ";
    os << v.theta() << " ";
  }
};

template <typename G, typename H>
struct streamer_t<gtsam::ProductLieGroup<G, H>> : std::true_type
{
public:
  using Element = gtsam::ProductLieGroup<G, H>;

  static void to_stream(std::ostream& os, const Element& v)
  {
    streamer_t<G>::to_stream(os, v.first);
    streamer_t<H>::to_stream(os, v.second);
  }
};

}  // namespace prx