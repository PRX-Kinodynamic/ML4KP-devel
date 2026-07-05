#pragma once
// #include "prx/utilities/defs.hpp"
// #include "prx/utilities/general/debug_utils.hpp"
#include <type_traits>
#include "prx/utilities/spaces/product_lie_group.hpp"
#include "prx/utilities/general/template_utils.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace prx
{

template <typename T, typename = void>  // primary template
struct stream_specialization : std::false_type
{
};

template <typename T, typename = void>  // primary template
struct streamer_t                       //: std::false_type     //: std::false_type
{
  // static void to_stream(std::ostream& os, const T& t)
  // {
  //   os << t << " ";
  // }
};

// template <typename T>  // primary template
// struct streamer_t<T, std::enable_if_t<std::is_arithmetic_v<T>>> : std::true_type
// {
//   static void to_stream(std::ostream& os, const T& t)
//   {
//     os << t << " ";
//   }
// };

// template <int Dim>  // primary template
// struct stream_specialization<Eigen::Vector<double, Dim>> : std::true_type
// {
// };

template <int Row, int Col>  // primary template
struct stream_specialization<Eigen::Matrix<double, Row, Col>> : std::true_type
{
};

// template <int Dim>
template <int Row, int Col>
struct streamer_t<Eigen::Matrix<double, Row, Col>> : std::true_type
{
public:
  // using Vector = Eigen::Vector<double, Row>;
  // using Matrix = Eigen::Matrix<double, Row, Col>;

  template <typename Vector, std::enable_if_t<Vector::ColsAtCompileTime == 1, bool> = true>
  static void to_stream(std::ostream& os, const Vector& v)
  {
    os << v.transpose() << " ";
  }

  template <typename Matrix, std::enable_if_t<Matrix::ColsAtCompileTime != 1, bool> = true>
  static void to_stream(std::ostream& os, const Matrix& m)
  {
    os << m << " ";
  }
};

template <>  // primary template
struct stream_specialization<gtsam::Rot2> : std::true_type
{
};

template <>  // explicit specialization for T = void
struct streamer_t<gtsam::Rot2> : std::true_type
{
public:
  static void to_stream(std::ostream& os, const gtsam::Rot2& v)
  {
    os << v.theta() << " ";
  }
};

template <>  // primary template
struct stream_specialization<gtsam::Pose2> : std::true_type
{
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
struct stream_specialization<gtsam::ProductLieGroup<G, H>> : std::true_type
{
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

template <typename G, typename H>
struct stream_specialization<gtsam::ProductLieGroupV43<G, H>> : std::true_type
{
};

template <typename G, typename H>
struct streamer_t<gtsam::ProductLieGroupV43<G, H>> : std::true_type
{
public:
  using Element = gtsam::ProductLieGroupV43<G, H>;

  static void to_stream(std::ostream& os, const Element& v)
  {
    streamer_t<G>::to_stream(os, v.first);
    streamer_t<H>::to_stream(os, v.second);
  }
};

template <typename G, typename H>
struct stream_specialization<std::pair<G, H>> : std::true_type
{
};

template <typename G, typename H>
struct streamer_t<std::pair<G, H>> : std::true_type
{
public:
  using Element = std::pair<G, H>;

  static void to_stream(std::ostream& os, const Element& v)
  {
    streamer_t<G>::to_stream(os, v.first);
    streamer_t<H>::to_stream(os, v.second);
  }
};

// template <>
// struct stream_specialization<std::string> : std::true_type
// {
// };

template <typename Element>
struct streamer_t<Element,                                               // no-lint
                  std::enable_if_t<                                      // no-lint
                      prx::utilities::is_streamable<Element>::value and  // no-lint
                      (not stream_specialization<Element>::value)>       // no-lint
                  >
{
  static void to_stream(std::ostream& os, const Element& t)
  {
    os << t << " ";
  }
};

template <typename Element>
struct streamer_t<Element,                                                     // no-lint
                  std::enable_if_t<                                            // no-lint
                      prx::utilities::is_iterable<Element>::value and          // no-lint
                      (not prx::utilities::is_streamable<Element>::value) and  // no-lint
                      (not stream_specialization<Element>::value)>             // no-lint
                  >
{
  static void to_stream(std::ostream& os, const Element& element)
  {
    for (auto&& e : element)
    {
      using CurrentType = std::remove_cv_t<std::remove_reference_t<decltype(e)>>;
      streamer_t<CurrentType>::to_stream(os, e);
      os << "\n";
    }
  }
};

template <typename Type>
void to_stream(std::ostream& os, const Type& type)
{
  streamer_t<Type>::to_stream(os, type);
}
}  // namespace prx