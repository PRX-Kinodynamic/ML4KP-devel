#pragma once

/**
 * @file gtsam_bridge.hpp
 * @author Edgar Granados
 * @brief Populate gtsam types from yaml
 * */

#include <yaml-cpp/yaml.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/ProductLieGroup.h>
#include "prx/utilities/general/transforms.hpp"
// Eigen::Matrix<double, 3, 1>
// Conversions to/from YAML
namespace YAML
{
// struct convert<Eigen::Matrix<Scalar,  Rows,  Cols,  Align,  RowsAtCompileTime,  ColsAtCompileTime> >{
// template <typename Type, Eigen::Index N>
template <>
struct convert<gtsam::Rot2>
{
  using Type = gtsam::Rot2;
  static Node encode(const Type& rhs)
  {
    Node node;
    node["theta"] = rhs.theta();
    return node;
  }

  static bool decode(const Node& node, Type& lhs)
  {
    lhs = Type(node["theta"].as<double>());
    return true;
  }
};

template <>
struct convert<gtsam::Pose2>
{
  using Type = gtsam::Pose2;
  static Node encode(const Type& rhs)
  {
    Node node;
    node["x"] = rhs.x();
    node["y"] = rhs.y();
    node["theta"] = rhs.theta();
    return node;
  }

  static bool decode(const Node& node, Type& lhs)
  {
    const double x{ node["x"].as<double>() };
    const double y{ node["y"].as<double>() };
    const double theta{ node["theta"].as<double>() };
    lhs = Type(x, y, theta);
    return true;
  }
};

template <>
struct convert<gtsam::Rot3>
{
  using Type = gtsam::Rot3;
  static Node encode(const Type& rhs)
  {
    Node node;
    Eigen::Matrix3d mat{ rhs.matrix() };
    // for (int i = 0; i < rhs.size(); ++i)
    // {
    node.push_back(mat(1, 1));
    node.push_back(mat(1, 2));
    node.push_back(mat(1, 3));

    node.push_back(mat(2, 1));
    node.push_back(mat(2, 2));
    node.push_back(mat(2, 3));

    node.push_back(mat(3, 1));
    node.push_back(mat(3, 2));
    node.push_back(mat(3, 3));
    // }
    // double R11, double R12, double R13,
    //     double R21, double R22, double R23,
    //     double R31, double R32, double R33
    return node;
  }

  static bool decode(const Node& node, Type& lhs)
  {
    if (node.size() == 4)
    {
      const double w{ node["w"].as<double>() };
      const double x{ node["x"].as<double>() };
      const double y{ node["y"].as<double>() };
      const double z{ node["z"].as<double>() };
      lhs = Type(w, x, y, z);
      return true;
    }
    else if (node.size() == 9)
    {
      const double R11{ node[0].as<double>() };
      const double R12{ node[1].as<double>() };
      const double R13{ node[2].as<double>() };
      const double R21{ node[3].as<double>() };
      const double R22{ node[4].as<double>() };
      const double R23{ node[5].as<double>() };
      const double R31{ node[6].as<double>() };
      const double R32{ node[7].as<double>() };
      const double R33{ node[8].as<double>() };
      lhs = Type(R11, R12, R13, R21, R22, R23, R31, R32, R33);
      return true;
    }
    return false;
  }
};

template <>
struct convert<gtsam::Pose3>
{
  using Type = gtsam::Pose3;
  static Node encode(const Type& rhs)
  {
    Node node;
    node["rotation"] = convert<gtsam::Rot3>::encode(rhs.rotation());
    node["translation"] = convert<Eigen::Vector3d>::encode(rhs.translation());
    return node;
  }

  static bool decode(const Node& node, Type& lhs)
  {
    const gtsam::Rot3 r{ node["rotation"].as<gtsam::Rot3>() };
    const Eigen::Vector3d t{ node["translation"].as<Eigen::Vector3d>() };

    lhs = Type(r, t);
    return true;
  }
};

template <typename G, typename H>
struct convert<gtsam::ProductLieGroup<G, H>>
{
  using Type = gtsam::ProductLieGroup<G, H>;
  static Node encode(const Type& rhs)
  {
    Node node;
    node["first"] = convert<G>::encode(rhs.first);
    node["second"] = convert<H>::encode(rhs.second);
    return node;
  }

  static bool decode(const Node& node, Type& lhs)
  {
    const G g{ node["first"].as<G>() };
    const H t{ node["second"].as<H>() };

    lhs = Type(g, t);
    return true;
  }
};
}  // namespace YAML