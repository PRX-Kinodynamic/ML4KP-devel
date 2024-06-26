#pragma once

#include "prx/utilities/defs.hpp"

#include <map>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>

#define PRX_PI 3.1415926535897932385
#define PRX_EPSILON 1e-7
#define PRX_INFINITY 1e10

namespace prx
{
namespace constants
{
const double pi{ PRX_PI };
const double epsilon{ PRX_EPSILON };
constexpr double infinity{ std::numeric_limits<double>::infinity() };

extern int precision;
extern char separating_value;

namespace color
{

constexpr std::string_view normal = "\033[0m";
constexpr std::string_view red{ "\033[31m" };
constexpr std::string_view green{ "\033[32m" };
constexpr std::string_view yellow{ "\033[33m" };
}  // namespace color
}  // namespace constants

static inline std::string lib_path_safe(std::string env_var)
{
  char* path = std::getenv(env_var.c_str());
  if (path == NULL)
  {
    std::cout << env_var << " environmental variable not set." << std::endl;
    exit(1);
  }

  auto p_str = std::string(path);
  return (p_str + (p_str.back() == '/' ? "" : "/"));
}

template <typename T, typename S>
static inline bool are_approx_equal(T n1, S n2, double tolerance = PRX_EPSILON)
{
  return std::fabs(n1 - n2) <= tolerance;
}

// TODO: Make this for any sequence container
template <typename T, typename S>
static inline bool are_approx_equal(std::vector<T> c1, std::vector<S> c2, double tolerance = PRX_EPSILON)
{
  prx_assert(c1.size() == c2.size(), "Containers must have the same size.");

  for (int i = 0; i < c1.size(); ++i)
  {
    if (!are_approx_equal(c1[i], c2[i], tolerance))
      return false;
  }
  return true;
}

const std::string lib_path = lib_path_safe("DIRTMP_PATH");
const std::string models_path = lib_path + "resources/models/";
const std::string obj_models_path = lib_path + "resources/models/obj/";
const std::string input_path = lib_path + "resources/input_files/";
const std::string js_path = lib_path + "resources/js/";
const std::string out_path = lib_path + "out/";

// mujoco
const bool MUJOCO_VIS = true;
const std::string mj_models_path = lib_path + "resources/models/";

enum plant_type
{
  ANALYTICAL,
  MUJOCO
};

static inline double norm_angle_pi(double angle, double min_angle = -PRX_PI, double max_angle = PRX_PI)
{
  // prx_warn_cond(std::fabs(angle) < 100 * max_angle, "Angle might be too high: " << std::to_string(angle));

  while (angle > max_angle)
    angle -= 2.0 * PRX_PI;
  while (angle < min_angle)
    angle += 2.0 * PRX_PI;
  return angle;
}

static inline vector_t heatmap_value(double val)
{
  const std::vector<double> r_vals = { 165, 215, 244, 253, 254, 224, 171, 116, 69, 49 };
  const std::vector<double> g_vals = { 0, 48, 109, 174, 224, 243, 217, 173, 117, 54 };
  const std::vector<double> b_vals = { 38, 39, 67, 97, 144, 248, 233, 209, 180, 149 };

  const double d_index = std::min(val * 10.0, 10.0);
  const int lower_index = std::min(std::floor(d_index), 9.0);
  const int ceil_index = std::min(std::ceil(d_index), 9.0);

  return vector_t(
      (r_vals[lower_index] * (1.0 - (d_index - lower_index)) + r_vals[ceil_index] * ((d_index - lower_index))) / 255.0,
      (g_vals[lower_index] * (1.0 - (d_index - lower_index)) + g_vals[ceil_index] * ((d_index - lower_index))) / 255.0,
      (b_vals[lower_index] * (1.0 - (d_index - lower_index)) + b_vals[ceil_index] * ((d_index - lower_index))) / 255.0);
}
static inline vector_t mono_heatmap_value(double val)
{
  const std::vector<double> r_vals = { 0, 28.33, 56.67, 85.0, 113.33, 141.67, 170.0, 198.33, 226.67, 255. };
  const std::vector<double> g_vals = { 0, 28.33, 56.67, 85.0, 113.33, 141.67, 170.0, 198.33, 226.67, 255. };
  const std::vector<double> b_vals = { 0, 28.33, 56.67, 85.0, 113.33, 141.67, 170.0, 198.33, 226.67, 255. };

  const double d_index = std::min(val * 10.0, 10.0);
  const int lower_index = std::min(std::floor(d_index), 9.0);
  const int ceil_index = std::min(std::ceil(d_index), 9.0);

  return vector_t(val, val, val);
}

static inline std::string rgb_to_string(double r, double g, double b)
{
  std::stringstream stream;
  stream << "0x";
  stream << std::setfill('0') << std::setw(2) << std::hex << (int)(r * 255);
  stream << std::setfill('0') << std::setw(2) << std::hex << (int)(g * 255);
  stream << std::setfill('0') << std::setw(2) << std::hex << (int)(b * 255);
  return stream.str();
}

static inline std::string rgb_to_string(vector_t v)
{
  return rgb_to_string(v.x(), v.y(), v.z());
}

static inline vector_t string_to_rgb(std::string input)
{
  std::vector<std::pair<char, char>> pairs;
  std::vector<double> outputs;
  pairs.push_back(std::make_pair(input[2], input[3]));
  pairs.push_back(std::make_pair(input[4], input[5]));
  pairs.push_back(std::make_pair(input[6], input[7]));
  for (auto val : pairs)
  {
    int value = 0;
    if (val.first >= 'a')
    {
      value += ((val.first - 'a') + 10) * 16;
    }
    else
    {
      value += ((val.first - '0')) * 16;
    }

    if (val.second >= 'a')
    {
      value += ((val.second - 'a') + 10);
    }
    else
    {
      value += ((val.second - '0'));
    }
    outputs.push_back(value / 255.0);
  }
  return vector_t(outputs[0], outputs[1], outputs[2]);
}

template <typename T>
std::vector<T> linspace(T a, T b, size_t N)
{
  T h = (b - a) / static_cast<T>(N - 1);
  std::vector<T> xs(N);
  typename std::vector<T>::iterator x;
  T val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
    *x = val;
  return xs;
}

template <typename T>
T vector_norm(std::vector<T> a)
{
  double norm = 0;
  for (int i = 0; i < a.size(); i++)
    norm += a.at(i) * a.at(i);
  return norm;
}

template <typename T>
int sgn(T a)
{
  return (a > 0) - (a < 0);
}

template <typename T>
static std::vector<T> split(const std::string str)
{
  using prx::constants::separating_value;

  std::vector<T> result;
  std::istringstream ss(str);
  std::string token;
  while (std::getline(ss, token, separating_value))
  {
    if (token.size() > 0)
    {
      std::istringstream ti(token);
      T x;
      if ((ti >> x))
        result.push_back(x);
    }
  }
  return result;
}

inline Eigen::Vector3d quaternion_to_euler(const quaternion_t& q)
{
  // roll (x-axis rotation)
  const double sinr_cosp{ 2 * (q.w() * q.x() + q.y() * q.z()) };
  const double cosr_cosp{ 1 - 2 * (q.x() * q.x() + q.y() * q.y()) };
  const double x{ std::atan2(sinr_cosp, cosr_cosp) };

  // pitch (y()-ax()is rotation)
  const double sinp{ std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())) };
  const double cosp{ std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z())) };
  const double y{ 2 * std::atan2(sinp, cosp) - M_PI / 2.0 };

  // yaw() (z-ax()is rotation)
  const double siny_cosp{ 2 * (q.w() * q.z() + q.x() * q.y()) };
  const double cosy_cosp{ 1 - 2 * (q.y() * q.y() + q.z() * q.z()) };
  const double z{ std::atan2(siny_cosp, cosy_cosp) };

  return { x, y, z };
}
// Split block by columns defined by Columns
// A Block of with columns {C0,C1,C2} and given columns={{0,1}, {2}}
// will return a block {C0,C1} and the input block will change to {C2}
template <typename Block, typename ColumnsQuery>
Block split_block(Block& block_in, ColumnsQuery columns)
{
  Block block_out;
  const auto columns_out{ columns[0] };
  const auto columns_in{ columns[1] };

  std::vector<bool> columns_flags(columns_in.size() + columns_out.size());
  for (auto idx : columns_out)
  {
    columns_flags[idx] = true;
  }
  for (auto idx : columns_in)
  {
    columns_flags[idx] = false;
  }

  for (std::size_t i = 0; i < block_in.size(); ++i)
  {
    auto line_in{ block_in[i] };
    decltype(line_in) line_out;
    decltype(line_in) line_in_new;

    for (std::size_t ci = 0; ci < line_in.size(); ++ci)
    {
      const auto val = line_in[ci];
      if (columns_flags[ci])
      {
        line_out.emplace_back(val);
      }
      else
      {
        line_in_new.emplace_back(val);
      }
    }
    block_in[i] = line_in_new;
    block_out.emplace_back(line_out);
  }
  return block_out;
}

}  // namespace prx
