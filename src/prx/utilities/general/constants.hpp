#pragma once

#include "prx/utilities/defs.hpp"

#include <map>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <fstream>

namespace prx
{
#define PRX_PI 3.1415926535897932385
#define PRX_EPSILON 1e-7
#define PRX_INFINITY 1e10
extern int precision;
extern char separating_value;

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
const std::string input_path = lib_path + "resources/input_files/";
const std::string js_path = lib_path + "resources/js/";
// TODO: Add code that creates/check for existance of directory
const std::string out_path = lib_path + "out/";

enum propagate_step
{
  FIRST_STEP,
  MIDDLE_STEP,
  FINAL_STEP
};
enum plant_type
{
  ANALYTICAL,
  MUJOCO,
  GZ
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
std::vector<T> linspace(T a, T b, std::size_t N)
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
void vector_to_file(const std::string file_name, const std::vector<std::vector<T>> _vec,
                    const std::ios_base::openmode _mode)
{
  std::ofstream ofs_map;
  ofs_map.open(file_name.c_str(), _mode);

  for (auto _vec_in : _vec)
  {
    for (auto e : _vec_in)
    {
      ofs_map << std::to_string(e) << " ";
    }
    ofs_map << "\n";
  }
  ofs_map << "\n";

  ofs_map.close();
}

template <typename T>
inline void hash_combine(std::size_t& seed, const T& val)
{
  seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T, std::size_t n_i, std::enable_if_t<(n_i == 0), bool> = true>
inline void range_hash_combine(std::size_t& seed, const T& range)
{
  hash_combine(seed, range[n_i]);
}

template <typename T, std::size_t n_i, std::enable_if_t<(n_i != 0), bool> = true>
inline void range_hash_combine(std::size_t& seed, const T& range)
{
  hash_combine(seed, range[n_i]);
  range_hash_combine<T, n_i - 1>(seed, range);
}

template <typename T, int dimension>
struct range_hash_combine_t
{
  template <int dim = dimension, std::enable_if_t<(dim <= 0), bool> = true>
  const std::size_t operator()(const T& t_to_hash) const noexcept
  {
    std::size_t seed = 0;
    for (auto e : t_to_hash)
    {
      hash_combine(seed, e);
    }
    return seed;
  }

  template <int dim = dimension, std::enable_if_t<(dim > 0), bool> = true>
  const std::size_t operator()(const T& t_to_hash) const noexcept
  {
    std::size_t seed = 0;
    range_hash_combine<T, dimension - 1>(seed, t_to_hash);
    return seed;
  }
};

// ToDo: Use In2... (variadic) to handle multiple containers.
template <typename Container, typename In1, typename In2>
static Container merge_container(const In1& in1, const In2& in2)
{
  Container container(in1.begin(), in1.end());
  container.insert(container.end(), in2.begin(), in2.end());
  return container;
}
template <typename Container, typename In1, typename... InRest>
static Container merge_container(const In1& in1, InRest... in_rest)
{
  Container container(in1.begin(), in1.end());
  const Container merged{ merge_container<Container>(in_rest...) };
  container.insert(container.end(), merged.begin(), merged.end());
  return container;
}

/**
 * @brief      Given a state space, repeated calls to this function will step through all the space. Every call to this
 *             function the state is incremented by step. This can be though of handling a state as a number and
 *             increasing a digit in each call but starting from the right for simplicity: 0xAF00 + step = 0xA010
 *
 * 							Example usage: \\ no-lint
 * 							state <- lower bound;
 * 							do{
 *             		Amazing code here
 *             } while(state_space_step(state, step, ...))
 *
 * @param      state        The state to step over
 * @param[in]  step         The step per dimension
 * @param[in]  dimension    The dimension of the state space
 * @param[in]  lower_bound  The lower bound
 * @param[in]  upper_bound  The upper bound
 *
 * @tparam     State        An object representing the state and accessable through operator[] (i.e state[i])
 * @tparam     Steps 				A container of size dimension. This allows each dimension to be step at different rate.
 * @tparam     Bound        A container of size dimension representing a bound of the space.
 */
template <typename State, typename Steps, typename Bound,
          std::enable_if_t<prx::utils::is_iterable<Steps>{}, bool> = true>
static bool state_space_step(State& state, const Steps steps, const std::size_t& dimension, const Bound lower_bound,
                             const Bound upper_bound)
{
  for (int i = 0; i < dimension; ++i)
  {
    state[i] = state[i] + steps[i];

    if (state[i] <= upper_bound[i])
    {
      return true;
    }
    state[i] = lower_bound[i];
  }
  return false;
}

template <typename State, typename Step, typename Bound,
          std::enable_if_t<!prx::utils::is_iterable<Step>{}, bool> = true>
static bool state_space_step(State& state, const Step step, const std::size_t& dimension, const Bound lower_bound,
                             const Bound upper_bound)
{
  const std::vector<double> steps(dimension, step);
  return state_space_step(state, steps, dimension, lower_bound, upper_bound);
}

template <typename T>
static std::vector<T> split(std::string str, const char separator = prx::separating_value)
{
  std::vector<T> result;
  std::istringstream ss(str);
  std::string token;
  while (std::getline(ss, token, separator))
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

}  // namespace prx
