#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/transforms.hpp"

#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system.hpp"

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>

#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
namespace utilities
{

const gtsam::Values& optimize_and_log(gtsam::NonlinearOptimizer& nl_opt, const gtsam::NonlinearOptimizerParams& params,
                                      logger_t& logger, const int extra_iters = 0,
                                      condition_check_t* checker_0 = nullptr);
const gtsam::Values& optimize_and_log(gtsam::NonlinearOptimizer& nl_opt, const gtsam::NonlinearOptimizerParams& params);

void values_to_plan_and_traj(const gtsam::Values& vals, trajectory_t* traj, plan_t* plan, const int total_steps);
// void values_to_plan_and_traj(gtsam::Values& vals, trajectory_t* traj, plan_t* plan, const double duration);

void updates_values_from_plan_and_traj(gtsam::Values& values, system_ptr_t _sys_ptr, const trajectory_t& traj,
                                       const plan_t& plan, const int total_steps);

void add_noise(gtsam::Values& values, const std::string symbol_name,
               boost::shared_ptr<gtsam::noiseModel::Isotropic>& noise);

void values_to_plan(const gtsam::Values& vals, plan_t* plan, const int total_steps);

void values_to_traj(const gtsam::Values& vals, trajectory_t& traj, const int total_steps);

template <typename Control, typename Tau>
void extract_plan_from_values(const gtsam::Values& vals, prx::plan_t& plan, const std::size_t total_controls)
{
  plan.clear();
  std::size_t ti = 0;
  for (; ti < total_controls; ti++)
  {
    auto us = symbol_factory_t::create_symbol("control_symbol", ti);
    auto ts = symbol_factory_t::create_symbol("time_symbol", ti);

    auto u = vals.at<Control>(us);
    double step = vals.at<Tau>(ts)[0];

    plan.copy_onto_back(u, step);
  }
}

template <typename T, std::enable_if_t<!prx::utils::is_iterable<T>{}, bool> = true>
void value_to_ostream(T value, std::ostream& ost = std::cout)
{
  ost << value;
}

template <typename T, std::enable_if_t<prx::utils::is_iterable<T>{}, bool> = true>
void value_to_ostream(T values, std::ostream& ost = std::cout)
{
  for (auto value : values)
  {
    value_to_ostream(value, ost);
    ost << " ";
  }
}

template <typename T>
void values_to_ostream(gtsam::Values& values, std::ostream& ost = std::cout)
{
  auto filtered = values.extract<T>();

  for (auto key_value : filtered)
  {
    ost << prx::symbol_factory_t::formatter(key_value.first) << " ";
    value_to_ostream(key_value.second, ost);
    ost << "\n";
  }
}

template <std::size_t I, typename... Tp, std::enable_if_t<(I == sizeof...(Tp) - 1), bool> = true>
inline static void print_values_1(gtsam::Values& values, std::ostream& ost = std::cout)
{
  // typename std::tuple_element<I, std::tuple<Tp...> >::type T;
  using type = typename std::tuple_element<I, std::tuple<Tp...>>::type;
  values_to_ostream<type>(values, ost);
}

template <std::size_t I, typename... Tp, std::enable_if_t<(I < sizeof...(Tp) - 1), bool> = true>
inline static void print_values_1(gtsam::Values& values, std::ostream& ost = std::cout)
{
  // typename std::tuple_element<I, std::tuple<Tp...> >::type T;
  using Type = typename std::tuple_element<I, std::tuple<Tp...>>::type;
  values_to_ostream<Type>(values, ost);
  print_values_1<I + 1, Tp...>(values, ost);
}

template <typename... Ts>
void print_values(gtsam::Values& values, std::ostream& ost = std::cout)
{
  print_values_1<0, Ts...>(values, ost);
}
template <typename... Ts>
void values_to_file(gtsam::Values& values, const std::string& filename,
                    const std::ios_base::openmode _mode = std::ofstream::trunc)
{
  std::ofstream ofs;
  ofs.open(filename.c_str(), _mode);
  print_values<Ts...>(values, ofs);
  ofs.close();
}

template <typename ValueType>
void values_to_file(gtsam::Values& values, const std::string regex, const std::string filename,
                    const std::string regex_key_to_number, const std::string sep = " ",
                    const std::ios_base::openmode _mode = std::ofstream::trunc)
{
  using ::prx::utilities::convert_to;
  using StrValuePair = std::pair<std::size_t, ValueType>;
  std::ofstream ofs(filename.c_str(), _mode);
  std::regex key_regex(regex);
  std::regex key_to_number_regex(regex_key_to_number);
  std::smatch base_match;
  std::vector<StrValuePair> list;
  for (auto key_value : values)
  {
    const prx::prx_symbol_t key{ key_value.key };
    const std::string key_str{ prx::symbol_factory_t::formatter(key) };
    if (std::regex_match(key_str, base_match, key_regex))
    {
      const ValueType value{ values.at<ValueType>(key) };  // Must be a better way of doing this.
      std::stringstream result;
      std::regex_replace(std::ostream_iterator<char>(result), key_str.begin(), key_str.end(), key_to_number_regex, "");

      list.push_back(std::make_pair(convert_to<std::size_t>(result.str()), value));
    }
  }
  std::sort(list.begin(), list.end(), [](StrValuePair& a, StrValuePair& b) { return a.first < b.first; });
  for (auto pair : list)
  {
    const std::size_t key_str{ pair.first };
    const ValueType value{ pair.second };
    ofs << key_str << sep;
    for (auto e : value)
    {
      ofs << e << sep;
    }
    ofs << "\n";
  }
}

}  // namespace utilities
}  // namespace fg
}  // namespace prx