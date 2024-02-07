#pragma once

#include <vector>
#include <unordered_set>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/factors/factors.hpp"
namespace prx
{
namespace fg
{

std::string factor_name(gtsam::NonlinearFactorGraph::sharedFactor& factor_ptr)
{
  gtsam::Factor* factor{ factor_ptr.get() };
  if (dynamic_cast<gtsam::PriorFactor<Eigen::Vector<double, 2>>*>(factor) != nullptr)
    return "PriorFactor";
  // if (dynamic_cast<prx::fg::ackermann_q_qdot_u_t*>(factor) != nullptr)
  //   return "ackermann_q_qdot_u_t";
  // if (dynamic_cast<prx::fg::ackermann_q_qdot_qdotdot_t*>(factor) != nullptr)
  //   return "ackermann_q_qdot_qdotdot_t";
  // if (dynamic_cast<prx::fg::ackermann_qdotdot_force_q_t*>(factor) != nullptr)
  //   return "ackermann_qdotdot_force_q_t";
  // if (dynamic_cast<prx::fg::ackermann_q_observation_t*>(factor) != nullptr)
  //   return "ackermann_q_observation_t";
  if (dynamic_cast<prx::fg::friction_fusion_t*>(factor) != nullptr)
    return "friction_fusion_t";
  prx_warn("Unknown factor!");
  factor->print("UnknownFactor", prx::symbol_factory_t::formatter);
  return "nullptr";
}

template <Eigen::Index FactorPosDim, typename VariablePositions>
void fg_to_csv(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const std::string& filename,
               const VariablePositions& var_pos)
{
  using FactorPosition = Eigen::Vector<double, FactorPosDim>;
  std::ofstream ofs;
  ofs.open(filename.c_str(), std::ofstream::trunc);

  std::size_t factor_id{ 0 };
  std::unordered_set<gtsam::Key> keys_set;
  for (auto factor : graph)
  {
    FactorPosition factor_pos{ FactorPosition::Zero() };
    std::size_t count{ 0 };
    for (auto key : *factor)
    {
      auto tuple = var_pos(values, key);
      if (std::get<0>(tuple))
      {
        auto val = std::get<1>(tuple);
        // PRX_DEBUG_VAR_1(val.transpose());
        // PRX_DEBUG_VAR_2(factor_id, factor_pos.transpose());
        factor_pos += val;
        count++;
        if (keys_set.count(key) == 0)
        {
          keys_set.insert(key);
          ofs << "key " << prx::symbol_factory_t::formatter(key) << " " << val.transpose() << "\n";
        }
      }
    }
    if (count > 0)
    {
      factor_pos = factor_pos / count;
      ofs << "factor " << factor_id << " " << factor_pos.transpose() << " " << count << "\n";
    }
    for (auto key : *factor)
    {
      auto tuple = var_pos(values, key);
      if (std::get<0>(tuple))
      {
        auto val = std::get<1>(tuple);
        // ofs << "key " << prx::symbol_factory_t::formatter(key) << " " << factor_pos.transpose() << " "
        ofs << "edge " << factor_pos.transpose() << " " << val.transpose() << "\n";
      }
    }
    factor_id++;
  }
}

void fg_errors_to_csv(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                      const std::string& filename)
{
  std::ofstream ofs_graph;
  ofs_graph.open(filename.c_str(), std::ofstream::trunc);

  std::size_t factor_id{ 0 };
  for (auto factor : graph)
  {
    if (factor != nullptr)
    {
      ofs_graph << factor_id << " ";
      ofs_graph << factor_name(factor) << " ";
      ofs_graph << factor->error(values) << "\n";
    }
    factor_id++;
  }
  ofs_graph.close();
}
}  // namespace fg
}  // namespace prx