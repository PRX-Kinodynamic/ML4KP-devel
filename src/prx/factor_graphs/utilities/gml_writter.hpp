#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/utilities/general/gml_parser.hpp"
#include <vector>

namespace prx
{
namespace fg
{
using prx::utilities::gml_list_t;

template <typename VariablePositions>
gml_list_t create_graphics(const VariablePositions& var_pos, const gtsam::Key& key, const gtsam::Values& values)
{
  gml_list_t gml_graphics{};
  // Might be a better way to do this
  // VariablePositions returns a tuple {bool, position}
  // bool -> true if position to be used
  auto tuple = var_pos(values, key);
  if (std::get<0>(tuple))
  {
    auto position = std::get<1>(tuple);
    // gml_list_t gml_center{};
    gml_graphics.emplace("x", -position[0]);
    gml_graphics.emplace("y", position[1]);

    // PRX_DEBUG_VAR_1(prx::symbol_factory_t::formatter(key));
    // gml_graphics.emplace("center", gml_center);
  }

  gml_graphics.emplace("w", 750);
  gml_graphics.emplace("h", 350);
  gml_graphics.emplace("type", std::string("ellipse"));
  gml_graphics.emplace("fill", std::string("#ffffff"));
  gml_graphics.emplace("outline", std::string("#000000"));

  return gml_graphics;
}

template <typename VariablePositions>
gml_list_t create_variable_node(const std::size_t& idx, const gtsam::Key& key, const gtsam::Values& values,
                                const VariablePositions& var_pos)
{
  gml_list_t gml_node{};
  gml_node.emplace("id", idx);
  gml_node.emplace("label", prx::symbol_factory_t::formatter(key));

  gml_node.emplace("graphics", create_graphics(var_pos, key, values));

  return gml_node;
}

gml_list_t create_factor_node(const std::size_t& idx, const double x, const double y)
{
  gml_list_t gml_factor{};
  gml_factor.emplace("id", idx);
  gml_factor.emplace("name", std::string("factor"));
  gml_factor.emplace("label", std::string("factor_") + std::to_string(idx));

  gml_list_t gml_graphics{};
  gml_graphics.emplace("x", x);
  gml_graphics.emplace("y", y);
  gml_graphics.emplace("w", 35);
  gml_graphics.emplace("h", 35);
  gml_graphics.emplace("type", std::string("ellipse"));
  gml_graphics.emplace("fill", std::string("#000000"));

  gml_factor.emplace("graphics", gml_graphics);
  return gml_factor;
}

gml_list_t create_edge(const std::size_t& source, const std::size_t& target)
{
  gml_list_t gml_edge{};
  gml_edge.emplace("source", source);
  gml_edge.emplace("target", target);

  return gml_edge;
}

template <typename VariablePositions>
void create_gml_file(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const std::string& filename,
                     const VariablePositions& var_pos)
{
  gml_list_t gml_nodes_edges{};
  // gml_list_t gml_factors_list();
  // gml_list_t gml_variables_list();

  std::size_t next_idx{ 0 };
  std::unordered_map<gtsam::Key, std::size_t> ids_map;
  // std::unordered_map<gtsam::Key, std::tuple<double, double, double>> poses_map;
  for (size_t i = 0; i < graph.size(); ++i)
  {
    const gtsam::NonlinearFactor::shared_ptr& factor = graph.at(i);
    if (factor)
    {
      const std::size_t factor_idx{ next_idx };
      next_idx++;

      // gml_nodes_edges.emplace("node", create_factor_node(factor_idx));
      const gtsam::KeyVector& factor_keys = factor->keys();
      double x{ 0.0 };
      double y{ 0.0 };
      double count{ 0.0 };
      for (auto& key : factor_keys)
      {
        if (ids_map.count(key) == 0)
        {
          ids_map[key] = next_idx;
          next_idx++;
          gml_nodes_edges.emplace("node", create_variable_node(ids_map[key], key, values, var_pos));
        }
        auto tuple = var_pos(values, key);
        if (std::get<0>(tuple))
        {
          auto position = std::get<1>(tuple);
          x += position[0];
          y += position[1];
          count += 1.0;
          // PRX_DEBUG_VAR_2(prx::symbol_factory_t::formatter(key), count);
        }

        // PRX_DEBUG_VAR_2(prx::symbol_factory_t::formatter(key), factor_idx);
        gml_nodes_edges.emplace("edge", create_edge(ids_map[key], factor_idx));
      }
      if (count > 0)
      {
        gml_nodes_edges.emplace("node", create_factor_node(factor_idx, -x / count, -y / count));
        // PRX_DEBUG_VAR_2(-x / count, -y / count);
      }
      else
        gml_nodes_edges.emplace("node", create_factor_node(factor_idx, x, y));
    }
  }
  gml_list_t gml_graph{};
  gml_graph.emplace("graph", gml_nodes_edges);

  std::cout << "filename: " << filename << std::endl;
  std::ofstream ofs_graph;
  ofs_graph.open(filename.c_str(), std::ofstream::trunc);
  ofs_graph << gml_graph;
  ofs_graph.close();
}

}  // namespace fg
}  // namespace prx
