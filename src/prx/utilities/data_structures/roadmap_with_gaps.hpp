#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/data_structures/gnn.hpp"

using namespace prx;

class roadmap_with_gaps_node_t : public abstract_node_t, std::enable_shared_from_this<roadmap_with_gaps_node_t>
{
public:
  roadmap_with_gaps_node_t() : abstract_node_t()
  {
  }

  ~roadmap_with_gaps_node_t()
  {
  }

  void set_index(node_index_t index_)
  {
    index = index_;
  }

  node_index_t get_index() const
  {
    return index;
  }

  void set_successor_index(int successor_index_)
  {
    successor_index = successor_index_;
  }

  int get_successor_index() const
  {
    return successor_index;
  }

  void set_cost_to_go(double cost_to_go_)
  {
    cost_to_go = cost_to_go_;
  }

  double get_cost_to_go() const
  {
    return cost_to_go;
  }

protected:
  node_index_t index;
  int successor_index;
  double cost_to_go;
};

class roadmap_with_gaps_edge_t : public abstract_edge_t
{
public:
  roadmap_with_gaps_edge_t() : abstract_edge_t()
  {
  }

  ~roadmap_with_gaps_edge_t()
  {
  }

  void set_target_index(node_index_t target_index_)
  {
    target_index = target_index_;
  }

  node_index_t get_target_index() const
  {
    return target_index;
  }

  void set_cost(double cost_)
  {
    cost = cost_;
  }

  double get_cost() const
  {
    return cost;
  }

protected:
  node_index_t target_index;
  double cost;
};

class roadmap_with_gaps_t
{
private:
  std::unordered_map<node_index_t, std::shared_ptr<roadmap_with_gaps_node_t>> vertices;
  std::unordered_map<node_index_t, std::vector<std::shared_ptr<roadmap_with_gaps_edge_t>>> edges;
  std::unordered_map<node_index_t, std::vector<std::shared_ptr<roadmap_with_gaps_edge_t>>> in_edges;
  std::unordered_map<node_index_t, double> vertex_costs;
  std::vector<std::pair<node_index_t, node_index_t>> all_edges;

  node_index_t vertex_counter, edge_counter;

protected:
  space_point_t point;
  std::vector<double> point_vec;

  std::vector<node_index_t> a_indices, d_indices;
  std::unordered_map<node_index_t, double> a_costs, d_costs;

public:
  std::vector<space_point_t> landmarks;
  void add_landmark(space_point_t landmark)
  {
    landmarks.push_back(landmark);
  }

  std::shared_ptr<rrt_specification_t> spec;
  std::shared_ptr<rrt_query_t> query;
  std::shared_ptr<learned_controller_t> controller;

  roadmap_with_gaps_t(rrt_specification_t& _spec, rrt_query_t& _query, learned_controller_t _controller)
  {
    vertex_counter = 0;
    edge_counter = 0;

    spec = std::make_shared<rrt_specification_t>(_spec);
    query = std::make_shared<rrt_query_t>(_query);
    controller = std::make_shared<learned_controller_t>(_controller);
  }

  ~roadmap_with_gaps_t()
  {
  }

  space_point_t get_vertex_point(node_index_t index)
  {
    return vertices[index]->point;
  }

  std::shared_ptr<roadmap_with_gaps_node_t> add_vertex(node_index_t idx, space_point_t point_to_add)
  {
    auto v = std::make_shared<roadmap_with_gaps_node_t>();
    v->set_index(idx);
    v->point = spec->state_space->clone_point(point_to_add);
    vertices.insert(std::make_pair(idx, v));
    vertex_counter = std::max(vertex_counter, idx + 1);
    return v;
  }

  void remove_vertex(node_index_t v)
  {
    for (auto e : edges[v])
    {
      remove_edge(v, e->get_target_index());
    }
    for (auto k : edges)
    {
      for (auto e : k.second)
      {
        if (e->get_target_index() == v)
        {
          remove_edge(k.first, v);
        }
      }
    }
    edges.erase(v);
    vertices.erase(v);
  }

  bool has_edge(node_index_t s, node_index_t t)
  {
    for (auto e : edges[s])
    {
      if (e->get_target_index() == t)
      {
        return true;
      }
    }
    return false;
  }

  void add_edge(node_index_t source, node_index_t target, double cost)
  {
    prx_assert(!has_edge(source, target), "Edge already exists!");
    prx_assert(source != target, "Cannot add self edge!");
    if (edges.find(source) == edges.end())
    {
      edges[source] = std::vector<std::shared_ptr<roadmap_with_gaps_edge_t>>();
    }
    if (in_edges.find(target) == in_edges.end())
    {
      in_edges[target] = std::vector<std::shared_ptr<roadmap_with_gaps_edge_t>>();
    }
    auto e = std::make_shared<roadmap_with_gaps_edge_t>();
    e->set_target_index(target);
    e->set_cost(cost);
    edges[source].push_back(e);

    auto e2 = std::make_shared<roadmap_with_gaps_edge_t>();
    e2->set_target_index(source);
    e2->set_cost(cost);
    in_edges[target].push_back(e2);

    all_edges.push_back(std::make_pair(source, target));
  }

  void remove_edge(node_index_t s, node_index_t t)
  {
    for (auto e = edges[s].begin(); e != edges[s].end();)
    {
      if ((*e)->get_target_index() == t)
      {
        e = edges[s].erase(e);
      }
      else
      {
        ++e;
      }
    }

    for (auto e = in_edges[t].begin(); e != in_edges[t].end();)
    {
      if ((*e)->get_target_index() == s)
      {
        e = in_edges[t].erase(e);
      }
      else
      {
        ++e;
      }
    }
  }

  // std::pair<std::unordered_map<node_index_t, std::shared_ptr<roadmap_with_gaps_node_t>>::iterator,
  //           std::unordered_map<node_index_t, std::shared_ptr<roadmap_with_gaps_node_t>>::iterator> get_vertices()
  // {
  //   return std::make_pair(vertices.begin(), vertices.end());
  // }

  void get_a_indices()
  {
    // a_indices -> all vertices that can be reached from the considered point
    a_indices.clear();
    a_costs.clear();

    for (auto v : vertices)
    {
      spec->state_space->copy_point(query->start_state, point);
      spec->state_space->copy_point(query->goal_state, v.second->point);

      if (!query->goal_check(query->start_state))
      {
        controller->fulfill_query(*spec.get(), *query.get());

        if (query->solution_traj.size() > 0 && spec->valid_check(query->solution_traj))
        {
          a_indices.push_back(v.first);
          a_costs[v.first] = (query->solution_traj.size() - 1) * simulation_step;
        }
      }

      query->clear_outputs();
    }
  }

  void get_d_indices()
  {
    // d_indices -> all vertices that can reach the considered point
    d_indices.clear();
    d_costs.clear();

    for (auto v : vertices)
    {
      spec->state_space->copy_point(query->start_state, v.second->point);
      spec->state_space->copy_point(query->goal_state, point);

      if (!query->goal_check(query->start_state))
      {
        controller->fulfill_query(*spec.get(), *query.get());

        if (query->solution_traj.size() > 0 && spec->valid_check(query->solution_traj))
        {
          d_indices.push_back(v.first);
          d_costs[v.first] = (query->solution_traj.size() - 1) * simulation_step;
        }
      }

      query->clear_outputs();
    }
  }

  void get_a_and_d_indices()
  {
    get_a_indices();
    get_d_indices();
  }

  void build_roadmap()
  {
    point = spec->state_space->make_point();
    std::vector<node_index_t> unconsidered;
    for (int i = 0; i < landmarks.size(); i++)
    {
      unconsidered.push_back(i);
    }
    std::cout << "Building roadmap with " << unconsidered.size() << " landmarks.\n";

    do
    {
      int idx = uniform_int_random(0, unconsidered.size() - 1);
      node_index_t v_idx = unconsidered[idx];
      unconsidered.erase(unconsidered.begin() + idx);

      spec->state_space->copy_point(point, landmarks.at(v_idx));
      get_a_and_d_indices();

      auto v = add_vertex(vertex_counter, point);

      double current_cost;
      for (auto a : a_indices)
      {
        current_cost = a_costs[a];
        add_edge(v->get_index(), a, current_cost);
      }

      for (auto d : d_indices)
      {
        current_cost = d_costs[d];
        add_edge(d, v->get_index(), current_cost);
      }

      vertex_counter++;

      if (unconsidered.size() % 10 == 0)
      {
        std::cout << unconsidered.size() << " landmarks remaining.\n";
        std::cout << "Vertices: " << vertices.size() << "\n";
        std::cout << "Edges: " << all_edges.size() << "\n";
      }

    } while (unconsidered.size() > 0);
  }
};