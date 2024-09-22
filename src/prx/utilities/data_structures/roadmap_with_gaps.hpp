#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/general/csv_reader.hpp"

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

  void load_roadmap_from_file(std::string vertices_fname, std::string edges_fname)
  {
    using prx::constants::separating_value;
    using prx::utilities::csv_reader_t;

    csv_reader_t v_reader(vertices_fname, separating_value);
    using Line = std::vector<std::string>;
    while (v_reader.has_next_line())
    {
      Line line { v_reader.next_line()};
      std::vector<double> landmark_vector;
      if (line.size() > 0)
      {
        node_index_t v_idx = prx::utilities::convert_to<node_index_t>(line[0]);
        for (int i = 1; i < line.size(); i++)
        {
          landmark_vector.push_back(prx::utilities::convert_to<double>(line[i]));
        }
        space_point_t landmark = spec->state_space->make_point();
        spec->state_space->copy_point_from_vector(landmark, landmark_vector);
        add_vertex(v_idx, landmark);
      }
    }

    csv_reader_t e_reader(edges_fname, separating_value);
    while (e_reader.has_next_line())
    {
      Line line { e_reader.next_line()};
      if (line.size() > 0)
      {
        node_index_t s = prx::utilities::convert_to<node_index_t>(line[0]);
        node_index_t t = prx::utilities::convert_to<node_index_t>(line[1]);
        double cost = prx::utilities::convert_to<double>(line[2]);
        add_edge(s, t, cost);
      }
    }

    std::cout << "Loaded the roadmap with " << vertices.size() << " vertices and " << all_edges.size() << " edges"
              << std::endl;
  }

  std::shared_ptr<roadmap_with_gaps_node_t> get_vertex(node_index_t index)
  {
    return vertices[index];
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

  std::vector<std::pair<node_index_t, std::shared_ptr<roadmap_with_gaps_node_t>>> get_vertices()
  {
    std::vector<std::pair<node_index_t, std::shared_ptr<roadmap_with_gaps_node_t>>> v;
    for (auto it = vertices.begin(); it != vertices.end(); it++)
    {
      v.push_back(std::make_pair(it->first, it->second));
    }
    return v;
  }

  std::vector<std::pair<node_index_t, node_index_t>> get_edges()
  {
    std::vector<std::pair<node_index_t, node_index_t>> e;
    for (auto it = edges.begin(); it != edges.end(); it++)
    {
      for (auto e_it = it->second.begin(); e_it != it->second.end(); e_it++)
      {
        e.push_back(std::make_pair(it->first, (*e_it)->get_target_index()));
      }
    }
    return e;
  }

  double get_edge_cost(node_index_t s, node_index_t t)
  {
    for (auto e : edges[s])
    {
      if (e->get_target_index() == t)
      {
        return e->get_cost();
      }
    }
    return std::numeric_limits<double>::infinity();
  }

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
          a_costs[v.first] = spec->cost_function(query->solution_traj,query->solution_plan);
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
          d_costs[v.first] = spec->cost_function(query->solution_traj,query->solution_plan);
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

      if (unconsidered.size() % 10 == 0)
      {
        std::cout << unconsidered.size() << " landmarks remaining.\n";
        std::cout << "Vertices: " << vertices.size() << "\n";
        std::cout << "Edges: " << all_edges.size() << "\n";
      }

    } while (unconsidered.size() > 0);
  }

  int get_shortcut_successor(space_point_t s, int vertex_id)
  {
    auto v = vertices[vertex_id];
    int best_idx = -1;

    do 
    {
      spec->state_space->copy_point(query->goal_state, v->point);
      spec->state_space->copy_point(query->start_state, s);

      controller->fulfill_query(*spec.get(), *query.get());

      if (spec->valid_check(query->solution_traj) && query->solution_traj.size() > 0)
      {
        best_idx = v->get_successor_index();
      }
      else
      {
        break;
      }
      if (v -> get_successor_index() == -1)
      {
        break;
      }
      v = vertices[v->get_successor_index()];
    } while (true);

    return best_idx;
  }

  node_index_t add_start(space_point_t s)
  {
    point = spec->state_space->clone_point(s);
    get_a_indices();

    auto v = add_vertex(vertex_counter, point);

    for (auto a : a_indices)
    {
      add_edge(v->get_index(), a, a_costs[a]);
    }

    return v->get_index();
  }

  node_index_t add_goal(space_point_t g)
  {
    point = spec->state_space->clone_point(g);
    get_d_indices();

    auto v = add_vertex(vertex_counter, point);

    for (auto d : d_indices)
    {
      add_edge(d, v->get_index(), d_costs[d]);
    }

    return v->get_index();
  }

  void compute_wavefront(node_index_t goal, bool verbose = false)
  {
    // Compute the wavefront from the goal
    std::priority_queue<std::pair<double, node_index_t>, std::vector<std::pair<double, node_index_t>>,
                        std::greater<std::pair<double, node_index_t>>>
        pq;
    std::map<node_index_t, int> parent;

    for (auto v : vertices)
    {
      vertex_costs[v.first] = std::numeric_limits<double>::infinity();
      parent[v.first] = -1;
    }

    vertex_costs[goal] = 0;
    parent[goal] = goal;
    pq.push(std::make_pair(0, goal));

    while (!pq.empty())
    {
      auto u = pq.top();
      pq.pop();
      for (auto e : in_edges[u.second])
      {
        node_index_t v = e->get_target_index();
        double new_cost = vertex_costs[u.second] + e->get_cost();
        if (new_cost < vertex_costs[v])
        {
          vertex_costs[v] = new_cost;
          parent[v] = u.second;
          pq.push(std::make_pair(new_cost, v));
        }
      }
    }

    for (auto v : vertices)
    {
      auto vertex = v.second;
      vertex->set_cost_to_go(vertex_costs[v.first]);
      vertex->set_successor_index(parent[v.first]);
      if (verbose) std::cout << "Vertex " << v.first << " has cost " << vertex_costs[v.first] << " and successor " << parent[v.first]
                << std::endl;
    }
  }
};