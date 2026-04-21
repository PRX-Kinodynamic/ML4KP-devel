#pragma once

#include "general/debug_utils.hpp"
#include "general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
// #include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/data_structures/shared_ptr_factory.hpp"

#include <algorithm>
#include <functional>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace prx
{
namespace data_structures
{
// #define MAX_KK 2000
// #define INIT_NODE_SIZE 1000
// #define INIT_CAP_NEIGHBORS 200

template <typename Value>
struct gnn_node_t
{
  using GnnIndex = std::size_t;
  using Neighbors = std::vector<GnnIndex>;

  void add_neighbor(const GnnIndex new_neighbor)
  {
    neighbors.push_back(new_neighbor);
  }

  void delete_neighbor(const GnnIndex node_to_delete)
  {
    // std::erase_if <- only available in c++20 which is implemented as:
    auto it = std::remove(neighbors.begin(), neighbors.end(), node_to_delete);
    auto r = neighbors.end() - it;
    neighbors.erase(it, neighbors.end());
  }

  GnnIndex idx;
  Value value;
  Neighbors neighbors;

  GnnIndex added_index;
};
/**
 * A proximity structure based on graph literature. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief <b> A proximity structure based on graph literature. </b>
 * @author Kostas Bekris, Edgar Granados
 */
template <typename Value, typename DistanceFunction, int MaxClosestNeighbors = 2000>
class graph_nearest_neighbors_t
{
public:
  using Node = gnn_node_t<Value>;
  using NodePtr = std::shared_ptr<Node>;
  using Index = typename Node::GnnIndex;
  using Neighbors = typename Node::Neighbors;
  using Distance = typename std::invoke_result<DistanceFunction, Value, Value>::type;

  // TODO: A better way would be to have pairs<NodePtr, Distance>, only compute distance once and
  // sort based on the precomputed distance. Only necessary if distance is expensive to compute.
  // (Usually not the bottle)
  using QueryNodeCompareFunction = std::function<bool(const NodePtr&, const NodePtr&)>;
  using ValueDistancePair = std::pair<Value, Distance>;
  /**
   * @brief Constructor
   * @param state The first node to add to the structure.
   */

  graph_nearest_neighbors_t(DistanceFunction distance_function = DistanceFunction())
    : _distance_function(distance_function)
    , _gnn_nodes_factory(1000)
    , MAX_NEIGHBORS(200)
    , _query_node(std::make_shared<Node>())
    , _node_compare_function([&](const NodePtr& a, const NodePtr& b) {
      return _distance_function(a->value, _aux_cmp_node->value) < _distance_function(b->value, _aux_cmp_node->value);
    })
    , _next_idx(0)
    , _added_node_id(0) {};

  graph_nearest_neighbors_t(prx::param_loader params, DistanceFunction df = DistanceFunction())
    : MAX_NEIGHBORS(params.get_or_default("max_neighbors", 200))
    , _distance_function(df)
    , _gnn_nodes_factory(params.get_or_default("buffer_size", 1000))
    , _query_node(std::make_shared<Node>())
    , _node_compare_function([&](const NodePtr& a, const NodePtr& b) {
      return _distance_function(a->value, _aux_cmp_node->value) < _distance_function(b->value, _aux_cmp_node->value);
    })
    , _next_idx(0)
    , _added_node_id(0) {};

  ~graph_nearest_neighbors_t() {};

  void distance_function(DistanceFunction df)
  {
    _distance_function = df;
  }
  /**
   * Adds a node to the proximity structure
   * @brief Adds a node to the proximity structure
   * @param node The node to insert.
   */
  std::size_t insert(Value& new_value)
  {
    return insert(new_value, _next_idx);
  }

  std::size_t insert(Value& new_value, const std::size_t value_idx)
  {
    NodePtr new_node{ _gnn_nodes_factory.next() };
    new_node->value = new_value;
    new_node->idx = value_idx;
    _next_idx = value_idx + 1;

    const std::size_t k{ percolation_threshold() };

    // int find_k_close(const State state, const int k)

    std::vector<NodePtr> close_nodes;
    const std::size_t new_k{ find_k_close(close_nodes, new_node, k) };

    _nodes[new_node->idx] = new_node;

    for (int i = 0; i < new_k; i++)
    {
      new_node->add_neighbor(close_nodes[i]->idx);
      close_nodes[i]->add_neighbor(new_node->idx);
    }
    return value_idx;
  }

  /**
   * @brief Removes a node from the structure.
   * @param node
   */
  void remove_node(const std::size_t node_to_delete)
  {
    NodePtr graph_node{ _nodes[node_to_delete] };
    Neighbors neighbors{ graph_node->neighbors };
    for (auto&& ni : neighbors)
    {
      _nodes[ni]->delete_neighbor(graph_node->idx);
    }
    graph_node->neighbors.clear();
    _nodes.erase(node_to_delete);
    /////////////////////////
    // long unsigned nr_neighbors;
    // long unsigned* neighbors = graph_node->get_neighbors(&nr_neighbors);
    // for (int i = 0; i < nr_neighbors; i++)
    // {
    //   nodes[neighbors[i]]->delete_neighbor(graph_node->get_prox_index());
    // }
    // graph_node->remove_all_neighbors();

    // int index = graph_node->get_prox_index();
    // if (index < nr_nodes - 1)
    // {
    //   nodes[index] = nodes[nr_nodes - 1];
    //   nodes[index]->set_index(index);

    //   neighbors = nodes[index]->get_neighbors(&nr_neighbors);
    //   for (int i = 0; i < nr_neighbors; i++)
    //   {
    //     nodes[neighbors[i]]->replace_neighbor(nr_nodes - 1, index);
    //   }
    // }
    // nr_nodes--;
    // graph_node->added_to_metric = false;

    // if (nr_nodes < (cap_nodes - 1) / 2)
    // {
    //   cap_nodes *= 0.5;
    //   nodes = (proximity_node_t**)realloc(nodes, cap_nodes * sizeof(proximity_node_t*));
    // }
  }

  // void clear();

  /**
   * Get function for number of nodes
   * @return Number of nodes in graph
   */
  std::size_t size()
  {
    return _nodes.size();
  }

  ValueDistancePair closest_neighbor(const Value query_value)
  {
    _query_node->value = query_value;
    auto [node, distance, idx] = basic_closest_search(_query_node);
    return { node->value, distance };
  }

  std::vector<Value> k_neighbors(const Value query_value, const int k)
  {
    _query_node->value = query_value;
    std::vector<NodePtr> close_nodes;
    const std::size_t new_k{ find_k_close(close_nodes, _query_node, k) };

    std::vector<Value> result;
    for (auto node : close_nodes)
    {
      result.push_back(node->value);
    }
    return std::move(result);
  }

  std::vector<Value> neighbors_in_radius(const Value query_value, const double radius)
  {
    _query_node->value = query_value;
    std::vector<NodePtr> close_nodes;
    ///////////////////////////////////
    // query_node->point = point;
    std::size_t new_k{ find_delta_close_and_closest(close_nodes, _query_node, radius) };
    std::vector<Value> result;
    for (auto node : close_nodes)
    {
      result.push_back(node->value);
    }
    return result;
  }

protected:
  /**
   * Returns the closest node in the data structure.
   * @brief Returns the closest node in the data structure.
   * @param state The query point.
   * @param distance The resulting distance between the closest point and the query point.
   * @return The closest point.
   */
  // proximity_node_t* find_closest(proximity_node_t* state, double* distance);

  /**
   * Find the k closest nodes to the query point.
   * @brief Find the k closest nodes to the query point.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param k The number to return.
   * @return The number of nodes actually returned.
   */
  std::size_t find_k_close(std::vector<NodePtr>& close_nodes, const NodePtr query_node, const int k_in)
  {
    if (_nodes.size() == 0)
    {
      return 0;
    }
    int k{ k_in };
    if (k > MaxClosestNeighbors)
    {
      k = MaxClosestNeighbors;
    }
    else if (k >= _nodes.size())
    {
      for (auto ni : _nodes)
      {
        close_nodes.push_back(ni.second);
      }
      // sort_proximity_nodes(close_nodes, distances, 0, _total_nodes - 1);
      _aux_cmp_node = query_node;
      std::sort(close_nodes.begin(), close_nodes.end(), _node_compare_function);
      _aux_cmp_node = nullptr;
      return close_nodes.size();
    }
    clear_added();

    std::vector<double> distances;

    auto [node, node_distance, min_index] = basic_closest_search(query_node);
    close_nodes.push_back(node);
    distances.push_back(node_distance);
    // close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);
    _nodes[min_index]->added_index = _added_node_id;

    min_index = 0;
    // int nr_elements{ 1 };
    // double max_distancedistance;

    /* Find the neighbors of the closest node if they are not already in the set of k-closest nodes.
    If the distance to any of the neighbors is less than the distance to the k-th closest element,
    then replace the last element with the neighbor and resort the list. In order to decide the next
    node to pivot about, it is either the next node on the list of k-closest
    */
    Distance prev_distance{ node_distance };
    do
    {
      Index idx{ close_nodes[min_index]->idx };
      Neighbors& neighbors{ _nodes[idx]->neighbors };
      // unsigned nr_neighbors;
      // long unsigned* neighbors = nodes[close_nodes[min_index]->get_prox_idx]->get_neighbors(&nr_neighbors);
      Index lowest_replacement{ close_nodes.size() };

      for (int j = 0; j < neighbors.size(); j++)
      {
        NodePtr neighbor{ _nodes[neighbors[j]] };
        // if (_nodes.count(neighbor->idx) > 0)
        if (not node_exists(neighbor))
        {
          neighbor->added_index = _added_node_id;
          const Distance distance{ _distance_function(neighbor->value, query_node->value) };
          bool to_resort{ false };
          if (close_nodes.size() < k)
          {
            close_nodes.push_back(neighbor);
            distances.push_back(distance);
            // distances[nr_elements] = distance;
            to_resort = true;
          }
          else if (distance < distances[k - 1])
          {
            close_nodes[k - 1] = neighbor;
            distances[k - 1] = distance;
            to_resort = true;
          }

          if (to_resort)
          {
            // std::sort(close_nodes.begin() , close_nodes.end(), _distance_function);
            const int test{ resort_proximity_nodes(close_nodes, distances, close_nodes.size() - 1) };
            lowest_replacement = (test < lowest_replacement ? test : lowest_replacement);
          }
        }
      }
      ////////////////////////////

      /* In order to decide the next node to pivot about,
      it is either the next node on the list of k-closest (min_index)
      or one of the new neighbors in the case that it is closer than nodes already checked.
      */
      if (min_index < lowest_replacement)
      {
        min_index++;
      }
      else
      {
        min_index = lowest_replacement;
      }
    } while (min_index < close_nodes.size());

    return close_nodes.size();
  }

  void clear_added()
  {
    _added_node_id++;
  }
  /**
   * Find all nodes within a radius and the closest node.
   * @brief Find all nodes within a radius and the closest node.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param delta The radius to search within.
   * @return The number of nodes returned.
   */
  // const std::size_t new_k{ find_k_close(close_nodes, _query_node, k) };
  std::size_t find_delta_close_and_closest(std::vector<NodePtr>& close_nodes, NodePtr query_node, const double delta)
  {
    if (_nodes.size() == 0)
    {
      return 0;
    }

    clear_added();

    // std::vector<double> distances;
    // Index min_index;  // = -1;
    // close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);
    auto [node, node_distance, min_index] = basic_closest_search(query_node);
    close_nodes.push_back(node);
    // distances.push_back(node_distance);

    if (node_distance > delta)
    {
      return 1;
    }

    _nodes[min_index]->added_index = _added_node_id;

    for (int counter = 0; counter < close_nodes.size(); counter++)
    {
      // long unsigned nr_neighbors;
      // long unsigned* neighbors = close_nodes[counter]->get_neighbors(&nr_neighbors);
      Neighbors& neighbors{ close_nodes[counter]->neighbors };
      for (int j = 0; j < neighbors.size(); j++)
      {
        NodePtr neighbor{ _nodes[neighbors[j]] };
        // proximity_node_t* the_neighbor = nodes[neighbors[j]];
        if (not node_exists(neighbor))
        {
          neighbor->added_index = _added_node_id;
          const Distance distance{ _distance_function(neighbor->value, query_node->value) };
          ////////////////////////
          // double distance = node_distance(the_neighbor, state);
          if (distance < delta && close_nodes.size() < MaxClosestNeighbors)
          {
            close_nodes.push_back(neighbor);
            // distances.push_back(distance);
            // nr_points++;
          }
        }
      }
    }

    _aux_cmp_node = query_node;
    std::sort(close_nodes.begin(), close_nodes.end(), _node_compare_function);
    _aux_cmp_node = nullptr;

    return close_nodes.size();
  }

  /**
   * Find all nodes within a radius.
   * @brief Find all nodes within a radius.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param delta The radius to search within.
   * @return The number of nodes returned.
   */
  // int find_delta_close(proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta);

  /**
   * Determine the number of nodes to sample for initial populations in queries.
   * @brief Determine the number of nodes to sample for initial populations in queries.
   * @return The number of random nodes to initially select.
   */
  inline std::size_t sampling_function() const
  {
    return std::min(_nodes.size(), MAX_NEIGHBORS);
  }

  /**
   * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @return
   */
  inline std::size_t percolation_threshold()
  {
    if (_nodes.size() > 12)
      return static_cast<std::size_t>(2.0 * std::log(_nodes.size()));
    else
      return _nodes.size();
  }

  /**
   * Sorts a list of proximity_node_t's. Performed using a quick sort operation.
   * @param close_nodes The list to sort.
   * @param distances The distances that determine the ordering.
   * @param low The lower index.
   * @param high The upper index.
   */
  // void sort_proximity_nodes(std::vector<NodePtr>& nodes_to_sort, std::vector<double>& node_distances, const int low,
  // const int high);

  /**
   * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
   * @param close_nodes The list to sort.
   * @param distances The distances that determine the ordering.
   * @param index The index to start from.
   */
  int resort_proximity_nodes(std::vector<NodePtr>& close_nodes, std::vector<double>& distances, int index_)
  {
    {
      double temp;
      NodePtr temp_node;

      while (index_ > 0 && distances[index_] < distances[index_ - 1])
      {
        temp = distances[index_];
        distances[index_] = distances[index_ - 1];
        distances[index_ - 1] = temp;

        temp_node = close_nodes[index_];
        close_nodes[index_] = close_nodes[index_ - 1];
        close_nodes[index_ - 1] = temp_node;

        index_--;
      }
      return index_;
    }
  }

  /**
   * Helper function for determining existance in a list.
   * @brief Helper function for determining existance in a list.
   * @param query_node The node to search for.
   * @param node_list The list to search.
   * @param list_size The size of the list.
   * @return If query_node exists in node_list.
   */
  // bool does_node_exist(proximity_node_t* query_node);

  Index random_uniform(const Index min, const Index max)
  {
    const double val{ _uniform_zero_one(_global_generator) };
    const Index r{ static_cast<Index>(val * (max - min) + min) };
    return r;
  }

  Index sample_node()
  {
    prx_assert(_nodes.size() > 0, "Nearest neighbor operation on empty graph");
    const Index index{ random_uniform(0, _nodes.size()) };

    if (_nodes.count(index) > 0)
    {
      return index;
    }
    // PRX_DEBUG_VARS(index, _nodes.size())
    return sample_node();
  }
  /**
   * The basic search process for finding the closest node to the query state.
   * @brief Find the closest node to the query state.
   * @param state The query state.
   * @param distance The corresponding distance to the query point.
   * @param node_index The index of the returned node.
   * @return The closest node.
   */
  // Node, distance, index
  std::tuple<NodePtr, Distance, Index> basic_closest_search(const NodePtr query_node)
  {
    if (_nodes.size() == 0)
    {
      return { nullptr, 0, 0 };
    }

    const Index total_samples{ sampling_function() };
    double min_distance{ std::numeric_limits<double>::max() };
    Index min_index{ std::numeric_limits<std::size_t>::max() };
    bool node_found{ false };
    for (int i = 0; i < total_samples; i++)
    {
      // int index_ = rand() % nr_nodes;
      const Index index{ sample_node() };
      const double distance{ _distance_function(_nodes[index]->value, query_node->value) };
      if (distance < min_distance)
      {
        min_distance = distance;
        min_index = index;
        node_found = true;
      }
    }

    prx_assert(node_found, "Error: GNN couldn't find a close neighbor");
    Index old_min_index{ min_index };
    do
    {
      old_min_index = min_index;
      // long unsigned nr_neighbors;
      Neighbors& neighbors{ _nodes[min_index]->neighbors };
      for (int j = 0; j < neighbors.size(); j++)
      {
        const Index neighbor{ neighbors[j] };
        const double distance{ _distance_function(_nodes[neighbor]->value, query_node->value) };
        if (distance < min_distance)
        {
          min_distance = distance;
          min_index = neighbor;
        }
      }
    } while (old_min_index != min_index);

    // *the_distance = min_distance;
    // *the_index = min_index;
    // return nodes[min_index];
    return { _nodes[min_index], min_distance, min_index };
  }

  bool node_exists(const NodePtr node) const
  {
    return node->added_index == _added_node_id;
  }

  const std::size_t MAX_NEIGHBORS;

  NodePtr _query_node, _aux_cmp_node;
  // void clear_added();
  DistanceFunction _distance_function;

  // Given two nodes, compute
  QueryNodeCompareFunction _node_compare_function;

  prx::pointer_factory_t<Node> _gnn_nodes_factory;

  Index _next_idx;
  Index _added_node_id;

  std::unordered_map<Index, NodePtr> _nodes;

  std::mt19937_64 _global_generator;
  std::uniform_real_distribution<double> _uniform_zero_one;
};
}  // namespace data_structures
}  // namespace prx