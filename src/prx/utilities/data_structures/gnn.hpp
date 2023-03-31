#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <functional>
#include <cmath>

#define MAX_KK 2000
#define INIT_NODE_SIZE 1000
#define INIT_CAP_NEIGHBORS 200
namespace prx
{

template <typename Point, typename Node>
class graph_nearest_neighbors_t;
class abstract_node_t;

/**
 * @brief <b> Proximity node for the graph-based distance metric.</b>
 *
 * Proximity node for the graph-based distance metric.
 *
 * @author Kostas Bekris
 */
template <typename Point>
class proximity_node_t
{
public:
  /**
   * @brief Constructor
   */
  proximity_node_t() : nr_neighbors(0), added_index(0), cap_neighbors(INIT_CAP_NEIGHBORS), added_to_metric(false)
  {
    neighbors = (std::size_t*)malloc(INIT_CAP_NEIGHBORS * sizeof(std::size_t));
  }
  virtual ~proximity_node_t()
  {
    free(neighbors);
  }

  /**
   * Gets the position of the node in the data structure. Used for fast deletion.
   * @brief Gets the position of the node in the data structure.
   * @return The index value.
   */
  inline std::size_t get_prox_index() const
  {
    return prox_index;
  }

  /**
   * Sets the position of the node in the data structure. Used for fast deletion.
   * @brief Sets the position of the node in the data structure.
   * @param indx The index value.
   */
  inline void set_index(const std::size_t indx)
  {
    prox_index = indx;
  }

  /**
   * Returns the stored neighbors.
   * @brief Returns the stored neighbors.
   * @param nr_neigh Storage for the number of neighbors returned.
   * @return The neighbor indices.
   */
  inline std::size_t* get_neighbors(std::size_t* nr_neigh)
  {
    *nr_neigh = nr_neighbors;
    return neighbors;
  }

  /**
   * Adds a node index into this node's neighbor list.
   * @brief Adds a node index into this node's neighbor list.
   * @param node The index to add.
   */
  void add_neighbor(const std::size_t node)
  {
    if (nr_neighbors >= cap_neighbors - 1)
    {
      cap_neighbors = 2 * cap_neighbors;
      neighbors = (std::size_t*)realloc(neighbors, cap_neighbors * sizeof(std::size_t));
    }
    neighbors[nr_neighbors] = node;
    nr_neighbors++;
  }

  /**
   * Deletes a node index from this node's neighbor list.
   * @brief Deletes a node index from this node's neighbor list.
   * @param node The index to delete.
   */
  void delete_neighbor(const std::size_t node)
  {
    int index_;
    for (index_ = 0; index_ < nr_neighbors; index_++)
    {
      if (neighbors[index_] == node)
      {
        break;
      }
    }
    assert(index_ < nr_neighbors);

    for (int i = index_; i < nr_neighbors - 1; i++)
    {
      neighbors[i] = neighbors[i + 1];
    }
    nr_neighbors--;
  }

  /**
   * Replaces a node index from this node's neighbor list.
   * @brief Replaces a node index from this node's neighbor list.
   * @param prev The index to look for.
   * @param new_index The index to replace with.
   */
  void replace_neighbor(const std::size_t prev, const std::size_t new_index)
  {
    int index_{ 0 };
    for (; index_ < nr_neighbors; index_++)
    {
      if (neighbors[index_] == prev)
      {
        break;
      }
    }
    // prx_assert( index_ < nr_neighbors , "Didn't find original index: "<<prev<<" to replace.");
    assert(index_ < nr_neighbors);

    neighbors[index_] = new_index;
  }

  void remove_all_neighbors()
  {
    nr_neighbors = 0;
    added_index = 0;
  }

  friend std::ostream& operator<<(std::ostream& os, const proximity_node_t<Point>& obj)
  {
    os << obj;
    return os;
  }

  std::size_t added_index;

  Point point;

  // protected:
  bool added_to_metric;

  /**
   * @brief Index in the data structure. Serves as an identifier to other nodes.
   */
  std::size_t prox_index;

  /**
   * @brief The max number of neighbors.
   */
  std::size_t cap_neighbors;

  /**
   * @brief The current number of neighbors.
   */
  std::size_t nr_neighbors;

  /**
   * @brief The neighbor list for this node.
   */
  std::size_t* neighbors;

  friend class graph_nearest_neighbors_t<Point, proximity_node_t<Point>>;
};

/**
 * A proximity structure based on graph literature. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief <b> A proximity structure based on graph literature. </b>
 * @author Kostas Bekris
 */
template <typename Point, typename Node = proximity_node_t<Point>>
class graph_nearest_neighbors_t
{
public:
  using Metric = std::function<double(const Point&, const Point&)>;

  /**
   * @brief Constructor
   * @param state The first node to add to the structure.
   */
  // graph_nearest_neighbors_t(distance_function_t);
  graph_nearest_neighbors_t(const Metric& metric, const std::size_t max_kk,
                            const std::size_t init_node_size = INIT_NODE_SIZE)
    : _metric(metric), added_node_id(0), nr_nodes(0), _max_kk(max_kk), cap_nodes(init_node_size)
  {
    nodes = (Node**)malloc(cap_nodes * sizeof(Node*));
    second_nodes = (Node**)malloc(_max_kk * sizeof(Node*));
    second_distances = (double*)malloc(_max_kk * sizeof(double));
    query_node = new Node();
  }

  graph_nearest_neighbors_t(const Metric& metric) : graph_nearest_neighbors_t(metric, MAX_KK, INIT_NODE_SIZE)
  {
  }

  ~graph_nearest_neighbors_t()
  {
    free(nodes);
    free(second_nodes);
    free(second_distances);
    delete query_node;
  }

  // Useful if query_node has some fancy initialization (aka Eigen::VectorXd)
  template <typename... NodeArgs>
  void init_query_point(NodeArgs... args)
  {
    query_node->point = Point(args...);
  }

  inline double node_distance(const Node* s1, const Node* s2) const
  {
    return _metric(s1->point, s2->point);
  }
  /**
   * Adds a node to the proximity structure
   * @brief Adds a node to the proximity structure
   * @param node The node to insert.
   */
  void add_node(Node* graph_node)
  {
    if (graph_node->added_to_metric)
    {
      prx_throw("Trying to add a node that is already added " << graph_node->get_prox_index());
    }
    int k = percolation_threshold();

    int new_k = find_k_close((Node*)(graph_node), second_nodes, second_distances, k);

    if (nr_nodes >= cap_nodes - 1)
    {
      cap_nodes = 2 * cap_nodes;
      nodes = (Node**)realloc(nodes, cap_nodes * sizeof(Node*));
    }
    nodes[nr_nodes] = graph_node;

    graph_node->set_index(nr_nodes);
    nr_nodes++;

    for (int i = 0; i < new_k; i++)
    {
      graph_node->add_neighbor(second_nodes[i]->get_prox_index());
      second_nodes[i]->add_neighbor(graph_node->get_prox_index());
    }
    graph_node->added_to_metric = true;
  }

  /**
   * @brief Removes a node from the structure.
   * @param node
   */
  void remove_node(Node* graph_node)
  {
    std::size_t nr_neighbors;
    std::size_t* neighbors = graph_node->get_neighbors(&nr_neighbors);
    for (int i = 0; i < nr_neighbors; i++)
    {
      nodes[neighbors[i]]->delete_neighbor(graph_node->get_prox_index());
    }
    graph_node->remove_all_neighbors();

    int index = graph_node->get_prox_index();
    if (index < nr_nodes - 1)
    {
      nodes[index] = nodes[nr_nodes - 1];
      nodes[index]->set_index(index);

      neighbors = nodes[index]->get_neighbors(&nr_neighbors);
      for (int i = 0; i < nr_neighbors; i++)
      {
        nodes[neighbors[i]]->replace_neighbor(nr_nodes - 1, index);
      }
    }
    nr_nodes--;
    graph_node->added_to_metric = false;

    if (nr_nodes < (cap_nodes - 1) / 2)
    {
      cap_nodes *= 0.5;
      nodes = (Node**)realloc(nodes, cap_nodes * sizeof(Node*));
    }
  }

  void clear()
  {
    for (std::size_t i = 0; i < nr_nodes; i++)
    {
      nodes[i]->nr_neighbors = 0;
    }
    nr_nodes = 0;
  }

  /**
   * Get function for number of nodes
   * @return Number of nodes in graph
   */
  std::size_t get_nr_nodes()
  {
    return nr_nodes;
  }

  Node* single_query(const Point& point)
  {
    query_node->point = point;
    double distance;
    return find_closest(query_node, &distance);
  }

  std::vector<Node*> multi_query(const Point& point, int k)
  {
    query_node->point = point;
    int new_k = find_k_close(query_node, second_nodes, second_distances, k);
    std::vector<Node*> ret(second_nodes, second_nodes + new_k);
    return std::move(ret);
  }

  template <typename QueryPoint>
  std::vector<Node*> radius_and_closest_query(const QueryPoint& point, const double rad)
  {
    // query_node->point = point;
    copy_query_point(point);
    int new_k = find_delta_close_and_closest(query_node, second_nodes, second_distances, rad);
    std::vector<Node*> ret(second_nodes, second_nodes + new_k);
    return ret;
  }

protected:
  const Metric _metric;

  template <typename ReturnType, typename Object, std::enable_if_t<prx::utils::is_ptr_type<Object>{}, bool> = true>
  inline ReturnType subscript_getter(Object& obj, const std::size_t idx)
  {
    return (*obj)[idx];
  }
  template <typename ReturnType, typename Object, std::enable_if_t<!prx::utils::is_ptr_type<Object>{}, bool> = true>
  inline ReturnType subscript_getter(Object& obj, const std::size_t idx)
  {
    return obj[idx];
  }
  template <typename Object, std::enable_if_t<prx::utils::is_ptr_type<Object>{}, bool> = true>
  inline std::size_t get_size(Object& obj)
  {
    return obj->size();
  }
  template <typename Object, std::enable_if_t<!prx::utils::is_ptr_type<Object>{}, bool> = true>
  inline std::size_t get_size(Object& obj)
  {
    return obj.size();
  }

  template <typename QueryPoint, typename Pt = Point,
            std::enable_if_t<std::is_same<Pt, QueryPoint>::value, bool> = true>
  void copy_query_point(const QueryPoint& point)
  {
    query_node->point = point;
  }
  template <typename QueryPoint, typename Pt = Point,
            std::enable_if_t<!std::is_same<Pt, QueryPoint>::value, bool> = true>
  void copy_query_point(const QueryPoint& point)
  {
    const std::size_t pt_size{ get_size(point) };
    // prx_assert(query_node->point.size() == point.size(), "Mismatch points");
    for (int i = 0; i < pt_size; ++i)
    {
      subscript_getter<double&>(query_node->point, i) = subscript_getter<const double>(point, i);
    }
  }
  /**
   * Returns the closest node in the data structure.
   * @brief Returns the closest node in the data structure.
   * @param state The query point.
   * @param distance The resulting distance between the closest point and the query point.
   * @return The closest point.
   */
  Node* find_closest(Node* state, double* distance)
  {
    long unsigned min_index = -1;
    return basic_closest_search(state, distance, &min_index);
  }

  /**
   * Find the k closest nodes to the query point.
   * @brief Find the k closest nodes to the query point.
   * @param state The query state.
   * @param close_nodes The returned close nodes.
   * @param distances The corresponding distances to the query point.
   * @param k The number to return.
   * @return The number of nodes actually returned.
   */
  int find_k_close(Node* state, Node** close_nodes, double* distances, int k)
  {
    if (nr_nodes == 0)
    {
      return 0;
    }

    if (k > _max_kk)
    {
      k = _max_kk;
    }
    else if (k >= nr_nodes)
    {
      for (int i = 0; i < nr_nodes; i++)
      {
        close_nodes[i] = nodes[i];
        distances[i] = node_distance(nodes[i], state);
      }
      sort_proximity_nodes(close_nodes, distances, 0, nr_nodes - 1);
      return nr_nodes;
    }

    clear_added();

    long unsigned min_index = -1;
    close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);
    nodes[min_index]->added_index = added_node_id;

    min_index = 0;
    int nr_elements = 1;
    double max_distance = distances[0];

    /* Find the neighbors of the closest node if they are not already in the set of k-closest nodes.
    If the distance to any of the neighbors is less than the distance to the k-th closest element,
    then replace the last element with the neighbor and resort the list. In order to decide the next
    node to pivot about, it is either the next node on the list of k-closest
    */
    do
    {
      long unsigned nr_neighbors;
      long unsigned* neighbors = nodes[close_nodes[min_index]->get_prox_index()]->get_neighbors(&nr_neighbors);
      int lowest_replacement = nr_elements;

      for (int j = 0; j < nr_neighbors; j++)
      {
        Node* the_neighbor = nodes[neighbors[j]];
        if (does_node_exist(the_neighbor) == false)
        {
          the_neighbor->added_index = added_node_id;

          double distance = node_distance(the_neighbor, state);
          bool to_resort = false;
          if (nr_elements < k)
          {
            close_nodes[nr_elements] = the_neighbor;
            distances[nr_elements] = distance;
            nr_elements++;
            to_resort = true;
          }
          else if (distance < distances[k - 1])
          {
            close_nodes[k - 1] = the_neighbor;
            distances[k - 1] = distance;
            to_resort = true;
          }

          if (to_resort)
          {
            int test = resort_proximity_nodes(close_nodes, distances, nr_elements - 1);
            lowest_replacement = (test < lowest_replacement ? test : lowest_replacement);
          }
        }
      }

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
    } while (min_index < nr_elements);

    return nr_elements;
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
  int find_delta_close_and_closest(Node* state, Node** close_nodes, double* distances, const double delta)
  {
    if (nr_nodes == 0)
    {
      return 0;
    }

    clear_added();

    long unsigned min_index = -1;
    close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);

    if (distances[0] > delta)
    {
      return 1;
    }

    nodes[min_index]->added_index = added_node_id;

    int nr_points = 1;
    for (int counter = 0; counter < nr_points; counter++)
    {
      std::size_t nr_neighbors;
      std::size_t* neighbors = close_nodes[counter]->get_neighbors(&nr_neighbors);
      for (int j = 0; j < nr_neighbors; j++)
      {
        Node* the_neighbor = nodes[neighbors[j]];
        if (does_node_exist(the_neighbor) == false)
        {
          the_neighbor->added_index = added_node_id;
          double distance = node_distance(the_neighbor, state);
          if (distance < delta && nr_points < _max_kk)
          {
            close_nodes[nr_points] = the_neighbor;
            distances[nr_points] = distance;
            nr_points++;
          }
        }
      }
    }

    if (nr_points > 0)
    {
      sort_proximity_nodes(close_nodes, distances, 0, nr_points - 1);
    }

    return nr_points;
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
  int find_delta_close(Node* state, Node** close_nodes, double* distances, double delta)
  {
    if (nr_nodes == 0)
    {
      return 0;
    }

    clear_added();

    long unsigned min_index = -1;
    close_nodes[0] = basic_closest_search(state, &(distances[0]), &min_index);

    if (distances[0] > delta)
    {
      return 0;
    }

    nodes[min_index]->added_index = added_node_id;

    int nr_points = 1;
    for (int counter = 0; counter < nr_points; counter++)
    {
      long unsigned nr_neighbors;
      long unsigned* neighbors = close_nodes[counter]->get_neighbors(&nr_neighbors);
      for (int j = 0; j < nr_neighbors; j++)
      {
        Node* the_neighbor = nodes[neighbors[j]];
        if (does_node_exist(the_neighbor) == false)
        {
          the_neighbor->added_index = added_node_id;
          double distance = node_distance(the_neighbor, state);
          if (distance < delta && nr_points < _max_kk)
          {
            close_nodes[nr_points] = the_neighbor;
            distances[nr_points] = distance;
            nr_points++;
          }
        }
      }
    }

    if (nr_points > 0)
    {
      sort_proximity_nodes(close_nodes, distances, 0, nr_points - 1);
    }

    return nr_points;
  }

  /**
   * Determine the number of nodes to sample for initial populations in queries.
   * @brief Determine the number of nodes to sample for initial populations in queries.
   * @return The number of random nodes to initially select.
   */
  inline std::size_t sampling_function()
  {
    if (nr_nodes < 200)
      return nr_nodes;
    else
      return 200;
  }

  /**
   * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
   * @return
   */
  inline std::size_t percolation_threshold()
  {
    if (nr_nodes > 12)
      return (2.0 * log(nr_nodes));
    else
      return nr_nodes;
  }

  /**
   * Sorts a list of Node's. Performed using a quick sort operation.
   * @param close_nodes The list to sort.
   * @param distances The distances that determine the ordering.
   * @param low The lower index.
   * @param high The upper index.
   */
  void sort_proximity_nodes(Node** close_nodes, double* distances, int low, int high)
  {
    if (low < high)
    {
      int left, right, pivot;
      double pivot_distance = distances[low];
      Node* pivot_node = close_nodes[low];

      double temp;
      Node* temp_node;

      pivot = left = low;
      right = high;
      while (left < right)
      {
        while (left <= high && distances[left] <= pivot_distance)
        {
          left++;
        }
        while (distances[right] > pivot_distance)
        {
          right--;
        }
        if (left < right)
        {
          temp = distances[left];
          distances[left] = distances[right];
          distances[right] = temp;

          temp_node = close_nodes[left];
          close_nodes[left] = close_nodes[right];
          close_nodes[right] = temp_node;
        }
      }
      distances[low] = distances[right];
      distances[right] = pivot_distance;

      close_nodes[low] = close_nodes[right];
      close_nodes[right] = pivot_node;

      sort_proximity_nodes(close_nodes, distances, low, right - 1);
      sort_proximity_nodes(close_nodes, distances, right + 1, high);
    }
  }

  /**
   * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
   * @param close_nodes The list to sort.
   * @param distances The distances that determine the ordering.
   * @param index The index to start from.
   */
  int resort_proximity_nodes(Node** close_nodes, double* distances, int index_)
  {
    double temp;
    Node* temp_node;

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

  /**
   * Helper function for determining existance in a list.
   * @brief Helper function for determining existance in a list.
   * @param query_node The node to search for.
   * @param node_list The list to search.
   * @param list_size The size of the list.
   * @return If query_node exists in node_list.
   */
  inline bool does_node_exist(Node* query_node) const
  {
    return query_node->added_index == added_node_id;
  }

  /**
   * The basic search process for finding the closest node to the query state.
   * @brief Find the closest node to the query state.
   * @param state The query state.
   * @param distance The corresponding distance to the query point.
   * @param node_index The index of the returned node.
   * @return The closest node.
   */
  Node* basic_closest_search(Node* state, double* the_distance, std::size_t* the_index)
  {
    if (nr_nodes == 0)
    {
      return nullptr;
    }

    std::size_t nr_neighbors;
    std::size_t nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();
    std::size_t min_index = -1;
    for (int i = 0; i < nr_samples; i++)
    {
      int index_ = rand() % nr_nodes;
      double distance = node_distance(nodes[index_], state);
      if (distance < min_distance)
      {
        min_distance = distance;
        min_index = index_;
      }
    }

    int old_min_index = min_index;
    do
    {
      old_min_index = min_index;
      std::size_t* neighbors = nodes[min_index]->get_neighbors(&nr_neighbors);
      for (int j = 0; j < nr_neighbors; j++)
      {
        const double distance = node_distance(nodes[neighbors[j]], state);
        if (distance < min_distance)
        {
          min_distance = distance;
          min_index = neighbors[j];
        }
      }
    } while (old_min_index != min_index);

    *the_distance = min_distance;
    *the_index = min_index;
    return nodes[min_index];
  }

  inline void clear_added()
  {
    added_node_id++;
  }

  /**
   * @brief The nodes being stored.
   */
  Node** nodes;

  /**
   * @brief The current number of nodes being stored.
   */
  std::size_t nr_nodes;

  /**
   * @brief The maximum number of nodes that can be stored.
   */
  std::size_t cap_nodes;

  /**
   * @brief Temporary storage for query functions.
   */
  Node** second_nodes;

  /**
   * @brief Temporary storage for query functions.
   */
  double* second_distances;

  // std::vector<Node*> added_nodes;

  std::size_t added_node_id;

  Node* query_node;

  std::size_t _max_kk;
};
}  // namespace prx