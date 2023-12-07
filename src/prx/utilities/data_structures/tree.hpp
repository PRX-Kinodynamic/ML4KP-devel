#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <unordered_map>
#include <list>
#include <fstream>
namespace prx
{
class tree_t;

enum tree_node_status
{
  IDLE,
  ACTIVE,
  MARKED_FOR_REMOVAL
};
/**
 * @brief <b> A node on a tree. </b>
 *
 * @author Zakary Littlefield
 */
class tree_node_t : public abstract_node_t
{
public:
  tree_node_t() : abstract_node_t()
  {
    status = tree_node_status::IDLE;
  }
  virtual ~tree_node_t()
  {
  }

  /**
   * @brief Returns the index of the parent node.
   *
   * @return Index of the parent node.
   */
  node_index_t get_parent() const
  {
    return parent;
  }
  /**
   * @brief Returns the index of the tree node.
   *
   * @return Index of the node.
   */
  node_index_t get_index() const
  {
    return index;
  }
  /**
   * @brief Returns the index of the tree edge to this node.
   *
   * @return Index of the edge from parent -> current node.
   */
  edge_index_t get_parent_edge() const
  {
    return parent_edge;
  }

  /**
   * @brief Returns the children of the current node.
   *
   * @return A list containing the node indices of the child nodes.
   */
  const std::list<node_index_t>& get_children() const
  {
    return children;
  }

  friend std::ostream& operator<<(std::ostream& os, const tree_node_t* obj)
  {
    os << *obj;
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<tree_node_t> obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const tree_node_t& obj)
  {
    os << "[TreeNode] parent" << prx::constants::separating_value;
    os << obj.parent << prx::constants::separating_value;
    os << "edge" << prx::constants::separating_value;
    os << obj.parent_edge << prx::constants::separating_value;
    os << "index" << prx::constants::separating_value;
    os << obj.index << prx::constants::separating_value;
    os << "State:" << prx::constants::separating_value;
    if (obj.point == nullptr)
    {
      os << "nullptr";
    }
    else
    {
      os << obj.point << prx::constants::separating_value;
    }
    os << "Children:";
    for (auto child : obj.children)
    {
      os << child << prx::constants::separating_value;
    }
    return os;
  }

protected:
  node_index_t parent;
  edge_index_t parent_edge;
  node_index_t index;
  tree_node_status status;

  std::list<node_index_t> children;

  friend class tree_t;
};

/**
 * @brief <b> An edge of a tree. </b>
 *
 * @author Zakary Littlefield
 */
class tree_edge_t : public abstract_edge_t
{
public:
  tree_edge_t()
  {
  }

  virtual ~tree_edge_t()
  {
  }

  /**
   * @brief Returns the index of the tree edge.
   *
   * @return Index of the tree edge.
   */
  edge_index_t get_index() const
  {
    return index;
  }

  /**
   * @brief Returns the index of the source node.
   *
   * @return Index of the parent or source node of this edge.
   */
  node_index_t get_source() const
  {
    return source;
  }

  /**
   * @brief Returns the index of the target node.
   *
   * @return Index of the child or target node of this edge.
   */
  node_index_t get_target() const
  {
    return target;
  }
  friend std::ostream& operator<<(std::ostream& os, const tree_edge_t* obj)
  {
    os << *obj;
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<tree_edge_t> obj)
  {
    os << *obj;
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, const tree_edge_t& obj)
  {
    os << "[TreeEdge] source" << prx::constants::separating_value;
    os << obj.source << prx::constants::separating_value;
    os << "index" << prx::constants::separating_value;
    os << obj.index << prx::constants::separating_value;
    os << "target" << prx::constants::separating_value;
    os << obj.target << prx::constants::separating_value;
    return os;
  }

protected:
  node_index_t source;
  node_index_t target;

  edge_index_t index;
  friend class tree_t;
};

/**
 *
 * @brief <b> A tree data structure. </b>
 *
 * @author Zakary Littlefield
 */
class tree_t
{
public:
  typedef std::list<std::shared_ptr<tree_node_t>>::const_iterator const_vertex_iterator;
  typedef std::list<std::shared_ptr<tree_edge_t>>::const_iterator const_edge_iterator;
  typedef std::list<std::shared_ptr<tree_node_t>>::iterator vertex_iterator;
  typedef std::list<std::shared_ptr<tree_edge_t>>::iterator edge_iterator;

  template <class node_type, class edge_type>
  void allocate_memory(uint64_t new_size)
  {
    uint64_t old_size = vertex_count;
    if (max_count < new_size)
    {
      v_index_map.resize(new_size);
      e_index_map.resize(new_size);
      for (uint64_t i = max_count; i < new_size; i++)
      {
        vertex_list.insert(vertex_list.end(), std::make_shared<node_type>());
        edge_list.insert(edge_list.end(), std::make_shared<edge_type>());
      }
      max_count = new_size;
    }
    if (old_size == 0)
    {
      v_iter = vertex_list.begin();
      e_iter = edge_list.begin();
      const_v_iter = vertex_list.begin();
      const_e_iter = edge_list.begin();
    }
    else
    {
      v_iter = vertex_list.begin();
      e_iter = edge_list.begin();
      const_v_iter = vertex_list.begin();
      const_e_iter = edge_list.begin();
      std::advance(v_iter, old_size);
      std::advance(const_v_iter, old_size);
      std::advance(e_iter, old_size - 1);
      std::advance(const_e_iter, old_size - 1);
    }
  }

  template <class node_type, class edge_type>
  node_index_t add_vertex()
  {
    if (vertex_id_counter == max_count)
    {
      allocate_memory<node_type, edge_type>(vertex_list.size() * 2 + 1);
    }
    auto node = *v_iter;
    node->index = vertex_id_counter;
    node->parent = vertex_id_counter;
    v_index_map[vertex_id_counter] = node;
    vertex_id_counter++;
    v_iter++;
    const_v_iter++;
    vertex_count++;
    node->status = tree_node_status::ACTIVE;
    return node->index;
  }

  template <class node_type>
  std::shared_ptr<node_type> get_vertex_as(node_index_t v) const
  {
    return std::dynamic_pointer_cast<node_type>(v_index_map[v]);
  }

  template <class edge_type>
  std::shared_ptr<edge_type> get_edge_as(edge_index_t e) const
  {
    return std::dynamic_pointer_cast<edge_type>(e_index_map[e]);
  }

  std::pair<const_vertex_iterator, const_vertex_iterator> vertices() const
  {
    return std::make_pair(vertex_list.begin(), const_v_iter);
  }

  std::pair<const_edge_iterator, const_edge_iterator> edges() const
  {
    return std::make_pair(edge_list.begin(), const_e_iter);
  }
  std::shared_ptr<tree_node_t> operator[](node_index_t v) const
  {
    return v_index_map[v];
  }
  unsigned num_vertices() const
  {
    return vertex_count;
  }

  unsigned num_edges() const
  {
    return edge_count;
  }

  bool is_leaf(node_index_t v)
  {
    return (v_index_map[v]->get_children().size() == 0);
  }

  edge_index_t edge(node_index_t from, node_index_t to) const
  {
    return v_index_map[to]->parent_edge;
  }

  tree_t();
  ~tree_t();

  edge_index_t add_edge(node_index_t from, node_index_t to);

  unsigned get_depth(node_index_t v);

  void remove_vertex(node_index_t v);

  void purge();

  void clear();

  uint64_t capacity()
  {
    return max_count;
  }

  // void rewire(node_index_t root, node_index_t new_parent);

  void transplant(node_index_t root, node_index_t new_parent);

  void mark_vertex_for_removal(node_index_t v);

  void remove_vertices();

  // Write the tree to a file with format:
  // parent_id edge_id node_id node_state
  void to_file(const std::string);

  template <class node_type, class edge_type>
  void from_file(const std::string filename_tree, space_t* space)
  {
    using prx::utilities::csv_reader_t;
    using Line = std::vector<std::string>;
    csv_reader_t reader_tree(filename_tree);
    reader_tree.next_line();  // Remove first line (header)
    while (reader_tree.has_next_line())
    {
      Line line{ reader_tree.next_line() };
      if (line.size() == 0)
      {
        break;
      }

      const node_index_t parent_idx{ prx::utilities::convert_to<std::size_t>(line[0]) };
      const edge_index_t edge_idx{ prx::utilities::convert_to<std::size_t>(line[1]) };
      const node_index_t node_idx{ prx::utilities::convert_to<std::size_t>(line[2]) };
      const Line node_state(line.begin() + 3, line.end());

      const std::size_t current_size{ v_index_map.size() };

      uint64_t max_idx{ std::max(parent_idx, node_idx) };
      // uint64_t
      while (max_idx >= vertex_id_counter)
      {
        add_vertex<node_type, edge_type>();
      }
      std::shared_ptr<node_type> node{ get_vertex_as<node_type>(node_idx) };
      std::shared_ptr<node_type> parent_node{ get_vertex_as<node_type>(parent_idx) };

      node->index = node_idx;
      node->point = space->make_point();
      space->copy(node->point, node_state);

      v_index_map[node_idx] = node;

      // The root is its own parent
      if (parent_idx == node_idx)
      {
        node->parent = node_idx;
      }
      else
      {
        // add_edge(parent_idx, node_idx);
        v_index_map[parent_idx]->children.push_back(node_idx);
        v_index_map[node_idx]->parent = parent_idx;

        // Edge has been added as dummy

        while (edge_id_counter <= edge_idx)
        {
          auto edge = *e_iter;

          edge->index = edge_id_counter;
          e_index_map[edge_id_counter] = edge;
          edge_id_counter++;
          e_iter++;
          const_e_iter++;
          edge_count++;
        }
        e_index_map[edge_idx]->source = parent_idx;
        e_index_map[edge_idx]->target = node_idx;
        v_index_map[node_idx]->parent_edge = e_index_map[edge_idx]->index;
      }
    }
  }

  uint64_t vertex_id_counter;

protected:
  std::list<std::shared_ptr<tree_node_t>> vertex_list;
  std::list<std::shared_ptr<tree_edge_t>> edge_list;
  vertex_iterator v_iter;
  edge_iterator e_iter;
  const_vertex_iterator const_v_iter;
  const_edge_iterator const_e_iter;
  uint64_t vertex_count;
  uint64_t edge_count;
  uint64_t max_count;
  uint64_t edge_id_counter;
  uint64_t nodes_to_remove;

  std::vector<std::shared_ptr<tree_node_t>> v_index_map;
  std::vector<std::shared_ptr<tree_edge_t>> e_index_map;
};
}  // namespace prx
