#pragma once

#include "prx/utilities/defs.hpp"

#include <unordered_map>
#include <list>

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

  // edge_index_t edge(node_index_t from, node_index_t to) const
  // {
  //   return v_index_map[to]->parent_edge;
  // }

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

  void transplant(node_index_t root, node_index_t new_parent);

  void mark_vertex_for_removal(node_index_t v);

  void remove_vertices();

  unsigned vertex_id_counter;

protected:
  std::list<std::shared_ptr<tree_node_t>> vertex_list;
  std::list<std::shared_ptr<tree_edge_t>> edge_list;
  vertex_iterator v_iter;
  edge_iterator e_iter;
  const_vertex_iterator const_v_iter;
  const_edge_iterator const_e_iter;
  std::size_t vertex_count;
  std::size_t edge_count;
  std::size_t max_count;
  std::size_t edge_id_counter;
  std::size_t nodes_to_remove;

  std::vector<std::shared_ptr<tree_node_t>> v_index_map;
  std::vector<std::shared_ptr<tree_edge_t>> e_index_map;
};