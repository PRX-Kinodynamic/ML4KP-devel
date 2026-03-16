#pragma once

#include "prx/utilities/defs.hpp"
// #include "prx/utilities/data_structures/abstract_node.hpp"
// #include "prx/utilities/data_structures/abstract_edge.hpp"
// #include "prx/utilities/general/csv_reader.hpp"
// #include "prx/utilities/spaces/space.hpp"

#include <iterator>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <list>

// Template implementation of a Tree, Nodes and Edges
// Tree<Node, Edge> can be constructed with *other* Nodes/Edges
// but need to have certain operations (children, parent, parent_edge...)
// This Tree implementation has:
//  * Amortized constant insertions (May need memory reallocation)
//  * Constant removal of leaf nodes (Branch removal not supported)
//  * Insertions are not necessarily in order -- Removed indices may be reused for newly inserted nodes.
namespace prx
{

class fast_tree_node_t
{
public:
  using Index = std::size_t;

  fast_tree_node_t()
  {
  }
  virtual ~fast_tree_node_t()
  {
  }

  void parent(const Index new_parent)
  {
    _parent_idx = new_parent;
  }

  Index parent() const
  {
    return _parent_idx;
  }

  void index(const Index new_index)
  {
    _index = new_index;
  }

  Index index() const
  {
    return _index;
  }

  void parent_edge(const Index new_edge_index)
  {
    _parent_edge_idx = new_edge_index;
  }

  Index parent_edge() const
  {
    return _parent_edge_idx;
  }

  std::list<Index>& children()
  {
    return _children;
  }

  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<fast_tree_node_t> obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const fast_tree_node_t& obj)
  {
    os << obj._parent_idx << " ";
    os << obj._parent_edge_idx << " ";
    os << obj._index << " ";

    for (auto child : obj._children)
    {
      os << child << " ";
    }
    return os;
  }

protected:
  Index _parent_idx;
  Index _parent_edge_idx;
  Index _index;

  std::list<Index> _children;
};

class fast_tree_edge_t
{
public:
  using Index = std::size_t;

  fast_tree_edge_t()
  {
  }

  virtual ~fast_tree_edge_t()
  {
  }

  void index(const Index new_index)
  {
    _index = new_index;
  }
  Index index() const
  {
    return _index;
  }

  void source(const Index new_source)
  {
    _source_idx = new_source;
  }

  Index source() const
  {
    return _source_idx;
  }

  void target(const Index new_target)
  {
    _target_idx = new_target;
  }

  Index target() const
  {
    return _target_idx;
  }

  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<fast_tree_edge_t> obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const fast_tree_edge_t& obj)
  {
    os << obj._source_idx << prx::constants::separating_value;
    os << obj._index << prx::constants::separating_value;
    os << obj._target_idx << prx::constants::separating_value;
    return os;
  }

protected:
  Index _source_idx;
  Index _target_idx;

  Index _index;
};

/**
 *
 * @brief <b> A tree data structure. </b>
 *
 * @author Edgar Granados
 */
template <typename Node, typename Edge>
class fast_tree_t
{
public:
  using NodeIndex = typename Node::Index;
  using EdgeIndex = typename Edge::Index;

  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;

  // using ConstNodeIterator = std::list<NodePtr>::const_iterator;
  // using NodeIterator = std::list<NodePtr>::iterator;
  // using ConstEdgeIterator = std::list<EdgePtr>::const_iterator;
  // using EdgeIterator = std::list<EdgePtr>::iterator;

  fast_tree_t(const std::size_t initial_allocation)
    : _total_nodes(0), _total_allocated(0), _next_node_idx(0), _total_edges(0), _next_edge_idx(0)
  {
    allocate_memory(initial_allocation);
  }

  fast_tree_t() : fast_tree_t(1000)
  {
  }

  ~fast_tree_t()
  {
  }

  void allocate_memory(const std::size_t nodes_to_allocate)
  {
    // const std::size_t old_node_count{ _total_nodes };
    // const std::size_t old_edge_count{ _total_nodes - 1 };

    for (std::size_t i = 0; i < nodes_to_allocate; i++)
    {
      _preallocated_nodes.emplace_back(new Node());
      _preallocated_edges.emplace_back(new Edge());
    }
    _total_allocated += nodes_to_allocate;

    _nodes.resize(_total_allocated);
    _edges.resize(_total_allocated);
  }

  NodeIndex add_node()
  {
    if (_preallocated_nodes.size() == 0)
    {
      allocate_memory(_nodes.size());  // Doubles the capacity
    }

    NodePtr new_node{ _preallocated_nodes.front() };
    _preallocated_nodes.pop_front();

    std::size_t new_node_index;
    if (_empty_nodes_indices.size() > 0)
    {
      new_node_index = _empty_nodes_indices.front();
      _empty_nodes_indices.pop_front();
    }
    else
    {
      new_node_index = _next_node_idx;
      _next_node_idx++;
    }
    new_node->index(new_node_index);
    new_node->parent(new_node_index);  // No parent
    _nodes[new_node_index] = new_node;
    _total_nodes++;

    return new_node->index();
  }

  NodePtr node(const NodeIndex nid) const
  {
    return _nodes[nid];
  }

  EdgePtr edge(const EdgeIndex eid) const
  {
    return _edges[eid];
  }

  NodeIndex size() const
  {
    return _total_nodes;
  }
  NodeIndex total_nodes() const
  {
    return _total_nodes;
  }

  EdgeIndex total_edges() const
  {
    return _total_edges;
  }

  EdgeIndex add_edge(const NodeIndex from, const NodeIndex to)
  {
    prx_assert(_nodes[to]->parent() == to,
               "The node with index [" << to << "] already has a parent node [" << _nodes[to] << "].");
    // prx_assert(edge_count != max_count, "There would now be more edges than vertices in the tree. This cannot
    // happen.");

    _nodes[from]->children().push_back(to);
    _nodes[to]->parent(from);

    EdgePtr new_edge{ _preallocated_edges.front() };
    _preallocated_edges.pop_front();

    std::size_t new_edge_index;
    if (_empty_edges_indices.size() > 0)
    {
      new_edge_index = _empty_edges_indices.front();
      _empty_edges_indices.pop_front();
    }
    else
    {
      new_edge_index = _next_edge_idx;
      _next_edge_idx++;
    }

    new_edge->index(new_edge_index);
    new_edge->source(from);
    new_edge->target(to);
    _edges[new_edge_index] = new_edge;
    _nodes[to]->parent_edge(new_edge_index);
    _total_edges++;

    return new_edge->index();
  }

  // unsigned get_depth(node_index_t v);

  void remove_node(NodePtr node_to_remove)
  {
    remove_node(node_to_remove->index());
  }

  void remove_node(const NodeIndex idx_to_remove)
  {
    NodePtr node_to_remove{ _nodes[idx_to_remove] };
    prx_assert(node_to_remove->children().size() == 0, "Can only remove a node if it doesn't have any children.");

    const NodeIndex parent{ node_to_remove->parent() };
    const EdgeIndex parent_edge{ node_to_remove->parent_edge() };

    _nodes[parent]->children().remove(idx_to_remove);

    node_to_remove->parent(idx_to_remove);

    _nodes[idx_to_remove] = nullptr;
    _empty_nodes_indices.push_back(idx_to_remove);
    _total_nodes--;

    // No need to remove edge if node has not been assigned an edge
    if (parent_edge != idx_to_remove)
    {
      _edges[parent_edge] = nullptr;
      _empty_edges_indices.push_back(parent_edge);
      _total_edges--;
    }
  }

  void clear()
  {
    _preallocated_nodes.clear();
    _preallocated_edges.clear();
    _total_nodes = 0;
    _total_allocated = 0;
    _next_node_idx = 0;
    _next_edge_idx = 0;
    _nodes.clear();
    _edges.clear();
    _empty_nodes_indices.clear();
    _empty_edges_indices.clear();
  }

  // Total allocated memory != nodes in tree
  std::size_t capacity()
  {
    return _total_allocated;
  }

  // void mark_vertex_for_removal(const node_index_t& v);

  // void remove_vertices();

  // Write the tree to a file with format:
  // parent_id edge_id node_id node_state
  // void to_file(const std::string);

  friend void swap(fast_tree_t& lhs, fast_tree_t& rhs)
  {
    std::swap(lhs._preallocated_nodes, rhs._preallocated_nodes);
    std::swap(lhs._preallocated_edges, rhs._preallocated_edges);
    std::swap(lhs._total_nodes, rhs._total_nodes);
    std::swap(lhs._total_allocated, rhs._total_allocated);
    std::swap(lhs._next_node_idx, rhs._next_node_idx);
    std::swap(lhs._next_edge_idx, rhs._next_edge_idx);
    std::swap(lhs._nodes, rhs._nodes);
    std::swap(lhs._edges, rhs._edges);
    std::swap(lhs._empty_nodes_indices, rhs._empty_nodes_indices);
    std::swap(lhs._empty_edges_indices, rhs._empty_edges_indices);
  }

protected:
  std::list<NodePtr> _preallocated_nodes;
  std::list<EdgePtr> _preallocated_edges;

  // _nodes may contain nullptr's --> _total_nodes = _nodes.size() - number of nullptrs
  std::size_t _total_nodes;      // No need for edges: total_edges = total_nodes - 1
  std::size_t _total_allocated;  // Total allocated nodes >= _preallocated_nodes.size()
  std::size_t _next_node_idx;
  std::size_t _next_edge_idx;
  std::size_t _total_edges;

  std::vector<NodePtr> _nodes;
  std::vector<EdgePtr> _edges;

  std::list<NodeIndex> _empty_nodes_indices;
  std::list<NodeIndex> _empty_edges_indices;
};
}  // namespace prx
