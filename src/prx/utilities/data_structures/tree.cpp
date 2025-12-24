
#include "prx/utilities/data_structures/tree.hpp"

namespace prx
{
tree_t::tree_t()
{
  vertex_count = 0;
  edge_count = 0;
  max_count = 0;
  vertex_id_counter = 0;
  edge_id_counter = 0;
  nodes_to_remove = 0;
}
tree_t::~tree_t()
{
}

edge_index_t tree_t::add_edge(node_index_t from, node_index_t to)
{
  prx_assert(v_index_map[to]->parent == to,
             "The node with index [" << to << "] already has a parent node [" << v_index_map[to] << "].");
  prx_assert(edge_count != max_count, "There would now be more edges than vertices in the tree. This cannot happen.");

  v_index_map[from]->children.insert(v_index_map[from]->children.begin(), to);
  v_index_map[to]->parent = from;

  std::shared_ptr<tree_edge_t> edge{ *e_iter };
  edge->index = edge_id_counter;
  edge->source = from;
  edge->target = to;
  e_index_map[edge_id_counter] = edge;
  edge_id_counter++;
  e_iter++;
  const_e_iter++;
  edge_count++;
  v_index_map[to]->parent_edge = edge->index;

  return edge->index;
}

unsigned tree_t::get_depth(node_index_t v)
{
  if (v_index_map[v]->parent == v)
    return 0;
  unsigned depth = 1;
  while (v_index_map[v]->parent != v_index_map[v_index_map[v]->parent]->parent)
  {
    depth = depth + 1;
    v = v_index_map[v]->parent;
  }
  return depth;
}

void tree_t::remove_vertex(node_index_t v)
{
  prx_assert(v_index_map[v]->children.size() == 0, "Can only remove a vertex if it doesn't have any children.");
  if (v_index_map[v]->status == tree_node_status::MARKED_FOR_REMOVAL)
  {
    nodes_to_remove--;
  }

  v_index_map[v_index_map[v]->parent]->children.remove(v);
  edge_index_t e = v_index_map[v]->parent_edge;

  auto temp_v = v_index_map[v];
  temp_v->parent = temp_v->index;
  v_index_map[v] = nullptr;
  auto v_iterator = std::find(vertex_list.begin(), v_iter, temp_v);
  v_iter--;
  const_v_iter--;
  *v_iterator = *v_iter;
  *v_iter = temp_v;
  vertex_count--;

  auto temp_e = e_index_map[e];
  auto e_iterator = std::find(edge_list.begin(), e_iter, temp_e);
  e_index_map[e] = nullptr;
  e_iter--;
  const_e_iter--;
  *e_iterator = *e_iter;
  *e_iter = temp_e;
  edge_count--;
}

void tree_t::mark_vertex_for_removal(const node_index_t& v)
{
  prx_assert(v < v_index_map.size(), "Node index " << v << " out or range.");
  // std::cout << "Marking for removal: " << v << std::endl;
  v_index_map[v]->status = tree_node_status::MARKED_FOR_REMOVAL;
  _invalid_nodes.insert(v);
  nodes_to_remove++;
}

void tree_t::remove_vertices()
{
  prx_assert(nodes_to_remove <= vertex_count,
             "More nodes to removed (" << nodes_to_remove << ") than nodes in the tree " << vertex_count << " !");
  std::unordered_map<edge_index_t, bool> edges_to_delete{};
  uint64_t total_edges_to_delete{ 0 };  // could be using edges_to_delete.erase(), but this guarantes cte time
  edges_to_delete.reserve(nodes_to_remove);
  auto vertex_iterator = vertex_list.begin();
  while (nodes_to_remove > 0)
  {
    std::shared_ptr<tree_node_t> temp_v = *vertex_iterator;
    node_index_t node_index = temp_v->index;

    if (temp_v->status == tree_node_status::MARKED_FOR_REMOVAL)
    {
      // If parent is nullptr, this should to be the root
      if (vertex_count > 1)
      {
        edges_to_delete[v_index_map[node_index]->parent_edge] = true;
        total_edges_to_delete++;
      }
      if (v_index_map[temp_v->parent] != nullptr)
      {
        v_index_map[temp_v->parent]->children.remove(node_index);
      }
      for (auto child_index : temp_v->children)
      {
        // std::cout << "\tChild is: " << child_index << std::endl;
        // std::cout << "\tstatus: " << v_index_map[child_index]->status << std::endl;
        prx_assert(v_index_map[child_index]->status == tree_node_status::MARKED_FOR_REMOVAL,
                   "Error removing vertex [" << node_index << "]: "                                    // no-lint
                                             << "A vertex can only be removed if it has no children "  // no-lint
                                                "or all have also been marked for removal.");          // no-lint
      }

      if (vertex_count > 0)
      {
        temp_v->parent = temp_v->index;
        v_index_map[node_index] = nullptr;
        vertex_iterator = vertex_list.erase(vertex_iterator);
        vertex_count--;
      }
      nodes_to_remove--;
    }
    else
    {
      vertex_iterator++;
    }
  }

  auto edge_iterator = edge_list.begin();
  while (total_edges_to_delete > 0)
  {
    const edge_index_t e = (*edge_iterator)->index;
    if (edges_to_delete[e])
    {
      auto temp_e = e_index_map[e];
      e_index_map[e] = nullptr;
      edge_iterator = edge_list.erase(edge_iterator);
      edge_count--;
      total_edges_to_delete--;
    }
    else
    {
      edge_iterator++;
    }
  }
}

void tree_t::purge()
{
  clear();
  vertex_list.clear();
  edge_list.clear();
  v_index_map.clear();
  e_index_map.clear();
}

void tree_t::clear()
{
  for (auto e : edge_list)
  {
    e->source = e->target = 0;
  }
  for (auto v : vertex_list)
  {
    v->children.clear();
    v->parent = v->index = 0;
  }
  vertex_count = 0;
  edge_count = 0;
  edge_id_counter = 0;
  vertex_id_counter = 0;
  max_count = 0;
  edge_list.clear();
  vertex_list.clear();
  v_iter = vertex_list.begin();
  e_iter = edge_list.begin();
  const_v_iter = vertex_list.begin();
  const_e_iter = edge_list.begin();
}

void tree_t::transplant(node_index_t node_idx, node_index_t new_parent_idx)
{
  const node_index_t old_parent{ v_index_map[node_idx]->parent };
  const edge_index_t parent_edge_idx{ v_index_map[node_idx]->parent_edge };
  std::shared_ptr<tree_edge_t> parent_edge{ e_index_map[parent_edge_idx] };

  // remove root from its parent's child list. root is dangling
  v_index_map[old_parent]->children.remove(node_idx);
  // update parent index, need to update the parent edge though
  v_index_map[node_idx]->parent = new_parent_idx;
  // update parent's child list. still need to update edge
  v_index_map[new_parent_idx]->children.push_back(node_idx);
  parent_edge->source = new_parent_idx;
}

void tree_t::to_file(const std::string filename_tree)
{
  using prx::constants::separating_value;

  std::cout << "Saving tree as:" << filename_tree << "\n";
  std::ofstream ofs_tree{ filename_tree.c_str(), std::ofstream::trunc };

  ofs_tree << "#" << separating_value;
  ofs_tree << "parent_idx" << separating_value;
  ofs_tree << "edge_idx" << separating_value;
  ofs_tree << "node_idx" << separating_value;
  ofs_tree << "node_state\n";

  for (auto iter = vertex_list.begin(); iter != const_v_iter; ++iter)
  {
    const std::shared_ptr<tree_node_t> node{ *iter };
    const node_index_t node_idx{ node->get_index() };
    const edge_index_t edge_idx{ node->get_parent_edge() };
    const node_index_t parent_idx{ node->get_parent() };

    ofs_tree << parent_idx << prx::constants::separating_value;
    ofs_tree << edge_idx << prx::constants::separating_value;
    ofs_tree << node_idx << prx::constants::separating_value;
    ofs_tree << node->point << "\n";
  }
  ofs_tree.close();
}

}  // namespace prx
