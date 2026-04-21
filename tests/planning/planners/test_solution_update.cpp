#include <memory>
#include "general/debug_utils.hpp"
#define BOOST_AUTO_TEST_MAIN solution_update_test
#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/planning/planners/solution_update.hpp"

namespace mock
{
struct node_t
{
  using NodePtr = std::shared_ptr<node_t>;
  double cost;
  std::vector<std::size_t> children_;

  double cost_to_come() const
  {
    return cost;
  }
  std::vector<std::size_t>& children()
  {
    return children_;
  }

  static NodePtr create(const double cost)
  {
    NodePtr node{ std::make_shared<node_t>() };
    node->cost = cost;
    return node;
  }
};
struct input_t
{
  using NodePtr = std::shared_ptr<node_t>;

  std::size_t node_index;
  bool delete_branch;
  double cost_bound;
};
struct output_t
{
};
struct tree_t
{
  using NodePtr = std::shared_ptr<node_t>;
  // Dummy *tree* that just stores nodes (w/its cost) and its children
  std::map<std::size_t, NodePtr> nodes;

  NodePtr node(const std::size_t idx)
  {
    return nodes[idx];
  }

  void remove_node(const std::size_t node_index)
  {
    // Slow but easy way of removing children
    for (auto ni : nodes)
    {
      if (ni.second == nullptr)
        continue;

      std::vector<std::size_t> new_children;
      for (auto child : ni.second->children_)
      {
        if (child != node_index)
        {
          new_children.push_back(child);
        }
      }
      ni.second->children_ = new_children;
    }
    // nodes[node_index].children_. = nullptr;  // -1 means node is not valid
    nodes[node_index] = nullptr;  // -1 means node is not valid
  }
};
struct memory_t
{
  using NodePtr = std::shared_ptr<node_t>;

  std::shared_ptr<tree_t> tree_internal;
  std::shared_ptr<tree_t> tree_nn;

  memory_t() : tree_internal(std::make_shared<tree_t>()), tree_nn(std::make_shared<tree_t>())
  {
  }

  NodePtr node(std::size_t idx)
  {
    return tree_internal->nodes[idx];
  }

  std::shared_ptr<tree_t> nearest_neighbors()
  {
    return tree_nn;
  }
  std::shared_ptr<tree_t> tree()
  {
    return tree_internal;
  }
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(tree_branch_and_bound_test)
{
  std::shared_ptr<mock::memory_t> memory{ std::make_shared<mock::memory_t>() };

  memory->tree_internal->nodes[0] = mock::node_t::create(0);
  memory->tree_internal->nodes[0]->children().push_back(1);
  memory->tree_internal->nodes[0]->children().push_back(2);

  memory->tree_internal->nodes[1] = mock::node_t::create(1);
  memory->tree_internal->nodes[1]->children().push_back(3);
  memory->tree_internal->nodes[1]->children().push_back(4);

  memory->tree_internal->nodes[2] = mock::node_t::create(2);
  memory->tree_internal->nodes[3] = mock::node_t::create(30);

  memory->tree_internal->nodes[4] = mock::node_t::create(4);
  for (int i = 0; i < 10; ++i)
  {
    memory->tree_internal->nodes[4 + i]->children().push_back(4 + 1 + i);
    memory->tree_internal->nodes[5 + i] = mock::node_t::create(5 + i);
  }

  std::shared_ptr<mock::input_t> input{ std::make_shared<mock::input_t>() };
  std::shared_ptr<mock::output_t> output{ std::make_shared<mock::output_t>() };

  input->node_index = 0;
  input->delete_branch = false;
  input->cost_bound = 4.5;
  prx::solution_update::tree_branch_and_bound(input, memory);

  BOOST_REQUIRE(memory->tree_internal->nodes[0] != nullptr);
  BOOST_REQUIRE(memory->tree_internal->nodes[1] != nullptr);
  BOOST_REQUIRE(memory->tree_internal->nodes[2] != nullptr);
  BOOST_REQUIRE(memory->tree_internal->nodes[3] == nullptr);
  BOOST_REQUIRE(memory->tree_internal->nodes[4] != nullptr);
  for (int i = 0; i < 10; ++i)
  {
    BOOST_REQUIRE(memory->tree_internal->nodes[5 + i] == nullptr);
  }
}
