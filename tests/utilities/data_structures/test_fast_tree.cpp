#define BOOST_AUTO_TEST_MAIN spaces_test
#include <chrono>
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/abstract_tree.hpp"
#include "prx/utilities/defs.hpp"

using Node = prx::fast_tree_node_t;
using Edge = prx::fast_tree_edge_t;
using Tree = prx::fast_tree_t<Node, Edge>;

BOOST_AUTO_TEST_CASE(test_empty_tree_is_built_correctly)
{
  Tree tree{};
  BOOST_CHECK(tree.size() == 0);
  BOOST_CHECK(tree.total_nodes() == 0);
  BOOST_CHECK(tree.total_edges() == 0);
}
BOOST_AUTO_TEST_CASE(test_tree_allocate_memory_method)
{
  Tree tree(10);
  BOOST_CHECK_MESSAGE(tree.capacity() == 10, EXPECTED_GOT(10, tree.capacity()));
  tree.allocate_memory(1000);
  BOOST_CHECK_MESSAGE(tree.capacity() == 1010, EXPECTED_GOT(1010, tree.capacity()));
  tree.allocate_memory(100);
  BOOST_CHECK_MESSAGE(tree.capacity() == 1110, EXPECTED_GOT(1110, tree.capacity()));
  tree.allocate_memory(2000);
  BOOST_CHECK_MESSAGE(tree.capacity() == 3110, EXPECTED_GOT(3110, tree.capacity()));
}
BOOST_AUTO_TEST_CASE(test_tree_add_vertices_and_edges)
{
  Tree tree(1000);

  // Adding root
  {
    const Node::Index node_index{ tree.add_node() };
    const std::shared_ptr<Node> new_tree_node{ tree.node(node_index) };
    BOOST_CHECK(new_tree_node != nullptr);
  }
  for (int i = 1; i < 500; ++i)
  {
    // Adding 499 nodes to the tree randomly
    const Node::Index node_index{ tree.add_node() };
    const std::shared_ptr<Node> new_tree_node{ tree.node(node_index) };

    const Node::Index parent_id{ static_cast<Node::Index>(prx::uniform_int_random(0, i)) };
    const Edge::Index edge_index{ tree.add_edge(parent_id, node_index) };
  }
  BOOST_CHECK_MESSAGE(tree.total_nodes() == 500, "Wrong number of vertices. Expected: 500, got " << tree.total_nodes());
  BOOST_CHECK_MESSAGE(tree.total_edges() == 499, "Wrong number of edges. Expected: 499, got " << tree.total_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == 1000, "Wrong capacity. Expected: 1000, got " << tree.capacity());
}

BOOST_AUTO_TEST_CASE(test_tree_remove_node)
{
  Tree tree(60'000);

  Node::Index node_index{ tree.add_node() };
  const std::shared_ptr<Node> new_tree_node{ tree.node(node_index) };

  for (int i = 1; i < 50'000; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_node();
    const std::shared_ptr<Node> new_tree_node{ tree.node(node_index) };
    const Edge::Index edge_index{ tree.add_edge(node_index - 1, node_index) };
  }
  for (int i = 0; i < 10'000; ++i)
  {
    tree.remove_node(node_index);
    node_index--;
  }

  const uint64_t expected_vertices{ 40'000 };
  const uint64_t expected_edges{ 39'999 };
  const uint64_t expected_capacity{ 60'000 };
  BOOST_CHECK_MESSAGE(tree.total_nodes() == expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.total_nodes());
  BOOST_CHECK_MESSAGE(tree.total_edges() == expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.total_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());
}
