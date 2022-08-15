#define BOOST_AUTO_TEST_MAIN spaces_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/tree.hpp"

BOOST_AUTO_TEST_CASE(test_empty_tree_is_built_correctly)
{
  prx::tree_t tree{};
  BOOST_CHECK(tree.num_vertices() == 0);
  BOOST_CHECK(tree.num_edges() == 0);
  BOOST_CHECK(tree.capacity() == 0);
}
BOOST_AUTO_TEST_CASE(test_tree_allocate_memory_method)
{
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(1000);
  std::cout << "tree.capacity(): " << tree.capacity() << std::endl;
  BOOST_CHECK(tree.capacity() == 1000);
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(100);
  std::cout << "tree.capacity(): " << tree.capacity() << std::endl;
  BOOST_CHECK(tree.capacity() == 1000);
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(2000);
  BOOST_CHECK(tree.capacity() == 2000);
}
BOOST_AUTO_TEST_CASE(test_tree_add_vertices_and_edges)
{
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(1000);

  // Adding root
  {
    const prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);
  }
  for (int i = 1; i < 500; ++i)
  {
    // Adding 499 nodes to the tree randomly
    const prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::node_index_t parent_id = static_cast<prx::node_index_t>(prx::uniform_int_random(0, i));
    const prx::edge_index_t edge_index = tree.add_edge(parent_id, node_index);
  }
  BOOST_CHECK_MESSAGE(tree.num_vertices() == 500,
                      "Wrong number of vertices. Expected: 500, got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == 499, "Wrong number of edges. Expected: 499, got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == 1000, "Wrong capacity. Expected: 1000, got " << tree.capacity());
}
BOOST_AUTO_TEST_CASE(test_tree_depth)
{
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(1000);

  prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
  const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

  for (int i = 1; i < 500; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::edge_index_t edge_index = tree.add_edge(node_index - 1, node_index);
  }
  BOOST_CHECK_MESSAGE(tree.get_depth(node_index) == 499, "Expected depth 500, got: " << tree.get_depth(node_index));
}
BOOST_AUTO_TEST_CASE(test_tree_remove_vertices)
{
  auto start = std::chrono::steady_clock::now();
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(600'000);

  prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
  const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

  for (int i = 1; i < 500'000; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::edge_index_t edge_index = tree.add_edge(node_index - 1, node_index);
  }
  for (int i = 0; i < 100'000; ++i)
  {
    tree.remove_vertex(node_index);
    node_index--;
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
  const uint64_t expected_vertices{ 400'000 };
  const uint64_t expected_edges{ 399'999 };
  const uint64_t expected_capacity{ 600'000 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());
}
BOOST_AUTO_TEST_CASE(test_tree_remove_marked_vertices)
{
  auto start = std::chrono::steady_clock::now();
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(600'000);

  prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
  const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

  for (int i = 1; i < 500'000; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::edge_index_t edge_index = tree.add_edge(node_index - 1, node_index);
  }
  for (int i = 0; i < 100'000; ++i)
  {
    tree.mark_vertex_for_removal(node_index);
    node_index--;
  }
  tree.remove_vertices();

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
  const uint64_t expected_vertices{ 400'000 };
  const uint64_t expected_edges{ 399'999 };
  const uint64_t expected_capacity{ 600'000 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());
}
BOOST_AUTO_TEST_CASE(test_tree_purge)
{
}
BOOST_AUTO_TEST_CASE(test_tree_clear)
{
}
BOOST_AUTO_TEST_CASE(test_tree_transplant)
{
}
