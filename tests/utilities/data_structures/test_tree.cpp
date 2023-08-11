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
  BOOST_CHECK(tree.capacity() == 1000);
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(100);
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
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(60'000);

  prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
  const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

  for (int i = 1; i < 50'000; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::edge_index_t edge_index = tree.add_edge(node_index - 1, node_index);
  }
  for (int i = 0; i < 10'000; ++i)
  {
    tree.remove_vertex(node_index);
    node_index--;
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
  const uint64_t expected_vertices{ 40'000 };
  const uint64_t expected_edges{ 39'999 };
  const uint64_t expected_capacity{ 60'000 };
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
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(60'000);

  prx::node_index_t node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
  const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

  for (int i = 1; i < 50'000; ++i)
  {
    // Adding 499 nodes to the tree randomly
    node_index = tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>();
    const std::shared_ptr<prx::tree_node_t> new_tree_node = tree.get_vertex_as<prx::tree_node_t>(node_index);

    const prx::edge_index_t edge_index = tree.add_edge(node_index - 1, node_index);
  }
  for (int i = 0; i < 10'000; ++i)
  {
    tree.mark_vertex_for_removal(node_index);
    node_index--;
  }
  tree.remove_vertices();

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
  const uint64_t expected_vertices{ 40'000 };
  const uint64_t expected_edges{ 39'999 };
  const uint64_t expected_capacity{ 60'000 };
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
// A "branch-and-bound" case where we do a recursive delete of a particular sub-branch of the tree.
BOOST_AUTO_TEST_CASE(test_tree_remove_marked_tree_branch)
{
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(100);

  //           7
  //       3   8
  //    1
  //       4   9
  //          10
  //  0
  //          11
  //       5  12
  //    2
  //       6  13
  //          14
  for (int i = 0; i < 15; ++i)
  {
    // Create 14 vertices; this are sequencially numbered
    prx::node_index_t node_index{ tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>() };
    BOOST_CHECK_MESSAGE(node_index == i, "Got: " << node_index << " Expected: " << i);
  }
  tree.add_edge(0, 1);
  tree.add_edge(0, 2);

  tree.add_edge(1, 3);
  tree.add_edge(1, 4);

  tree.add_edge(2, 5);
  tree.add_edge(2, 6);

  tree.add_edge(3, 7);
  tree.add_edge(3, 8);

  tree.add_edge(4, 9);
  tree.add_edge(4, 10);

  tree.add_edge(5, 11);
  tree.add_edge(5, 12);

  tree.add_edge(6, 13);
  tree.add_edge(6, 14);

  tree.mark_vertex_for_removal(2);
  tree.mark_vertex_for_removal(5);
  tree.mark_vertex_for_removal(6);
  tree.mark_vertex_for_removal(11);
  tree.mark_vertex_for_removal(12);
  tree.mark_vertex_for_removal(13);
  tree.mark_vertex_for_removal(14);

  tree.remove_vertices();

  const uint64_t expected_vertices{ 8 };
  const uint64_t expected_edges{ 7 };
  const uint64_t expected_capacity{ 100 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());
}

BOOST_AUTO_TEST_CASE(test_tree_remove_full_tree_and_readd)
{
  prx::tree_t tree{};
  tree.allocate_memory<prx::tree_node_t, prx::tree_edge_t>(100);

  //           7
  //       3   8
  //    1
  //       4   9
  //          10
  //  0
  //          11
  //       5  12
  //    2
  //       6  13
  //          14
  for (int i = 0; i < 15; ++i)
  {
    // Create 14 vertices; this are sequencially numbered
    prx::node_index_t node_index{ tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>() };
    BOOST_CHECK_MESSAGE(node_index == i, "Got: " << node_index << " Expected: " << i);
  }
  tree.add_edge(0, 1);
  tree.add_edge(0, 2);

  tree.add_edge(1, 3);
  tree.add_edge(1, 4);

  tree.add_edge(2, 5);
  tree.add_edge(2, 6);

  tree.add_edge(3, 7);
  tree.add_edge(3, 8);

  tree.add_edge(4, 9);
  tree.add_edge(4, 10);

  tree.add_edge(5, 11);
  tree.add_edge(5, 12);

  tree.add_edge(6, 13);
  tree.add_edge(6, 14);

  tree.mark_vertex_for_removal(2);
  tree.mark_vertex_for_removal(5);
  tree.mark_vertex_for_removal(6);
  tree.mark_vertex_for_removal(11);
  tree.mark_vertex_for_removal(12);
  tree.mark_vertex_for_removal(13);
  tree.mark_vertex_for_removal(14);
  tree.remove_vertices();

  tree.mark_vertex_for_removal(1);
  tree.mark_vertex_for_removal(3);
  tree.mark_vertex_for_removal(4);
  tree.mark_vertex_for_removal(7);
  tree.mark_vertex_for_removal(8);
  tree.mark_vertex_for_removal(9);
  tree.mark_vertex_for_removal(10);
  tree.remove_vertices();

  const uint64_t expected_vertices{ 1 };
  const uint64_t expected_edges{ 0 };
  const uint64_t expected_capacity{ 100 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());

  tree.mark_vertex_for_removal(0);
  tree.remove_vertices();
  const uint64_t expected_zero_vertices{ 0 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == expected_zero_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  prx::node_index_t prev_node_index{ tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>() };
  for (int i = 1; i < 20; ++i)
  {
    prx::node_index_t node_index{ tree.add_vertex<prx::tree_node_t, prx::tree_edge_t>() };
    tree.add_edge(prev_node_index, node_index);
    prev_node_index = node_index;
  }

  const uint64_t readded_expected_vertices{ 20 };
  const uint64_t readded_expected_edges{ 19 };
  const uint64_t readded_expected_capacity{ 100 };
  BOOST_CHECK_MESSAGE(tree.num_vertices() == readded_expected_vertices,
                      "Wrong number of vertices. Expected: " << expected_vertices << ", got " << tree.num_vertices());
  BOOST_CHECK_MESSAGE(tree.num_edges() == readded_expected_edges,
                      "Wrong number of edges. Expected: " << expected_edges << ", got " << tree.num_edges());
  BOOST_CHECK_MESSAGE(tree.capacity() == readded_expected_capacity,
                      "Wrong capacity. Expected: " << expected_capacity << ", got " << tree.capacity());
}