#define BOOST_AUTO_TEST_MAIN condition_check_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/condition_check.hpp"

#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"

namespace mock
{
struct plant2d_t
{
  plant2d_t() : x(0), y(0), address({ &x, &y }), space("EE", address, "space_test")
  {
    prx::simulation_step = 0.1;
  }
  double x, y;
  std::vector<double*> address;
  prx::space_t space;
};
class node_t : public prx::tree_node_t
{
public:
  node_t() : duration(0) {};
  virtual ~node_t() {};

  template <typename NodePtr, typename EdgePtr>
  void update(NodePtr parent_node, EdgePtr parent_edge)
  {
    duration = parent_node->duration + parent_edge->plan->duration();
  }

  double duration;
};

struct edge_t : public prx::tree_edge_t
{
  edge_t() : edge_cost(0) {};
  virtual ~edge_t() {};

  std::shared_ptr<prx::plan_t> plan;
  std::shared_ptr<prx::trajectory_t> traj;
  double edge_cost;
};

struct planner_t
{
  using Node = node_t;
  using Edge = edge_t;
  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;

  EdgePtr split_edge(EdgePtr e0, const double time_of_split)
  {
    EdgePtr e1{ tree->split_edge<Node, Edge>(e0->get_index()) };

    e1->plan = std::make_shared<prx::plan_t>(e0->plan->split(time_of_split));
    e1->traj = std::make_shared<prx::trajectory_t>(e0->traj->split(time_of_split));
    return e1;
  }
  prx::tree_t* tree;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(discretize_edge_test)
{
  using Node = typename mock::node_t;
  using Edge = typename mock::edge_t;
  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;

  mock::plant2d_t plant;  // using same space as control and state for simplicity
  prx::space_t* space{ &(plant.space) };

  std::cout << *space << std::endl;
  prx::tree_t tree{};
  mock::planner_t planner;
  planner.tree = &tree;

  tree.allocate_memory<Node, Edge>(10);

  const prx::node_index_t n0_id{ tree.add_vertex<Node, Edge>() };
  const prx::node_index_t n1_id{ tree.add_vertex<Node, Edge>() };

  NodePtr n0{ tree.get_vertex_as<Node>(n0_id) };
  NodePtr n1{ tree.get_vertex_as<Node>(n1_id) };

  const prx::edge_index_t e0_id{ tree.add_edge(n0_id, n1_id) };
  EdgePtr e0{ tree.get_edge_as<Edge>(e0_id) };

  prx::cost_function_t cost_f = [](const prx::trajectory_t& traj, const prx::plan_t& plan) { return plan.duration(); };

  prx::plan_t original_plan(space);
  e0->traj = std::make_shared<prx::trajectory_t>(space);
  e0->plan = std::make_shared<prx::plan_t>(original_plan);

  const double plan_duration{ 5.0 };
  e0->plan->copy_onto_back(Eigen::Vector2d::Ones(), plan_duration);

  for (double ti = 0; ti < plan_duration; ti += prx::simulation_step)
  {
    e0->traj->push_back(Eigen::Vector2d::Ones());
  }
  e0->edge_cost = cost_f(*(e0->traj), *(e0->plan));

  n1->update(n0, e0);

  const double desired_edge_duration{ 1.0 };

  BOOST_CHECK_MESSAGE(2 == tree.size(), EXPECTED_GOT(2, tree.size()));
  prx::planning::discretize_edge(e0, tree, desired_edge_duration, planner);
  //
  BOOST_CHECK_MESSAGE(6 == tree.size(), EXPECTED_GOT(6, tree.size()));

  const double epsilon{ 0.00001 };

  BOOST_REQUIRE_CLOSE(desired_edge_duration, e0->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(0 == e0->get_source(), EXPECTED_GOT(0, e0->get_source()));
  BOOST_CHECK_MESSAGE(2 == e0->get_target(), EXPECTED_GOT(2, e0->get_target()));

  BOOST_CHECK_MESSAGE(1 == n0->get_children().size(), EXPECTED_GOT(1, n0->get_children().size()));
  BOOST_CHECK_MESSAGE(2 == n0->get_children().front(), EXPECTED_GOT(2, n0->get_children().front()));

  const NodePtr n2{ tree.get_vertex_as<Node>(e0->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n2->get_children().size(), EXPECTED_GOT(1, n2->get_children().size()));
  BOOST_CHECK_MESSAGE(3 == n2->get_children().front(), EXPECTED_GOT(3, n2->get_children().front()));
  BOOST_CHECK_MESSAGE(0 == n2->get_parent_edge(), EXPECTED_GOT(0, n2->get_parent_edge()));

  const EdgePtr e1{ tree.get_edge_as<Edge>(1) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e1->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(2 == e1->get_source(), EXPECTED_GOT(2, e1->get_source()));
  BOOST_CHECK_MESSAGE(3 == e1->get_target(), EXPECTED_GOT(3, e1->get_target()));

  const NodePtr n3{ tree.get_vertex_as<Node>(e1->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n3->get_children().size(), EXPECTED_GOT(1, n3->get_children().size()));
  BOOST_CHECK_MESSAGE(4 == n3->get_children().front(), EXPECTED_GOT(4, n3->get_children().front()));

  const EdgePtr e2{ tree.get_edge_as<Edge>(2) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e2->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(3 == e2->get_source(), EXPECTED_GOT(3, e2->get_source()));
  BOOST_CHECK_MESSAGE(4 == e2->get_target(), EXPECTED_GOT(4, e2->get_target()));

  const NodePtr n4{ tree.get_vertex_as<Node>(e2->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n4->get_children().size(), EXPECTED_GOT(1, n4->get_children().size()));
  BOOST_CHECK_MESSAGE(5 == n4->get_children().front(), EXPECTED_GOT(5, n4->get_children().front()));

  const EdgePtr e3{ tree.get_edge_as<Edge>(3) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e3->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(4 == e3->get_source(), EXPECTED_GOT(4, e3->get_source()));
  BOOST_CHECK_MESSAGE(5 == e3->get_target(), EXPECTED_GOT(5, e3->get_target()));

  const NodePtr n5{ tree.get_vertex_as<Node>(e3->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n5->get_children().size(), EXPECTED_GOT(1, n5->get_children().size()));
  BOOST_CHECK_MESSAGE(1 == n5->get_children().front(), EXPECTED_GOT(1, n5->get_children().front()));

  const EdgePtr e4{ tree.get_edge_as<Edge>(4) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e4->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(5 == e4->get_source(), EXPECTED_GOT(5, e4->get_source()));
  BOOST_CHECK_MESSAGE(1 == e4->get_target(), EXPECTED_GOT(1, e4->get_target()));

  BOOST_CHECK_MESSAGE(0 == n1->get_children().size(), EXPECTED_GOT(0, n1->get_children().size()));
}

BOOST_AUTO_TEST_CASE(discretize_tree_test)
{
  using Node = typename mock::node_t;
  using Edge = typename mock::edge_t;
  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;

  mock::plant2d_t plant;  // using same space as control and state for simplicity
  prx::space_t* space{ &(plant.space) };

  prx::tree_t tree{};
  mock::planner_t planner;
  planner.tree = &tree;

  tree.allocate_memory<Node, Edge>(10);

  const prx::node_index_t n0_id{ tree.add_vertex<Node, Edge>() };
  const prx::node_index_t n1_id{ tree.add_vertex<Node, Edge>() };
  const prx::node_index_t n2_id{ tree.add_vertex<Node, Edge>() };

  NodePtr n0{ tree.get_vertex_as<Node>(n0_id) };
  NodePtr n1{ tree.get_vertex_as<Node>(n1_id) };
  NodePtr n2{ tree.get_vertex_as<Node>(n2_id) };

  const prx::edge_index_t e0_id{ tree.add_edge(n0_id, n1_id) };
  const prx::edge_index_t e1_id{ tree.add_edge(n0_id, n2_id) };

  EdgePtr e0{ tree.get_edge_as<Edge>(e0_id) };
  EdgePtr e1{ tree.get_edge_as<Edge>(e1_id) };

  prx::cost_function_t cost_f = [](const prx::trajectory_t& traj, const prx::plan_t& plan) { return plan.duration(); };

  e0->traj = std::make_shared<prx::trajectory_t>(space);
  e0->plan = std::make_shared<prx::plan_t>(space);

  e1->traj = std::make_shared<prx::trajectory_t>(space);
  e1->plan = std::make_shared<prx::plan_t>(space);

  const double plan_duration{ 2.0 };
  e0->plan->copy_onto_back(Eigen::Vector2d::Ones(), plan_duration);
  e1->plan->copy_onto_back(Eigen::Vector2d::Ones(), plan_duration);

  for (double ti = 0; ti < plan_duration; ti += prx::simulation_step)
  {
    e0->traj->push_back(Eigen::Vector2d::Ones());
    e1->traj->push_back(Eigen::Vector2d::Ones());
  }

  e0->edge_cost = cost_f(*(e0->traj), *(e0->plan));
  e1->edge_cost = cost_f(*(e1->traj), *(e1->plan));

  n1->update(n0, e0);
  n2->update(n0, e1);

  const double desired_edge_duration{ 1.0 };

  BOOST_CHECK_MESSAGE(3 == tree.size(), EXPECTED_GOT(2, tree.size()));
  prx::planning::discretize_tree(tree, planner, desired_edge_duration);
  //
  BOOST_CHECK_MESSAGE(5 == tree.size(), EXPECTED_GOT(5, tree.size()));

  const double epsilon{ 0.00001 };

  BOOST_REQUIRE_CLOSE(desired_edge_duration, e0->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(0 == e0->get_source(), EXPECTED_GOT(0, e0->get_source()));
  BOOST_CHECK_MESSAGE(3 == e0->get_target(), EXPECTED_GOT(3, e0->get_target()));

  BOOST_CHECK_MESSAGE(2 == n0->get_children().size(), EXPECTED_GOT(2, n0->get_children().size()));

  const NodePtr n3{ tree.get_vertex_as<Node>(e0->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n3->get_children().size(), EXPECTED_GOT(1, n3->get_children().size()));
  BOOST_CHECK_MESSAGE(1 == n3->get_children().front(), EXPECTED_GOT(1, n3->get_children().front()));
  BOOST_CHECK_MESSAGE(0 == n3->get_parent_edge(), EXPECTED_GOT(0, n3->get_parent_edge()));

  BOOST_REQUIRE_CLOSE(desired_edge_duration, e1->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(0 == e1->get_source(), EXPECTED_GOT(0, e1->get_source()));
  BOOST_CHECK_MESSAGE(4 == e1->get_target(), EXPECTED_GOT(4, e1->get_target()));

  const NodePtr n4{ tree.get_vertex_as<Node>(e1->get_target()) };
  BOOST_CHECK_MESSAGE(1 == n4->get_children().size(), EXPECTED_GOT(1, n4->get_children().size()));
  BOOST_CHECK_MESSAGE(2 == n4->get_children().front(), EXPECTED_GOT(2, n4->get_children().front()));

  const EdgePtr e2{ tree.get_edge_as<Edge>(2) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e2->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(3 == e2->get_source(), EXPECTED_GOT(3, e2->get_source()));
  BOOST_CHECK_MESSAGE(1 == e2->get_target(), EXPECTED_GOT(1, e2->get_target()));

  BOOST_CHECK_MESSAGE(0 == n1->get_children().size(), EXPECTED_GOT(0, n1->get_children().size()));

  const EdgePtr e3{ tree.get_edge_as<Edge>(3) };
  BOOST_REQUIRE_CLOSE(desired_edge_duration, e3->plan->duration(), epsilon);
  BOOST_CHECK_MESSAGE(4 == e3->get_source(), EXPECTED_GOT(4, e3->get_source()));
  BOOST_CHECK_MESSAGE(2 == e3->get_target(), EXPECTED_GOT(2, e3->get_target()));

  BOOST_CHECK_MESSAGE(0 == n2->get_children().size(), EXPECTED_GOT(0, n2->get_children().size()));
}

BOOST_AUTO_TEST_CASE(discretize_tree_test_same_dt)
{
  using Node = typename mock::node_t;
  using Edge = typename mock::edge_t;
  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;
  prx::simulation_step = 0.1;

  mock::plant2d_t plant;  // using same space as control and state for simplicity
  prx::space_t* space{ &(plant.space) };

  prx::tree_t tree{};
  mock::planner_t planner;
  planner.tree = &tree;

  tree.allocate_memory<Node, Edge>(10);

  const prx::node_index_t n0_id{ tree.add_vertex<Node, Edge>() };
  const prx::node_index_t n1_id{ tree.add_vertex<Node, Edge>() };
  const prx::node_index_t n2_id{ tree.add_vertex<Node, Edge>() };

  NodePtr n0{ tree.get_vertex_as<Node>(n0_id) };
  NodePtr n1{ tree.get_vertex_as<Node>(n1_id) };
  NodePtr n2{ tree.get_vertex_as<Node>(n2_id) };

  const prx::edge_index_t e0_id{ tree.add_edge(n0_id, n1_id) };
  const prx::edge_index_t e1_id{ tree.add_edge(n1_id, n2_id) };

  EdgePtr e0{ tree.get_edge_as<Edge>(e0_id) };
  EdgePtr e1{ tree.get_edge_as<Edge>(e1_id) };

  prx::cost_function_t cost_f = [](const prx::trajectory_t& traj, const prx::plan_t& plan) { return plan.duration(); };

  e0->traj = std::make_shared<prx::trajectory_t>(space);
  e0->plan = std::make_shared<prx::plan_t>(space);

  e1->traj = std::make_shared<prx::trajectory_t>(space);
  e1->plan = std::make_shared<prx::plan_t>(space);

  const double plan_duration_0{ 0.3 };  // 0 --0.3-- 1 ==> 0 -0.1- 1 -0.1- 2 -0.1- 3
  const double plan_duration_1{ 0.5 };  // 1 --0.5-- 2 ==> 3 -0.1- 4 -0.1- 5 -0.1- 6 -0.1- 7 -0.1- 8
  e0->plan->copy_onto_back(Eigen::Vector2d::Ones(), plan_duration_0);
  e1->plan->copy_onto_back(Eigen::Vector2d::Ones(), plan_duration_1);

  for (double ti = 0; ti < plan_duration_0; ti += prx::simulation_step)
  {
    e0->traj->push_back(Eigen::Vector2d::Ones());
  }
  for (double ti = 0; ti < plan_duration_1; ti += prx::simulation_step)
  {
    e1->traj->push_back(Eigen::Vector2d::Ones());
  }

  e0->edge_cost = cost_f(*(e0->traj), *(e0->plan));
  e1->edge_cost = cost_f(*(e1->traj), *(e1->plan));

  n1->update(n0, e0);
  n2->update(n1, e1);

  const double desired_edge_duration{ prx::simulation_step };

  const int expected_original_size{ 3 };
  BOOST_CHECK_MESSAGE(expected_original_size == tree.size(), EXPECTED_GOT(expected_original_size, tree.size()));

  prx::planning::discretize_tree(tree, planner, desired_edge_duration);

  const int expected_new_size{ 41 };  // 20+20+1
  BOOST_CHECK_MESSAGE(expected_new_size == tree.size(), EXPECTED_GOT(expected_new_size, tree.size()));

  const double epsilon{ 0.00001 };

  auto edges = tree.edges();
  for (auto iter = edges.first; iter != edges.second; iter++)
  {
    auto edge_i = std::dynamic_pointer_cast<Edge>(*iter);

    const std::size_t id{ edge_i->get_index() };
    const double duration{ edge_i->plan->duration() };
    PRX_DBG_VARS(id, duration);
    BOOST_REQUIRE_CLOSE(desired_edge_duration, edge_i->plan->duration(), epsilon);
  }
}