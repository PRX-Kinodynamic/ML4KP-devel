#include <queue>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/controllers/pid.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/roadmap_with_gaps.hpp"

using namespace prx;
using Control = Eigen::Vector2d;
using Error = Eigen::Vector2d;
using State = Eigen::Vector3d;
using Gain = Eigen::Matrix2d;
using PID = prx::pid_t<Control, Error, State, Gain>;

using prx::utilities::csv_reader_t;

double angle_diff(const double& a, const double& b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

// VerticesEdges: vertex -> Iterable of edges
// Edges: edge -> pair of vertices
// EdgesCost: edge -> cost of traversing this edge
// OutNeighbors: vertex -> vertex
// CostToGo: vertex -> cost
template <typename Vertex, typename VerticesEdges, typename Edges, typename EdgesCost, typename OutNeighbors,
          typename CostToGo>
void dijkstra(Vertex goal, VerticesEdges& vertices_edges, Edges& edges, EdgesCost& edges_cost, OutNeighbors& neighbors,
              CostToGo& cost_to_go)
{
  using Edge = std::size_t;  // Should be inferred from EdgeCost
  auto cmp = [&](const Edge& a, const Edge& b) { return !(edges_cost[a] < edges_cost[b]); };

  std::set<Vertex> visited;
  // std::map<std::size_t, double> cost_to_go;
  std::priority_queue<Edge, std::vector<Edge>, decltype(cmp)> pq(cmp);

  cost_to_go[goal] = 0.0;
  pq.push(goal);
  while (!pq.empty())
  {
    const Vertex u{ pq.top() };
    pq.pop();

    if (visited.count(u) == 1)
      continue;

    visited.insert(u);

    const double current_cost_to_go{ cost_to_go[u] };
    const auto u_edges = vertices_edges[u];     // u_edges is an iterable of edges out of vertex u
    for (const Edge& candidate_edge : u_edges)  //
    {
      const auto pair_vertices{ edges[candidate_edge] };
      const Vertex candidate{ pair_vertices.first == u ? pair_vertices.second : pair_vertices.first };

      const double candidate_cost_to_go{ cost_to_go.count(candidate) == 1 ? cost_to_go[candidate] :
                                                                            std::numeric_limits<double>::infinity() };
      const double candidate_edge_cost{ edges_cost[candidate_edge] };
      const double new_cost{ current_cost_to_go + candidate_edge_cost };
      if (new_cost <= candidate_cost_to_go)
      {
        cost_to_go[candidate] = new_cost;
        neighbors[candidate] = u;
      }
      pq.push(candidate);
    }
  }
}

void read_problem(const std::string& problems_filename, const std::size_t& problem_idx, Eigen::Vector3d& start,
                  Eigen::Vector3d& goal)
{
  csv_reader_t reader(problems_filename);
  std::size_t i{ 0 };
  while (reader.has_next_line())
  {
    const csv_reader_t::Line<double> line{ reader.next_line<double>() };
    if (line.size() > 0)
    {
      if (problem_idx == i)
      {
        start = Eigen::Vector3d(line[0], line[1], line[2]);
        goal = Eigen::Vector3d(line[3], line[4], line[5]);
        break;
      }
      i++;
    }
  }
}
struct regulator_params
{
  double kp;
  double ka;
  double kb;
};
Eigen::Vector2d regulator(Eigen::Vector3d current_state, Eigen::Vector3d desired_state, regulator_params& params,
                          int& reverse, int version)
{
  const Eigen::Vector3d delta{ desired_state - current_state };
  const double theta{ current_state[2] };

  const double p{ std::sqrt(std::pow(delta[0], 2) + std::pow(delta[1], 2)) };
  // const double a{ std::atan2(delta[1], delta[0]) - theta };
  const double a{ angle_diff(std::atan2(delta[1], delta[0]), theta) };
  static const double pi2{ prx::constants::pi / 2.0 };
  const int curr_reverse{ (-pi2 < a and a <= pi2) ? 1 : -1 };
  if (reverse == 0)
    reverse = curr_reverse;
  // const double beta{ reverse > 0 ? +theta + a - desired_state[2] : theta - a + desired_state[2] };
  // const double beta{ -theta - a + desired_state[2] };
  const double beta{ norm_angle_pi(angle_diff(-theta, a) + desired_state[2]) };
  // PRX_DEBUG_VAR_1(current_state.transpose());
  // PRX_DEBUG_VAR_1(desired_state.transpose());
  // PRX_DEBUG_VAR_2(a, beta);
  // PRX_DEBUG_VAR_2(reverse, curr_reverse);
  const double v{ params.kp * p };
  const double omega{ (params.ka * a + params.kb * beta) };
  // if (version == 1)
  return Eigen::Vector2d(curr_reverse * omega, curr_reverse * v);
  // else
  // return Eigen::Vector2d(curr_reverse * omega, reverse * v);
  // return Eigen::Vector2d(reverse * omega, reverse * v);
}

template <typename Roadmap, typename Metric>
bool get_next_from_index(Roadmap& roadmap, int& index, prx::space_point_t& next, Eigen::Vector3d& res, Metric metric)
{
  if (index == -1)
  {
    return get_local_goal(roadmap, next, res, metric, index);
  }
  auto pt = roadmap->get_vertex(index);
  int new_index = pt->get_successor_index();
  PRX_DEBUG_VAR_2(index, new_index);
  if (new_index != -1)
  {
    res = Vec(pt->point).head(3);
    index = new_index;
    // res = Vec(nearest->point).head(3);
    PRX_DEBUG_VAR_2(pt->point, index);
    return true;
  }
  PRX_DEBUG_PRINT;
  return get_local_goal(roadmap, next, res, metric, index);
}

template <typename Roadmap, typename Metric>
bool get_local_goal(Roadmap& roadmap, prx::space_point_t& next, Eigen::Vector3d& res, Metric metric,
                    int& nearest_successor_index)
{
  // std::vector<roadmap_with_gaps_node_t*> nodes;

  auto prox_nodes = metric->multi_query(next, 10);
  PRX_DEBUG_PRINT;
  for (auto node : prox_nodes)
  {
    auto nearest = static_cast<roadmap_with_gaps_node_t*>(node);

    nearest_successor_index = nearest->get_successor_index();
    PRX_DEBUG_VAR_2(nearest->get_index(), nearest_successor_index);

    if (nearest_successor_index != -1)
    {
      auto pt = roadmap->get_vertex_point(nearest_successor_index);
      res = Vec(pt).head(3);
      // res = Vec(nearest->point).head(3);
      return true;
    }
  }
  return false;
}

template <typename Roadmap, typename Metric, typename CG>
void wavefront_to_file(prx::space_t* ss, Roadmap& roadmap, Metric& metric, CG& cg)
{
  //   lower_bounds
  // upper_bounds
  const double x_min{ 0 };
  const double x_max{ 30 };
  const double y_min{ 0 };
  const double y_max{ 18 };
  const double step{ 0.4 };

  std::ofstream ofs(prx::out_path + "/wavefront.txt", std::ofstream::trunc);

  prx::space_point_t next{ ss->make_point() };
  for (double x = x_min; x < x_max; x += step)
  {
    for (double y = y_min; y < y_max; y += step)
    {
      for (double th = -3.14; th < 3.14; th += step)
      {
        Vec(next).head(3) = Eigen::Vector3d(x, y, th);
        if (default_valid_state(next, ss, cg))
        {
          auto node = metric->single_query(next);

          auto nearest = static_cast<roadmap_with_gaps_node_t*>(node);

          int nearest_successor_index = nearest->get_successor_index();
          if (nearest_successor_index != -1)
          {
            auto pt = roadmap->get_vertex_point(nearest_successor_index);
            ofs << Vec(next).transpose() << " ";
            ofs << Vec(nearest->point).transpose() << " ";
            ofs << Vec(pt).transpose() << " ";
            ofs << "\n";
            // ss->copy_point(local_goal, roadmap->get_vertex_point(nearest_successor_index));
          }
        }
      }
    }
  }
  ofs.close();
}

int main(int argc, char* argv[])
{
  prx::param_loader params("executables/control/ackermann_roadmap_ctrl.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  auto sg = context.system_group;
  auto cg = context.collision_group;

  space_t* ss{ context.first->get_state_space() };
  space_t* cs{ context.first->get_control_space() };

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  prx::constants::separating_value = ',';
  std::unordered_map<std::size_t, Eigen::Vector3d> vertices;
  // std::unordered_map<std::size_t, std::pair<std::size_t, double>> edges;
  csv_reader_t reader_vertices(params["files/vertices"].as<>());
  csv_reader_t reader_edges(params["files/edges"].as<>());

  Eigen::Vector3d start_state, goal_state;
  const std::size_t problem_idx{ params["problem"].as<std::size_t>() };
  read_problem(params["files/problems"].as<std::string>(), problem_idx, start_state, goal_state);
  std::size_t start_id{ 0 }, goal_id{ 0 };
  space_point_t start{ ss->make_point() };
  space_point_t goal{ ss->make_point() };
  while (reader_vertices.has_next_line())
  {
    const csv_reader_t::Line<double> line{ reader_vertices.next_line<double>() };
    if (line.size() > 0)
    {
      const Eigen::Vector3d vertex{ line[1], line[2], line[3] };
      const std::size_t vertex_id{ static_cast<std::size_t>(line[0]) };
      vertices[vertex_id] = vertex;
      if (start_state.isApprox(vertex, 1e-3))
      {
        start_id = vertex_id;
        ss->copy(start, { line[1], line[2], line[3], line[4], line[5] });
      }
      if (goal_state.isApprox(vertex, 1e-3))
      {
        ss->copy(goal, { line[1], line[2], line[3], line[4], line[5] });
        goal_id = vertex_id;
      }
    }
  }
  prx_assert(start_id != goal_id, "Error with start_id [" << start_id << "] and goal_id [" << goal_id << "].");

  std::unordered_map<std::size_t, std::vector<std::size_t>> vertices_edges;
  std::unordered_map<std::size_t, std::pair<std::size_t, std::size_t>> edges;
  std::unordered_map<std::size_t, double> edges_cost;
  // VerticesEdges: vertex -> Iterable of edges
  // Edges: edge -> pair of vertices
  // EdgesCost: edge -> cost of traversing this edge
  std::size_t edge_id{ 0 };

  const std::string filename{ params["out/graph_filename"].as<>() };
  std::ofstream ofs_graph(filename.c_str(), std::ofstream::trunc);

  while (reader_edges.has_next_line())
  {
    const csv_reader_t::Line<double> line{ reader_edges.next_line<double>() };
    if (line.size() > 0)
    {
      const double vertex0{ line[0] };
      const double vertex1{ line[1] };
      const double edge_cost{ line[2] };
      vertices_edges[vertex0].push_back(edge_id);
      vertices_edges[vertex1].push_back(edge_id);
      edges[edge_id] = std::make_pair(vertex0, vertex1);
      edges_cost[edge_id] = edge_cost;
      edge_id++;

      ofs_graph << vertices[vertex0].transpose() << " ";
      ofs_graph << vertices[vertex1].transpose() << " ";
      ofs_graph << edge_cost << "\n";
    }
  }
  ofs_graph.close();
  // OutNeighbors: vertex -> vertex
  // CostToGo: vertex -> cost
  std::unordered_map<std::size_t, std::size_t> out_neighbors;
  std::unordered_map<std::size_t, double> cost_to_go;

  dijkstra(goal_id, vertices_edges, edges, edges_cost, out_neighbors, cost_to_go);

  const std::string filename_path{ params["out/path_filename"].as<>() };
  std::ofstream ofs_path(filename_path.c_str(), std::ofstream::trunc);
  std::size_t curr_vertex{ start_id };

  Eigen::Vector<double, 3> x_desired{ vertices[out_neighbors[start_id]] };

  space_point_t current_state{ ss->make_point() };

  prx::trajectory_t result{ ss };
  space_point_t control{ cs->make_point() };
  regulator_params ctrl_params;
  ctrl_params.kp = params["control/kp"].as<double>();
  ctrl_params.ka = params["control/ka"].as<double>();
  ctrl_params.kb = params["control/kb"].as<double>();

  prx_assert(ctrl_params.kp > 0, "kp has to be greater than 0");
  prx_assert(ctrl_params.kb < 0, "kp has to be less than 0");
  prx_assert(ctrl_params.ka - ctrl_params.kp > 0, " ka -kp has to be greater than 0");
  double max_time = 500.0;
  double executed_time{ 0.0 };
  int reverse{ 0 };
  bool valid_traj{ true };
  double trajectory_duration{ 0.0 };
  const int version{ params["version"].as<int>() };

  rrt_specification_t rogue_spec(context.first, context.second);
  rrt_query_t controller_query(context.first->get_state_space(), context.first->get_control_space());
  controller_query.start_state = context.first->get_state_space()->make_point();
  controller_query.goal_state = context.first->get_state_space()->make_point();

  learned_controller_t controller(plant, prx::param_loader(params["controller"].as<std::string>()));

  std::shared_ptr<roadmap_with_gaps_t> roadmap =
      std::make_shared<roadmap_with_gaps_t>(rogue_spec, controller_query, controller);
  roadmap->load_roadmap_from_file(params["files/vertices"].as<>(), params["files/edges"].as<>());

  auto s_nn = roadmap->add_start(start);
  auto g_nn = roadmap->add_goal(goal);
  // PRX_DEBUG_VAR_2(s_nn, g_nn);
  roadmap->compute_wavefront(g_nn);
  space_point_t next{ ss->make_point() };
  Eigen::Vector<double, 5> start_5d{ (Eigen::Vector<double, 5>() << vertices[start_id], 0, 0).finished() };
  distance_function_t distance = [](const space_point_t& a, const space_point_t& b) {
    double dist{ (Vec(a).head(2) - Vec(b).head(2)).norm() };
    dist += 0.2 * angle_diff(a->at(2), b->at(2));
    // dist += std::atan2(std::sin(a->at(2) - b->at(2)), std::cos(a->at(2) - b->at(2)));
    return dist;
  };
  prx::space_point_t desired_state{ ss->make_point() };
  prx::graph_nearest_neighbors_t* metric = new prx::graph_nearest_neighbors_t(distance);
  for (auto it : roadmap->get_vertices())
  {
    metric->add_node(it.second.get());
  }
  Eigen::Vector2d vec_to_goal{ Vec(current_state).head(2) - goal_state.head(2) };

  wavefront_to_file(ss, roadmap, metric, cg);
  ss->copy_from(start_5d);

  PRX_DEBUG_VAR_1(start_5d.transpose());

  const double goal_rad{ 0.5 };
  x_desired = start_5d.head(3);
  Vec(desired_state).head(3) = x_desired;
  int current_index{ -1 };
  do
  {
    ss->copy_to(current_state);

    ofs_path << x_desired.transpose() << " ";
    // prx_assert(get_local_goal(roadmap, distance, g_nn, desired_state, x_desired, metric, goal_rad), "no local
    // goal!");
    prx_assert(get_next_from_index(roadmap, current_index, desired_state, x_desired, metric), "no local goal");
    Vec(desired_state).head(3) = x_desired;
    // ofs_path << Vec(current_state).head(3).transpose() << " ";
    ofs_path << x_desired.transpose() << " ";
    // ofs_path << cost_to_go[vertex_to] << "\n";
    ofs_path << "\n";
    executed_time = 0;
    PRX_DEBUG_VAR_1(current_state);
    PRX_DEBUG_VAR_1(desired_state);
    prx::constants::separating_value = ' ';
    reverse = 0;

    do
    {
      result.copy_onto_back(ss);
      ss->copy_to(current_state);
      valid_traj &= default_valid_state(current_state, ss, cg);
      Eigen::Vector2d ctrl{ regulator(Vec(current_state).head(3), x_desired, ctrl_params, reverse, version) };
      Vec(control) = ctrl - Vec(current_state).tail(2);
      cs->enforce_bounds(control);

      sg->propagate_once(control);
      ss->enforce_bounds();
      executed_time += prx::simulation_step;
      trajectory_duration += prx::simulation_step;
      // } while (distance(current_state, desired_state) > goal_rad and executed_time < max_time);
      vec_to_goal = Vec(current_state).head(2) - x_desired.head(2);
    } while (vec_to_goal.norm() > goal_rad and executed_time < max_time);
    // ss->copy_from(desired_state);
    vec_to_goal = Vec(current_state).head(2) - goal_state.head(2);
    ss->copy_to(current_state);
    // PRX_DEBUG_VAR_1(current_state);
    // PRX_DEBUG_VAR_2(goal_state.transpose(), vec_to_goal.transpose());
    result.copy_onto_back(ss);
    PRX_DEBUG_VAR_1(trajectory_duration);
    if (executed_time >= max_time)
    {
      PRX_DEBUG_VAR_1("max_time excided");
      PRX_DEBUG_VAR_1(current_state);
      break;
    }
  } while (vec_to_goal.norm() > 0.5 and trajectory_duration < 1000);
  PRX_DEBUG_VAR_2(goal_state.transpose(), vec_to_goal.transpose());
  ofs_path << Vec(current_state).head(3).transpose() << " ";
  ofs_path << x_desired.transpose() << " ";
  ofs_path << "\n";
  // ofs_path << vertices[curr_vertex].transpose() << " ";
  // ofs_path << vertices[goal_id].transpose() << " ";
  // ofs_path << cost_to_go[goal_id] << "\n";
  ofs_path.close();

  PRX_DEBUG_VAR_1(valid_traj);

  ss->copy(current_state, start_5d);
  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, result, body_name, ss);
  vis_group->add_animation(result, ss, current_state);

  const std::string experiment{ params["experiment"].as<>() };
  const std::string out_dir{ "ackermann_roadmaps/" + experiment + "/" };
  const std::string out_name{ "problem_" + std::to_string(problem_idx) };
  result.to_file(out_dir + out_name + ".txt");
  std::filesystem::create_directories(prx::out_path + out_dir);
  vis_group->output_html(out_dir + out_name + ".html");

  return 0;
}
