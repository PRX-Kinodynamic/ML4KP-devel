#define BOOST_AUTO_TEST_MAIN rrt_test
#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/statistics.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
#include "prx/visualization/three_js_group.hpp"

#ifdef __cpp_lib_filesystem
#include <filesystem>
namespace fs = std::filesystem;
#else
#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

BOOST_AUTO_TEST_CASE(rrt_ackermann_test)
{
  auto params = prx::param_loader();
  params["stats_step_size"] = 1;
  params["checker_type"] = "iterations";
  params["checker_value"] = 10000;
  params["min_steps"] = 1;
  params["max_steps"] = 50;
  params["out_dir"] = prx::lib_path + "/out/unit_tests/";
  params["out_file"] = "/rrt_ackermann.txt";
  params["visualize"] = false;
  params["random_seed"] = std::random_device()();

  // 1 for collecting the "original" sample. 2 for the collecting a new sample
  // and test against 1
  params["sample_gathering"] = 2;

  if (params["sample_gathering"].as<int>() == 1)
  {
    params["total_runs"] = 120 + 1;
  }
  else if (params["sample_gathering"].as<int>() == 2)
  {
    params["total_runs"] = 10 + 1;
  }
  else
  {
    prx_throw_quiet("Only two types of tests...");
    exit(-1);
  }

  // Data recolected on 29/07/2022 -> After some changes on random.cpp
  // header,                   time     , iters, nodes      , solution_cost,
  // solution_time, solution_iters Sample_mean,              1.10632,    10000,
  // 5230.97,    5.4719,     0.742294,   6854.4 Sample_variance, 0.0336643,  0,
  // 601931,     1.59777,    0.102511,   6.41978e+06
  // Sample_standard_deviation,0.183478,   0,      775.842,    1.26403,
  // 0.320174,   2533.73
  params["sample_1"]["nodes"]["mean"] = 5230.97;
  params["sample_1"]["nodes"]["variance"] = 601931;
  params["sample_1"]["solution_cost"]["mean"] = 5.4719;
  params["sample_1"]["solution_cost"]["variance"] = 1.59777;
  params["sample_1"]["solution_iters"]["mean"] = 6854.4;
  params["sample_1"]["solution_iters"]["variance"] = 6.41978e+06;

  // if (! fs::exists(params["out_dir"].as<>()) )
  // {
  //     fs::create_directories(params["out_dir"].as<>());
  // }
  // std::ofstream fout;
  // fout.open(params["out_dir"].as<>() + params["out_file"].as<>());

  prx::simulation_step = 0.01;
  prx::init_random(params["random_seed"].as<double>());
  prx::statistics_t stats;
  std::string header;

  int failed_runs{ 0 };
  int max_failed_runs{ 3 };
  bool success_test{ true };
  while (failed_runs < max_failed_runs)
  {
    success_test = true;
    for (int i = 0; i < params["total_runs"].as<int>(); ++i)
    {
      std::vector<std::shared_ptr<prx::movable_object_t> > obstacle_list;
      std::vector<std::string> obstacles_names;

      prx::transform_t obstacle_pose;
      for (int i = 0; i < 10; ++i)
      {
        double x = prx::uniform_random(10.0, 30.0);
        double y = prx::uniform_random(0.0, 20.0);

        obstacle_pose.setIdentity();
        obstacle_pose.translation() = (prx::vector_t(x, y, 0.5));

        double w = prx::uniform_random(0.5, 1.5);
        double h = prx::uniform_random(0.5, 1.5);
        std::string name = "box_" + std::to_string(i);
        obstacle_list.push_back(create_obstacle(new prx::box_t(name, w, h, 1, obstacle_pose)));
        obstacles_names.push_back(name);
      }

      obstacle_pose.setIdentity();

      obstacles_names.push_back("lower_bound");
      obstacle_pose.translation() = (prx::vector_t(20, 0., 0.5));
      obstacle_list.push_back(create_obstacle(new prx::box_t(obstacles_names.back(), 40, 0.1, 1, obstacle_pose)));

      obstacles_names.push_back("upper_bound");
      obstacle_pose.translation() = (prx::vector_t(20, 20, 0.5));
      obstacle_list.push_back(create_obstacle(new prx::box_t(obstacles_names.back(), 40, 0.1, 1, obstacle_pose)));

      obstacles_names.push_back("right_bound");
      obstacle_pose.translation() = (prx::vector_t(0., 10, 0.5));
      obstacle_list.push_back(create_obstacle(new prx::box_t(obstacles_names.back(), 0.1, 20, 1, obstacle_pose)));

      obstacles_names.push_back("left_bound");
      obstacle_pose.translation() = (prx::vector_t(40, 10, 0.5));
      obstacle_list.push_back(create_obstacle(new prx::box_t(obstacles_names.back(), 0.1, 20, 1, obstacle_pose)));

      auto plant_name = "Ackermann_FO";
      auto plant = prx::system_factory_t::create_system(plant_name, plant_name);

      prx::world_model_t world_model({ plant }, { obstacle_list });
      world_model.create_context("context", { plant_name }, { obstacles_names });
      auto context = world_model.get_context("context");

      prx::rrt_t rrt("RRT");
      prx::rrt_specification_t rrt_spec(context.first, context.second);

      auto ctrl_pt = context.first->get_control_space()->make_point();

      int min_steps = params["min_steps"].as<int>();
      int max_steps = params["max_steps"].as<int>();

      rrt_spec.min_control_steps = min_steps;
      rrt_spec.max_control_steps = max_steps;

      prx::rrt_query_t rrt_query(context.first->get_state_space(), context.first->get_control_space());
      rrt_query.start_state = context.first->get_state_space()->make_point();
      rrt_query.goal_state = context.first->get_state_space()->make_point();

      std::vector<double> lower_bounds = { 0, 0, -PRX_PI };
      std::vector<double> upper_bounds = { 40, 20, PRX_PI };
      context.first->get_state_space()->set_bounds(lower_bounds, upper_bounds);

      std::vector<double> start_state = { 5, 10, 0 };
      std::vector<double> goal_state = { 35, 10, 0 };
      context.first->get_state_space()->copy_point_from_vector(rrt_query.start_state, start_state);
      context.first->get_state_space()->copy_point_from_vector(rrt_query.goal_state, goal_state);

      rrt_query.goal_region_radius = 0.5;
      rrt_query.get_visualization = params["visualize"].as<bool>();

      rrt.link_and_setup_spec(&rrt_spec);
      rrt.preprocess();
      rrt.link_and_setup_query(&rrt_query);

      // prx::condition_check_t checker("iterations", 10'000);
      prx::condition_check_t checker(params["checker_type"].as<>(),
                                     params["checker_value"].as<int>());  //'

      prx::planner_statistics_t pl_stats;
      pl_stats.link_planner(&rrt);
      pl_stats.link_criterion(&checker);
      pl_stats.repeat_data_gathering(params["stats_step_size"].as<double>(), false);

      auto s = rrt.get_statistics();
      stats.add_sample(s);
      if (i == 0)
      {
        header = pl_stats.serialize_header();
        // fout << header;
      }
      // fout << pl_stats.serialize();

      if (params["visualize"].as<bool>())
      {
        rrt.fulfill_query();

        prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

        std::string body_name = "Ackermann_FO/body";

        auto ss = context.first->get_state_space();

        vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_query.tree_visualization, body_name, ss);

        vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_query.solution_traj, body_name, ss);

        vis_group->add_animation(rrt_query.solution_traj, ss, rrt_query.start_state);

        vis_group->output_html("rrt_test_output.html");

        delete vis_group;
      }
    }
    // fout.close();
    if (params["sample_gathering"].as<int>() == 1)
    {
      std::cout << "header," << header;
      std::cout << stats.serialize() << std::endl;
    }
    else if (params["sample_gathering"].as<int>() == 2)
    {
      auto f_test = [](double v1, double v2, double f_upper, double f_lower) {
        double F = v1 / v2;

        // H0: the two variances are equal (\sigma_1 == \sigma_2)
        // Rejected if
        // F < F_{1−\alpha/2,N1−1,N2−1} or
        // F > F_{\alpha/2,N1−1,N2−1}
        // BOOST_CHECK_MESSAGE(!((F < f_lower) || (F > f_upper)), "H0: Equal variance REJECTED");
        return !((F < f_lower) || (F > f_upper));
      };
      std::cout << "header," << header;
      std::cout << stats.serialize() << std::endl;
      //   0     1       2        3               4               5
      // time, iters, nodes, solution_cost, solution_time, solution_iters
      auto mu2 = stats.sample_mean();
      auto sigma2 = stats.sample_variance();
      // Using \alpha = 0.05
      // N1 = 121
      // N2 = 11
      double v1, v2;
      double f_upper = 3.14;
      double f_lower = 1.0 / 2.16;
      std::cout << "Checking nodes variances" << std::endl;
      v1 = params["/sample_1/nodes/variance"].as<double>();
      v2 = sigma2[2];
      success_test &= f_test(v1, v2, f_upper, f_lower);
      std::cout << "Checking solution_cost variances" << std::endl;
      v1 = params["/sample_1/solution_cost/variance"].as<double>();
      v2 = sigma2[3];
      success_test &= f_test(v1, v2, f_upper, f_lower);
      std::cout << "Checking solution_iters variances" << std::endl;
      v1 = params["/sample_1/solution_iters/variance"].as<double>();
      v2 = sigma2[5];
      success_test &= f_test(v1, v2, f_upper, f_lower);
    }
    if (success_test)
      break;
    failed_runs++;
  }
  BOOST_CHECK_MESSAGE(failed_runs < max_failed_runs, "H0: Equal variance REJECTED");

  // std::cout << "sample mean: " << stats.sample_mean() << std::endl;
  // std::cout << "sample variance: " << stats.sample_variance() << std::endl;
  // std::cout << "sample std dev: " << stats.sample_standard_deviation() <<
  // std::endl; BOOST_CHECK(context.first != nullptr);
  // BOOST_CHECK(context.second != nullptr);
  // BOOST_CHECK(world_model.get_all_context_names()[0] == "test_context");
}
