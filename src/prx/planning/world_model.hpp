#pragma once

#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/system_group_manager.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include <memory>
#include <unordered_map>

namespace prx
{
typedef std::pair<std::shared_ptr<system_group_t>, std::shared_ptr<collision_group_t>> world_model_context;

class world_model_t : public simulator_t
{
public:
  using WorldModelPtr = std::shared_ptr<world_model_t>;
  using SystemGroupPtr = std::shared_ptr<system_group_t>;
  using CollisionGroupPtr = std::shared_ptr<collision_group_t>;
  using WorldGroupsTuple = std::tuple<WorldModelPtr, SystemGroupPtr, CollisionGroupPtr>;
  //
  // std::shared_ptr<prx::system_group_t> _system_group;
  // std::shared_ptr<prx::collision_group_t> _collision_group;

  static WorldGroupsTuple create(const prx::param_loader& params, prx::system_ptr_t plant)
  {
    prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(params) };
    auto obstacle_list = obstacle_loader.get_obstacles();
    auto obstacle_names = obstacle_loader.get_names();

    const std::vector<prx::system_ptr_t> all_systems{ { plant } };
    const std::vector<std::shared_ptr<prx::movable_object_t>> all_obstacles{ { obstacle_list } };
    WorldModelPtr world_ptr{ std::make_shared<prx::world_model_t>(all_systems, all_obstacles) };

    world_ptr->create_context("planner_context", { plant->get_pathname() }, { obstacle_names });
    auto planning_context = world_ptr->get_context("planner_context");

    return { world_ptr, prx::system_group(planning_context), prx::collision_group(planning_context) };
  }
  static WorldGroupsTuple create(prx::system_ptr_t plant)  // empty world
  {
    // std::vector<std::shared_ptr<movable_object_t>> obstacle_list;
    // std::vector<std::string> obstacle_names;
    const std::vector<std::shared_ptr<prx::movable_object_t>> all_obstacles;
    const std::vector<prx::system_ptr_t> all_systems{ { plant } };
    WorldModelPtr world_ptr{ std::make_shared<prx::world_model_t>(all_systems, all_obstacles) };

    world_ptr->create_context("planner_context", { plant->get_pathname() }, {});
    auto planning_context = world_ptr->get_context("planner_context");

    return { world_ptr, prx::system_group(planning_context), prx::collision_group(planning_context) };
  }

  // template<typename SGM = system_group_manager_t, typename CC = collision_checker_t>
  explicit world_model_t(const std::vector<system_ptr_t>& all_systems,
                         const std::vector<std::shared_ptr<movable_object_t>>& all_obstacles)
    : simulator_t(plant_type::ANALYTICAL)
  {
    collision_groups = std::make_shared<collision_checker_t>();

    this->add_group(all_systems);

    for (auto o : all_obstacles)
    {
      obstacles[o->get_object_name()] = o;
    }
  }

  template <typename Obstacle, typename... Args>
  void emplace_obstacle(const std::string context_name, Args... args)
  {
    std::shared_ptr<Obstacle> obstacle_ptr{ std::make_shared<Obstacle>(args...) };
    obstacles[obstacle_ptr->get_object_name()] = obstacle_ptr;

    collision_groups->get_collision_group(context_name)->add_new_obstacle(obstacle_ptr);
  }

  ~world_model_t() {};

  inline world_model_context get_context(const std::string& context_name)
  {
    return std::make_pair(system_groups->get_system_group(context_name),
                          collision_groups->get_collision_group(context_name));
  }

  // QUESTION: what about separating this in two stages:
  // 				1) add_context
  // 				2) initialize_simulation
  // 				With the benefit of analytical and bullet being more alike
  void create_context(const std::string& context_name, const std::vector<std::string>& system_names,
                      const std::vector<std::string>& obstacle_names)
  {
    // auto ptr = std::static_pointer_cast<world_model_t>(this -> shared_ptr());
    // std::shared_ptr<world_model_t> ptr;
    // ptr.reset(this);
    // auto ptr = this -> shared_ptr();
    system_groups->link_simulator(this);

    all_context_names.push_back(context_name);
    std::vector<system_ptr_t> context_systems;
    std::vector<std::shared_ptr<movable_object_t>> context_obstacles;
    for (auto&& s : system_names)
    {
      context_systems.push_back(this->systems[s]);
    }
    for (auto&& o : obstacle_names)
    {
      context_obstacles.push_back(obstacles[o]);
    }

    system_groups->add_system_group(context_name, context_systems);
    collision_groups->add_collision_group(context_name, context_systems, context_obstacles);
  }

  inline std::vector<std::string> get_all_context_names()
  {
    return all_context_names;
  }

  // TODO: is stepping all contexts ok?
  virtual void step_simulation() override
  {
    for (auto s : this->systems)
    {
      s.second->propagate(simulation_step);
    }
    // system_groups -> propagate(step);
    // int steps = (int)((duration / simulation_step) + .1);
    // // int i = 0;
    // prx_assert(steps > 0, "Cannot step simulation! Check duration for world_model_t::step_simulation");

    // system_groups -> propagate(steps);
  }

  virtual void reset_simulation() override final
  {
    prx_throw("World model doesn't implement reset");
  }

  std::vector<std::shared_ptr<movable_object_t>> get_obstacles()
  {
    std::vector<std::shared_ptr<movable_object_t>> obstacles_out;
    for (auto pair : obstacles)
    {
      obstacles_out.push_back(pair.second);
    }
    return obstacles_out;
  }

  std::shared_ptr<movable_object_t> obstacle(const std::string& name)
  {
    return obstacles[name];
  }

private:
  std::unordered_map<std::string, std::shared_ptr<movable_object_t>> obstacles;

  std::vector<std::string> all_context_names;
};
}  // namespace prx