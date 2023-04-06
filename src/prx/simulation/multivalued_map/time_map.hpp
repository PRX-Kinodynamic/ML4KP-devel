#pragma once
#include <mutex> 
#include "prx/simulation/multivalued_map/tm_controllers.hpp"
#include "prx/simulation/system.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/simulation/world_model.hpp"

namespace prx
{
namespace simulation
{

struct time_map_data_t
{
  time_map_data_t()
    : _state_space(nullptr)
    , _control_space(nullptr)
    , _parameter_space(nullptr)
    , _checker(nullptr)
    , _world_model(nullptr)
    , _params()
  {
  }

  prx::param_loader _params;
  system_ptr_t _system;
  space_point_t x_goal;
  space_point_t u_goal;

  std::string _system_name;
  prx::space_t* _state_space;
  prx::space_t* _control_space;
  prx::space_t* _parameter_space;
  std::shared_ptr<system_group_t> _system_group;

  prx::condition_check_t* _checker;
  prx::controller_ptr_t _controller;

  prx::world_model_t* _world_model;  // Maybe change it to simulator_t to use mujoco (or other?)
};

class time_map_t
{
public:
  time_map_t(const std::string& param_filename) : _data()
  {
    _data._params.add_file(param_filename);
    init_from_param_loader();
    init_spaces();
  }
  time_map_t(prx::param_loader& param) : _data()
  {
    _data._params = param;
    init_from_param_loader();
    init_spaces();
  }

  time_map_t(std::string system_name, system_ptr_t system_ptr,
             std::shared_ptr<system_group_t> system_group)
    : _data()
  {
    _data._system_group = system_group;
    _data._system_name = system_name;
    _data._system = system_ptr;
    _data._checker = new condition_check_t("sim_time", 1);
    init_spaces();
  }

  void set_duration(const double duration)
  {
    _data._checker->set_check_value(duration);
  }

  template <typename StartState, typename ResultType>
  void operator()(const StartState start, ResultType& result)
  {
    _data._checker->reset();
    _data._system_group->propagate(start, _data._controller, *(_data._checker), result);
  }

  space_t* get_state_space()
  {
    return _data._state_space;
  }

  time_map_data_t _data;

protected:
  void init_from_param_loader()
  {
    prx::simulation_step = _data._params["simulation_step"].as<double>();
    prx::init_random(_data._params["random_seed"].as<int>());
    _data._system_name = _data._params["system_name"].as<>();

    auto lower_bounds = _data._params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = _data._params["/plant/state_space_upper_bound"].as<std::vector<double>>();

    const std::string plant_name = _data._params["/plant/name"].as<>();
    const std::string plant_path = _data._params["/plant/path"].as<>();
    _data._system = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(_data._system != nullptr, "Plant is nullptr!");
    _data._world_model = new prx::world_model_t({ _data._system }, {});  // TODO: add obstacles
    _data._world_model->create_context("context", { plant_name }, {});   // TODO: add obstacles
    auto context = _data._world_model->get_context("context");
    _data._system_group = context.first;
    _data._checker = new condition_check_t("sim_time", _data._params["duration"].as<double>());
  }
  void init_spaces()
  {
    prx_assert(_data._system_group != nullptr, "System group has not been initialized.");
    _data._state_space = _data._system_group->get_state_space();
    _data._control_space = _data._system_group->get_control_space();
    _data._parameter_space = _data._system_group->get_parameter_space();
    _data.x_goal = _data._state_space->make_point();
    _data.u_goal = _data._control_space->make_point();
    auto controller_generator = time_map_controllers_t::get_controller(_data._system_name);
    _data._controller = controller_generator(_data);
  }
};

}  // namespace simulation
}  // namespace prx
