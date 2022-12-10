#include "prx/mujoco/mj_plant.hpp"

namespace prx
{
    mujoco_plant_t::mujoco_plant_t(const std::string& path) : plant_t(path)
    {
        system_type = plant_type::MUJOCO;
    }

    mujoco_plant_t::~mujoco_plant_t()
    {
    }

    void mujoco_plant_t::initialize(std::shared_ptr<mujoco_simulator_t> sim)
    {
        this->sim = sim;

        // Set the state and control spaces
        auto joint_iterator = sim->get_joint_info();
        for(auto it = joint_iterator.first; it != joint_iterator.second; ++it)
        {
            prx_throw("Not implemented yet");
        }

        // current_control = input_control_space -> make_point();
        // current_state = state_space -> make_point();
    }

    void mujoco_plant_t::propagate(const double simulation_step)
    {
    }

    void mujoco_plant_t::compute_control()
    {
    }

    void mujoco_plant_t::update_to_mujoco(const space_point_t& point)
    {
    }

    void mujoco_plant_t::update_from_mujoco(const bool save_sim_state)
    {
    }

    void mujoco_plant_t::update_configuration()
    {
    }

    void mujoco_plant_t::compute_derivative()
    {
    }

}