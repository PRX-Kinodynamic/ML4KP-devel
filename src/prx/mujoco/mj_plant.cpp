#ifndef MUJOCO_NOT_BUILT

#include "prx/mujoco/mj_plant.hpp"
// #include "prx/mujoco/mj_simulator.hpp"

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

  // [qpos, qvel] is the state of the system that we will use for planning.
  // [ctrl] is the control of the system that we will use for planning.
  for (int i = 0; i < sim->m->nq; i++)
    state_memory.push_back(new double);
  for (int i = 0; i < sim->m->nv; i++)
    state_memory.push_back(new double);
  for (int i = 0; i < sim->m->nu; i++)
    control_memory.push_back(new double);

  std::string state_topo_string = "";
  std::string control_topo_string = "";
  std::vector<double> ss_lb, ss_ub, cs_lb, cs_ub;
  unsigned next_qpos = 0;
  unsigned idx = 0;

  for (int i = 0; i < sim->joint_info.size(); i++)
  {
    auto joint = sim->joint_info[i];

    if (joint->qposadr != next_qpos)
    {
      std::cout << "joint: " << joint->name << std::endl;
      std::cout << "joint->qposadr: " << joint->qposadr << " next_qpos: " << next_qpos << std::endl;
      prx_throw("Joints are not in the order of qposadr...");
    }

    switch (joint->type)
    {
      case 0:
        // Free
        prx_warn("This is a free joint. Setting limits arbitrarily.");
        // These are the qpos positions
        state_topo_string += "EEE";
        for (int i = 0; i < 3; i++)
        {
          state_memory[idx + i] = &sim->d->qpos[joint->qposadr + i];
        }
        ss_lb.push_back(-10);
        ss_lb.push_back(-10);
        ss_lb.push_back(0);
        ss_ub.push_back(10);
        ss_ub.push_back(10);
        ss_ub.push_back(10);
        // These are the qpos rotations
        state_topo_string += "QQQQ";
        for (int i = 3; i < 7; i++)
        {
          ss_lb.push_back(-1);
          ss_ub.push_back(1);
          state_memory[idx + i] = &sim->d->qpos[joint->qposadr + i];
        }
        // These are the qvel
        state_topo_string += "EEEEEE";
        for (int i = 0; i < 6; i++)
        {
          ss_lb.push_back(-10.);
          ss_ub.push_back(10.);
          state_memory[idx + i + 7] = &sim->d->qvel[joint->dofadr + i];
        }
        idx += 13;
        next_qpos += 7;
        break;
      case 2:
        // Slide
        state_topo_string += "EE";

        if (joint->limited)
        {
          ss_lb.push_back(joint->range[0]);
          ss_ub.push_back(joint->range[1]);
          ss_lb.push_back(-50.);
          ss_ub.push_back(50.);
        }
        else
        {
          prx_warn("Slide joint is not limited. Setting limits to (-inf, inf)");
          ss_lb.push_back(-PRX_INFINITY);
          ss_ub.push_back(PRX_INFINITY);
          ss_lb.push_back(-50.);
          ss_ub.push_back(50.);
        }

        state_memory[idx] = &sim->d->qpos[joint->qposadr];
        state_memory[idx + 1] = &sim->d->qvel[joint->dofadr];
        idx += 2;
        next_qpos++;
        break;
      case 3:
        // Hinge
        state_topo_string += "EE";

        if (joint->limited)
        {
          ss_lb.push_back(joint->range[0]);
          ss_ub.push_back(joint->range[1]);
          ss_lb.push_back(-10.);
          ss_ub.push_back(10.);
        }
        else
        {
          ss_lb.push_back(-PRX_PI);
          ss_ub.push_back(PRX_PI);
          ss_lb.push_back(-10.);
          ss_ub.push_back(10.);
        }

        state_memory[idx] = &sim->d->qpos[joint->qposadr];
        state_memory[idx + 1] = &sim->d->qvel[joint->dofadr];
        idx += 2;
        next_qpos++;
        break;
      default:
        prx_throw("Joint type not supported (yet)");
    }
  }

  if (next_qpos != sim->m->nq)
  {
    std::cout << "Error: joint dims " << next_qpos << " != nq " << sim->m->nq << std::endl;
    prx_throw("Exiting...");
  }

  for (int i = 0; i < sim->actuator_info.size(); i++)
  {
    auto actuator = sim->actuator_info[i];
    control_topo_string += "E";
    control_memory[i] = &sim->d->ctrl[i];

    if (actuator->limited)
    {
      cs_lb.push_back(actuator->range[0]);
      cs_ub.push_back(actuator->range[1]);
    }
    else
    {
      cs_lb.push_back(-1.);
      cs_ub.push_back(1.);
    }
  }

  state_space = new space_t(state_topo_string, state_memory, "mujoco_state");
  input_control_space = new space_t(control_topo_string, control_memory, "mujoco_control");

  state_space->set_bounds(ss_lb, ss_ub);
  input_control_space->set_bounds(cs_lb, cs_ub);
}

void mujoco_plant_t::propagate(const double simulation_step)
{
}

void mujoco_plant_t::compute_control()
{
}

void mujoco_plant_t::update_configuration()
{
  mj_fwdPosition(sim->m, sim->d);
}

void mujoco_plant_t::compute_derivative()
{
}

}  // namespace prx
#endif