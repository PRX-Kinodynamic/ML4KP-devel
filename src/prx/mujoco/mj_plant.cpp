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

// void mujoco_plant_t::initialize(std::shared_ptr<mujoco_simulator_t> sim)
// {
//   this->sim = sim;

//   // [qpos, qvel] is the state of the system that we will use for planning.
//   // [ctrl] is the control of the system that we will use for planning.
//   for (int i = 0; i < sim->_mj_model->nq; i++)
//     state_memory.push_back(new double);
//   for (int i = 0; i < sim->_mj_model->nv; i++)
//     state_memory.push_back(new double);
//   for (int i = 0; i < sim->_mj_model->nu; i++)
//     control_memory.push_back(new double);

//   std::string state_topo_string = "";
//   std::string control_topo_string = "";
//   std::vector<double> ss_lb, ss_ub, cs_lb, cs_ub;
//   unsigned next_qpos = 0;
//   unsigned idx = 0;

//   for (int i = 0; i < sim->joint_info.size(); i++)
//   {
//     auto joint = sim->joint_info[i];

//     if (joint->qposadr != next_qpos)
//     {
//       std::cout << "joint: " << joint->name << std::endl;
//       std::cout << "joint->qposadr: " << joint->qposadr << " next_qpos: " << next_qpos << std::endl;
//       prx_throw("Joints are not in the order of qposadr...");
//     }

//     switch (joint->type)
//     {
//       case 0:
//         // Free
//         prx_warn("This is a free joint. Setting limits arbitrarily.");
//         // These are the qpos positions
//         state_topo_string += "EEE";
//         for (int i = 0; i < 3; i++)
//         {
//           state_memory[idx + i] = &sim->_mj_data->qpos[joint->qposadr + i];
//         }
//         ss_lb.push_back(-20);
//         ss_lb.push_back(-20);
//         ss_lb.push_back(-1);
//         ss_ub.push_back(20);
//         ss_ub.push_back(20);
//         ss_ub.push_back(10);
//         // These are the qpos rotations
//         state_topo_string += "QQQQ";
//         for (int i = 3; i < 7; i++)
//         {
//           ss_lb.push_back(-1);
//           ss_ub.push_back(1);
//           state_memory[idx + i] = &sim->_mj_data->qpos[joint->qposadr + i];
//         }
//         idx += 7;
//         next_qpos += 7;
//         break;
//       case 2:
//         // Slide
//         state_topo_string += "E";

//         if (joint->limited)
//         {
//           ss_lb.push_back(joint->range[0]);
//           ss_ub.push_back(joint->range[1]);
//         }
//         else
//         {
//           prx_warn("Slide joint is not limited. Setting limits to (-inf, inf)");
//           ss_lb.push_back(-PRX_INFINITY);
//           ss_ub.push_back(PRX_INFINITY);
//         }

//         state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr];
//         idx += 1;
//         next_qpos++;
//         break;
//       case 3:
//         // Hinge
//         state_topo_string += "E";

//         if (joint->limited)
//         {
//           ss_lb.push_back(joint->range[0]);
//           ss_ub.push_back(joint->range[1]);
//         }
//         else
//         {
//           ss_lb.push_back(-PRX_PI);
//           ss_ub.push_back(PRX_PI);
//         }

//         state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr];
//         idx += 1;
//         next_qpos++;
//         break;
//       case 1:
//         // Ball
//         state_topo_string += "QQQQ";

//         if (joint->limited)
//         {
//           prx_throw("Ball joint is limited. Not supported yet.");
//         }

//         for (int i = 0; i < 4; i++)
//         {
//           ss_lb.push_back(-1);
//           ss_ub.push_back(1);
//           state_memory[idx + i] = &sim->_mj_data->qpos[joint->qposadr + i];
//         }

//         idx += 4;
//         next_qpos += 4;
//         break;
//       default:
//         prx_throw("Joint type not supported (yet)");
//     }
//   }

//   for (int i = 0; i < sim->_mj_model->nv; i++)
//   {
//     state_topo_string += "E";
//     state_memory[idx + i] = &sim->_mj_data->qvel[i];
//     ss_lb.push_back(-PRX_INFINITY);
//     ss_ub.push_back(PRX_INFINITY);
//   }

//   if (next_qpos != sim->_mj_model->nq)
//   {
//     std::cout << "Error: joint dims " << next_qpos << " != nq " << sim->_mj_model->nq << std::endl;
//     prx_throw("Exiting...");
//   }

//   for (int i = 0; i < sim->actuator_info.size(); i++)
//   {
//     auto actuator = sim->actuator_info[i];
//     control_topo_string += "E";
//     control_memory[i] = &sim->_mj_data->ctrl[i];

//     if (actuator->limited)
//     {
//       cs_lb.push_back(actuator->range[0]);
//       cs_ub.push_back(actuator->range[1]);
//     }
//     else
//     {
//       cs_lb.push_back(-1.);
//       cs_ub.push_back(1.);
//     }
//   }

//   state_space = new space_t(state_topo_string, state_memory, "mujoco_state");
//   input_control_space = new space_t(control_topo_string, control_memory, "mujoco_control");

//   state_space->set_bounds(ss_lb, ss_ub);
//   input_control_space->set_bounds(cs_lb, cs_ub);
// }

void mujoco_plant_t::initialize(std::shared_ptr<mujoco_simulator_t> sim)
{
  this->sim = sim;

  // [qpos, qvel] is the state of the system that we will use for planning.
  // [ctrl] is the control of the system that we will use for planning.
  //
  // ToDo: We could avoid adding "empty" pointers and just add the pointers to the vector directly...
  for (int i = 0; i < sim->_mj_model->nq; i++)
    state_memory.push_back(new double);
  for (int i = 0; i < sim->_mj_model->nv; i++)
    state_memory.push_back(new double);
  for (int i = 0; i < sim->_mj_model->na; i++)
    state_memory.push_back(new double);
  for (int i = 0; i < sim->_mj_model->nu; i++)
    control_memory.push_back(new double);

  std::string state_topo_string = "";
  std::string control_topo_string = "";
  std::vector<double> ss_lb, ss_ub, cs_lb, cs_ub;
  unsigned next_qpos = 0;
  std::size_t idx = 0;
  std::size_t offset = 0;

  std::string ss_name{};
  for (int i = 0; i < sim->joint_info.size(); i++)
  {
    auto joint = sim->joint_info[i];
    ss_name += joint->name + ":";
    if (joint->qposadr != next_qpos)
    {
      std::cout << "joint: " << joint->name << std::endl;
      std::cout << "joint->qposadr: " << joint->qposadr << " next_qpos: " << next_qpos << std::endl;
      prx_throw("Joints are not in the order of qposadr...");
    }

    switch (joint->type)
    {
      case mjtJoint_::mjJNT_FREE:
        // Free
        // prx_warn_once("Free joint, setting limits arbitrarily.");
        // These are the qpos positions
        state_topo_string += "EEE";
        offset = 0;
        for (; offset < 3; idx++, offset++)
        {
          state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr + offset];
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
        }
        // These are the qpos rotations
        state_topo_string += "QQQQ";
        //  offset \in [3,7]
        for (; offset < 7; idx++, offset++)
        {
          state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr + offset];
          ss_lb.push_back(-1);
          ss_ub.push_back(1);
        }
        // These are the qvel
        state_topo_string += "EEEEEE";
        for (offset = 0; offset < 6; idx++, offset++)
        {
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
          state_memory[idx] = &sim->_mj_data->qvel[joint->dofadr + offset];
        }
        // idx += state_topo_string.size();
        next_qpos += 7;
        break;
      case mjtJoint_::mjJNT_SLIDE:
        // Slide
        state_topo_string += "EE";

        if (joint->limited)
        {
          ss_lb.push_back(joint->range[0]);
          ss_ub.push_back(joint->range[1]);
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
        }
        else
        {
          // prx_warn_once("Slide joint is not limited. Setting limits to (-inf, inf)");
          ss_lb.push_back(-PRX_INFINITY);
          ss_ub.push_back(PRX_INFINITY);
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
        }

        state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr];
        state_memory[idx + 1] = &sim->_mj_data->qvel[joint->dofadr];
        idx += 2;
        next_qpos++;
        break;
      case mjtJoint_::mjJNT_HINGE:
        // Hinge
        state_topo_string += "EE";

        if (joint->limited)
        {
          ss_lb.push_back(joint->range[0]);
          ss_ub.push_back(joint->range[1]);
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
        }
        else
        {
          ss_lb.push_back(-PRX_PI);
          ss_ub.push_back(PRX_PI);
          ss_lb.push_back(std::numeric_limits<double>::lowest());
          ss_ub.push_back(std::numeric_limits<double>::max());
        }

        state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr];
        state_memory[idx + 1] = &sim->_mj_data->qvel[joint->dofadr];
        idx += 2;
        next_qpos++;
        break;
      case mjtJoint_::mjJNT_BALL:
        state_topo_string += "QQQQ";
        ss_lb.insert(ss_lb.end(), 4, -1);
        ss_ub.insert(ss_ub.end(), 4, 1);
        state_topo_string += "EEE";
        ss_lb.insert(ss_lb.end(), 3, -10000.);
        ss_ub.insert(ss_ub.end(), 3, 10000.);
        // state_memory[idx + i + 7] = &sim->d->qvel[joint->dofadr + i];
        state_memory[idx] = &sim->_mj_data->qpos[joint->qposadr];
        state_memory[idx + i + 7] = &sim->_mj_data->qvel[joint->dofadr];
        idx += 4;
        next_qpos += 4;
        break;
      default:
        prx_throw("Joint " << *joint << " not supported (yet)");
    }
  }

  if (next_qpos != sim->_mj_model->nq)
  {
    std::cout << "Error: joint dims " << next_qpos << " != nq " << sim->_mj_model->nq << std::endl;
    prx_throw("Exiting...");
  }

  std::string actuator_name{};
  for (int i = 0; i < sim->actuator_info.size(); i++)
  {
    auto actuator = sim->actuator_info[i];
    control_topo_string += "E";
    control_memory[i] = &sim->_mj_data->ctrl[i];

    actuator_name += actuator->name + ":";
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

  for (int offset = 0; offset < sim->_mj_model->na; offset++)
  {
    state_topo_string += "E";
    state_memory[idx] = &sim->_mj_data->act[offset];
    ss_lb.push_back(std::numeric_limits<double>::lowest());
    ss_ub.push_back(std::numeric_limits<double>::max());
  }

  std::cout << "state space name: " << ss_name << std::endl;
  std::cout << "control space name: " << actuator_name << std::endl;
  std::cout << "control topology name: " << control_topo_string << std::endl;
  state_space = new space_t(state_topo_string, state_memory, ss_name);
  input_control_space = new space_t(control_topo_string, control_memory, actuator_name);

  parameter_space = new space_t();

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
  mj_fwdPosition(sim->_mj_model, sim->_mj_data);
}

void mujoco_plant_t::compute_derivative()
{
}

}  // namespace prx
#endif
