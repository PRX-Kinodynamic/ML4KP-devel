#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_omnirobot.hpp"

namespace prx
{
    bullet_omnirobot_t::bullet_omnirobot_t(const std::string& path) : bullet_t(path)
    {
        x = y = z = r = p = yaw = dx = dy = dz = dr = dp = dyaw = sid = 0;
        z = 0.5;
        state_memory = {&x,&y,&z,&r,&p,&yaw,&dx,&dy,&dz,&dr,&dp,&dyaw,&sid};
        state_space = new space_t("EEERRREEEEEED",state_memory,"XYZRPYdxdydzdrdpdyaId");
            PRX_DEBUG_PRINT
        state_bounds_l = {-15,-15,-.1,-3.14,-3.14,-3.14,-20,-20,-20,-20,-20,-20,0};
        state_bounds_u = {15,15,1,3.14,3.14,3.14,20,20,20,20,20,20,PRX_INFINITY};
        state_space->set_bounds(state_bounds_l, state_bounds_u);

        w1 = w2 = w3 = w4 = 0;
        control_memory = {&w1,&w2,&w3,&w4};
        input_control_space = new space_t("EEEE",control_memory,"ctrl_space");
        control_bounds_l={-2,-2,-2,-2};
        control_bounds_u={ 2, 2, 2, 2};

        input_control_space->set_bounds(control_bounds_l,control_bounds_u); 

        sampled_control = input_control_space->make_point();

    }

    int bullet_omnirobot_t::get_state_id()
    {
        return state_space->at(12);;
    }

    void bullet_omnirobot_t::initialize(std::shared_ptr<b3RobotSimulatorClientAPI> _sim)
    {
      // std::vector<double> start_state = {0,0,0.5,0,0,0};
      // shared_constructor(path,start_state);
        sim = _sim;
        // std::cout << "Loading file " << robot_model_path << std::endl;
        b3RobotSimulatorLoadUrdfFileArgs* loadURDArgs = new b3RobotSimulatorLoadUrdfFileArgs();

        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;

        basePosition[0] = state_space -> at(0);
        basePosition[1] = state_space -> at(1);
        basePosition[2] = state_space -> at(2);
        baseRotation[0] = state_space -> at(3);
        baseRotation[1] = state_space -> at(4);
        baseRotation[2] = state_space -> at(5);
        bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
        loadURDArgs -> m_startPosition = basePosition;

        uniqueId = sim -> loadURDF(robot_model_path, *loadURDArgs);

        int numJoints = sim -> getNumJoints(uniqueId);
        b3JointInfo jointInfo;
        for (int i = 0; i < numJoints; i++)
        {
            sim -> getJointInfo(uniqueId,i,&jointInfo);
            // std::cout << "Joint name: " << jointInfo.m_jointName << std::endl;
            std::string joint_name(jointInfo.m_jointName);
            if (joint_name == "base_ow_1_axis")
            {
                wheelJoints.push_back(i);
            }
            else if (joint_name == "base_ow_2_axis")
            {
                wheelJoints.push_back(i);
            }
            else if (joint_name == "base_ow_3_axis")
            {
                wheelJoints.push_back(i);
            }
            else if (joint_name == "base_ow_4_axis")
            {
                wheelJoints.push_back(i);
            }
            auto control_type = CONTROL_MODE_TORQUE;
            if(first_order)
            {
                control_type = CONTROL_MODE_VELOCITY;
            }
            b3RobotSimulatorJointMotorArgs controlArgs(control_type);

            controlArgs.m_targetVelocity = 0;
            controlArgs.m_maxTorqueValue = 0;
            sim->setJointMotorControl(uniqueId,i,controlArgs);
        }

        // btVector3 basePosition;
        // btQuaternion baseOrientation;
        for (int i =0; i < 100; i++)
        {
            sim->stepSimulation();
        }

        add_exclusion(0,-1,1,-1); //exclude collisions with plane
        // sim -> getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        // auto baseRotation = 
        // bullet_simulator::get_euler_from_quaternion(baseOrientation);
        // btVector3 baseVel, baseAngVel;
        // sim->getBaseVelocity(uniqueId, baseVel, baseAngVel);
        state_space -> at(12) = sim->saveStateToMemory();

    }

    bullet_omnirobot_t::~bullet_omnirobot_t()
    {}

    // void bullet_omnirobot_t::setup()
    // {
       
        // std::cout << "Finished stepping" << std::endl;
        // simulation_step = 0.01;
        // sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        // auto baseRotation = getEulerFromQuaternion(baseOrientation);
        // btVector3 baseVel, baseAngVel;
        // sim->getBaseVelocity(uniqueId, baseVel, baseAngVel);
        // sid = sim->saveStateToMemory();
        
        // x = basePosition[0];
        // y = basePosition[1];
        // z = basePosition[2];
        // r = baseRotation[0];
        // p = baseRotation[1];
        // yaw = baseRotation[2];
        // dx = baseVel[0];
        // dy = baseVel[1];
        // dz = baseVel[2];
        // dr = baseAngVel[0];
        // dp = baseAngVel[1];
        // dyaw = baseAngVel[2];
  
        // current_state = state_space -> make_point();
    // }

    void bullet_omnirobot_t::compute_control()
    {

        input_control_space -> sample(sampled_control);
            std::cout << "sampled_control: " << sampled_control << std::endl;
        input_control_space -> copy_from_point(sampled_control);

        // std::cout << "ctrl: " << sampled_control << std::endl;
        auto control_type = CONTROL_MODE_TORQUE;
        if(first_order)
        {
            control_type = CONTROL_MODE_VELOCITY;
        }
        b3RobotSimulatorJointMotorArgs controlArgs(control_type);
  
        controlArgs.m_maxTorqueValue = maxForce;

        for (int i = 0; i < wheelJoints.size(); i++)
        {
            if(first_order)
            {
                controlArgs.m_targetVelocity = sampled_control -> at(i);
            }
            else
            {
                controlArgs.m_maxTorqueValue = sampled_control -> at(i);
            }
            // std::cout << "uniqueId: " << uniqueId << " wheelJoints[" << i << "]: " << wheelJoints[i] << std::endl;
            sim -> setJointMotorControl(uniqueId, wheelJoints[i], controlArgs);
        }
        
    }
    
    void bullet_omnirobot_t::update_from_bullet(const bool save_sim_state)
    {
        current_state_vec.clear();
        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;

        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        baseRotation = getEulerFromQuaternion(baseOrientation);

        current_state_vec.push_back(basePosition[0]);
        current_state_vec.push_back(basePosition[1]);
        current_state_vec.push_back(basePosition[2]);
    
        current_state_vec.push_back(baseRotation[0]);
        current_state_vec.push_back(baseRotation[1]);
        current_state_vec.push_back(baseRotation[2]);
    
        btVector3 baseVel, baseAngVel;
        sim->getBaseVelocity(uniqueId,baseVel,baseAngVel);
        current_state_vec.insert(current_state_vec.end(),{baseVel[0],baseVel[1],baseVel[2],baseAngVel[0],baseAngVel[1],baseAngVel[2]});  
  
        int sid = state_space->at(12);
       
        if (save_sim_state)
        {
            sid = sim->saveStateToMemory();
            std::cout << "sid updated!" << std::endl;
            std::cout << "sid: " << sid << std::endl;
        } 
        current_state_vec.push_back(sid);
        // lastSavedId = std::max(lastSavedId, sid);
        state_space -> copy_from_vector(current_state_vec);
        // state_space->copy_point_from_vector(current_state,current_state_vec);
        // state_space->copy_from_point(current_state);
        // space_point_t result = state_space->make_point();
        // state_space->copy_to_point(result);
    }
}
#endif
