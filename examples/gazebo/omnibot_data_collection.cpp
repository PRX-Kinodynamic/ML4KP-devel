// #include <conio.h>
#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/gazebo/gz_sim.hpp"


using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/gazebo/omnibot_data_collection.yaml", argc, argv);
    std::srand(std::time(0));
	auto sim = std::make_shared<gz_sim_t>();

    sim -> set_world(worlds_path + "omnibot_friction_map.world");

    std::string plant_name = "RUmnibot";//params["/plant/name"].as<>();
    std::string plant_path = "RUmnibot";//params["/plant/path"].as<>();

    auto plant = prx::system_factory_t::create_system_as<plant_gz_wrapper_t>(plant_name, plant_path);
    std::cout << "plant created as: " << typeid(plant).name() << plant << std::endl;
    prx_assert(plant != nullptr, "Plant is nullptr!");

    sim -> add_group({plant});
    sim -> create_context({plant_name}, {});
    sim -> initialize_simulation();
    
    auto ss = plant -> get_state_space();
    auto cs = plant -> get_control_space();
    std::cout << plant -> get_pathname() << " space " << ss << std::endl;
    auto state = ss -> make_point();

    cs -> copy_from_vector(params["control_to_use"].as<std::vector<double>>());

	bool keep_running = true;

    std::cout<<"press q to exit! "<<std::endl;
    gazebo::physics::ModelPtr m = sim -> get_model_ptr("RUmnibot");

    std::vector<std::string> joint_names {"joint_chassis_omniwheel_1", "joint_chassis_omniwheel_2", "joint_chassis_omniwheel_3", "joint_chassis_omniwheel_4"};
    for (auto jn : joint_names)
    {
        m -> GetJoint(jn) -> SetPosition(0, uniform_random(0, 2.0 * M_PI), true);
    }

    std::cout << "Wheel angles: [ " << 
        m -> GetJoint("joint_chassis_omniwheel_1") -> Position() << ", " <<
        m -> GetJoint("joint_chassis_omniwheel_2") -> Position() << ", " <<
        m -> GetJoint("joint_chassis_omniwheel_3") -> Position() << ", " <<
        m -> GetJoint("joint_chassis_omniwheel_4") -> Position() << " ]" << std::endl;


    // std::ofstream fout_roa;
    // fout_roa.open((out_path + "omnibot_trajs/gazebo/" + params["out_file"].as<>()).c_str());
    double prev = 0.0;

    trajectory_t traj(ss);
    plan_t plan(cs);
    while(keep_running) 
    {
        auto c = std::cin.get();
        if (c == 'q')
        {
            keep_running = false;
        }
        else if (c == 's')
        {
            std::cout << "Stepping..." << std::endl;
            // for (int i = 0; i < 100; ++i)
            while(sim -> world -> SimTime().sec < 5)
            {
            // std::cout << sim -> world -> SimTime() << std::endl;
                sim -> step_simulation(1);

                if (sim -> world -> SimTime().Double() - prev > 0.1)
                {
                    traj.copy_onto_back(ss);
                    plan.append_onto_back(sim -> world -> SimTime().Double() - prev, true);
                    prev = sim -> world -> SimTime().Double();
                    // fout_roa << prev << " " << m -> WorldPose() << " " << cs -> print_memory(4) << std::endl;
                }
            }
            std::cout << "done stepping!" << std::endl;
        }
        else if (c == 'r')
        {
            sim -> reset_simulation();
        }
        else if (c == 'm')
        {
            auto m_ptr = sim -> get_model_ptr("RUmnibot");
        }
        else if (c == 'e')
        {   
            ss -> sample(state);
            std::cout << "sample state: " << state << std::endl;
            ss -> copy_from_point(state);
        }
    }

    traj.to_file(out_path + "omnibot_trajs/gazebo/traj_" + params["test_num"].as<>() + ".txt", std::ofstream::app);
    plan.to_file(out_path + "omnibot_trajs/gazebo/plan_" + params["test_num"].as<>() + ".txt");

    std::cout << "DONE!" << std::endl;

}