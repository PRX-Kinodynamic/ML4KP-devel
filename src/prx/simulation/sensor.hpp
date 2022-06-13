#pragma once
#include "prx/utilities/defs.hpp"

namespace prx
{
    struct obstacle_info_t
    {
        std::string name;
        std::vector<double> initial_position, current_position, linear_velocity;
    };

    class sensor_t
    {
    public:
        sensor_t(const std::string& s_name):sensor_name(s_name){}
        virtual ~sensor_t(){}

        inline std::string get_sensor_name()
        {
            return sensor_name;
        }

        virtual void add_obstacle(const std::string &name, const std::vector<double> &initial_position)
        {
            obstacle_info_t obstacle;
            obstacle.name = name;
            obstacle.initial_position = initial_position;
            obstacles.push_back(obstacle);
        }

        virtual void print_obstacle_infos()
        {
            for (auto && obstacle : obstacles)
            {
                std::cout << "Obstacle: " << obstacle.name << std::endl;
                std::cout << "Initial position: " << obstacle.initial_position[0] << " " << obstacle.initial_position[1] << " " << obstacle.initial_position[2] << std::endl;
            }
        }

    private:
        std::string sensor_name;
        std::vector<obstacle_info_t> obstacles;
    };

}