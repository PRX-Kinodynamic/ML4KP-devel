#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"

namespace prx
{
    struct obstacle_info_t
    {
        std::vector<double> initial_position, current_position, linear_velocity;
        double initial_update_time, current_update_time;
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
            std::shared_ptr<obstacle_info_t> obstacle(new obstacle_info_t);
            obstacle->current_position = initial_position;
            obstacle->initial_position = initial_position;
            obstacle->initial_update_time = obstacle->current_update_time = 0;
            obstacle->linear_velocity = {0.0, 0.0, 0.0};
            obstacles.insert(std::make_pair(name, obstacle));
        }

        virtual void update_obstacle_pose(const std::string &name, const std::vector<double> &new_position, double timestamp)
        {
			prx_assert(obstacles.find(name) != obstacles.end(), "Obstacle " << name << " not found in world model");
            current_time = timestamp;
            auto obstacle = obstacles[name];
            obstacle->initial_position = obstacle->current_position;
            obstacle->current_position = new_position;
            obstacle->initial_update_time = obstacle->current_update_time;
            obstacle->current_update_time = timestamp;
            obstacle->linear_velocity.clear();
            for (int i = 0; i < new_position.size(); i++)
            {
                obstacle->linear_velocity.push_back((obstacle->current_position[i] - obstacle->initial_position[i]) / (obstacle->current_update_time - obstacle->initial_update_time));
            }
        }

        virtual std::unordered_map<std::string, std::vector<double>> get_obstacle_poses()
        {
            std::unordered_map<std::string, std::vector<double>> result;
            for (auto it : obstacles)
            {
                result.insert(std::make_pair(it.first, it.second->current_position));
            }
            return result;
        }

        virtual std::unordered_map<std::string, std::vector<double>> get_obstacle_poses(double time)
        {
            std::unordered_map<std::string, std::vector<double>> result;
            std::vector<double> pos(3);
            if (time <= current_time) 
            {
                result = get_obstacle_poses();
            } 
            else 
            {
                for(auto it : obstacles)
                {
                    pos.at(0) = it.second->current_position[0] + (time - it.second->current_update_time) * it.second->linear_velocity[0];
                    pos.at(1) = it.second->current_position[1] + (time - it.second->current_update_time) * it.second->linear_velocity[1];
                    pos.at(2) = it.second->current_position[2] + (time - it.second->current_update_time) * it.second->linear_velocity[2];
                    result.insert(std::make_pair(it.first, pos));
                }
            }
            return result;
        }

        virtual void print_obstacle_infos()
        {
            for (auto it : obstacles)
            {
                std::cout << "Obstacle: " << it.first << std::endl;
                std::cout << "Current position: " << it.second->current_position[0] << " " << it.second->current_position[1] << " " << it.second->current_position[2] << std::endl;
            }
        }

    private:
        std::string sensor_name;
        double current_time;
        std::unordered_map<std::string,std::shared_ptr<obstacle_info_t>> obstacles;
    };

}