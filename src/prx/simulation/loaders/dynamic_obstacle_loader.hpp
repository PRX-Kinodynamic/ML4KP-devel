#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/movable_object.hpp"
#include "prx/simulation/loaders/pose_function_helper.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"

namespace prx
{
    std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_dynamic_obstacles(std::string obstacles_file)
    {
        if(obstacles_file=="")
		{
			std::cout<<"Trying to load an empty obstacle file. No obstacles loaded"<<std::endl;
			return std::make_pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>>({},{});
		}
		param_loader obstacle_loader(obstacles_file);
        auto static_geometries_list = obstacle_loader["environment"]["geometries"];
        auto dynamic_geometries_list = obstacle_loader["environment"]["dynamic_geometries"];
        
        std::vector<std::shared_ptr<movable_object_t>> obstacle_list;
        std::vector<std::string> obstacle_names;

        for (auto geom : static_geometries_list)
        {
            std::string name = geom["name"].as<std::string>();
			auto geom_transform = geom["config"];
			auto geom_position = geom_transform["position"].as<std::vector<double>>();
			auto geom_orientation = geom_transform["orientation"].as<std::vector<double>>();
			transform_t obstacle_pose;
			obstacle_pose.linear() = quaternion_t(geom_orientation[3],geom_orientation[0],geom_orientation[1],geom_orientation[2]).toRotationMatrix();
			obstacle_pose.translation() = vector_t(geom_position[0],geom_position[1],geom_position[2]);
			auto geom_params = geom["collision_geometry"];
			auto geom_type = geom_params["type"].as<std::string>();
			int shapeType=-1;

			if (geom_type == "box")
			{
				auto dims = geom_params["dims"].as<std::vector<double>>();
				obstacle_list.push_back(create_obstacle(new box_t(name,dims[0],dims[1],dims[2],obstacle_pose)));
				obstacle_names.push_back(name);
                obstacle_list.back() -> position_function.init("position_function_descriptor", 1.0, geom_position);
            }
            else
			{
				prx_throw("Obstacle loader can't load an obstacle of type: "<<geom_type);
			}
        }
        for (auto geom : dynamic_geometries_list)
        {
            std::string name = geom["name"].as<std::string>();
			auto geom_transform = geom["config"];
			auto geom_position = geom_transform["position"].as<std::vector<double>>();
			auto geom_rotation = geom_transform["rotation"].as<double>();
			transform_t obstacle_pose;
			obstacle_pose.linear() = quaternion_t(cos(geom_rotation/2),0,0,sin(geom_rotation/2)).toRotationMatrix();
			obstacle_pose.translation() = vector_t(geom_position[0],geom_position[1],geom_position[2]);
			auto geom_params = geom["collision_geometry"];
			auto geom_type = geom_params["type"].as<std::string>();

            auto dims = geom_params["dims"].as<std::vector<double>>();
            obstacle_list.push_back(create_obstacle(new box_t(name,dims[0],dims[1],dims[2],obstacle_pose)));
            obstacle_names.push_back(name);

            double func_multiplier = geom["multiplier"].as<double>();
            obstacle_list.back() -> position_function.init("const", func_multiplier, geom_position,geom_rotation);
        }

        // unsigned num_boxes = uniform_int_random(dynamic_geometries_list["min_bodies"].as<int>(),dynamic_geometries_list["max_bodies"].as<int>());
        // for (int i = 0; i < num_boxes; i++)
        // {
        //     std::string name = dynamic_geometries_list["name_prefix"].as<std::string>() + std::to_string(i);
            
        //     std::vector<double> geom_position;
        //     geom_position.push_back(uniform_random(dynamic_geometries_list["min_box"].as<double>(),dynamic_geometries_list["max_box"].as<double>()));
        //     geom_position.push_back(uniform_random(dynamic_geometries_list["min_box"].as<double>(),dynamic_geometries_list["max_box"].as<double>()));
        //     geom_position.push_back(0.0);
        //     double geom_orientation = uniform_random(-PRX_PI*0.5,PRX_PI*0.5);

        //     transform_t obstacle_pose;
        //     obstacle_pose.linear() = quaternion_t(cos(geom_orientation/2),0,0,sin(geom_orientation/2)).toRotationMatrix();
		// 	obstacle_pose.translation() = vector_t(geom_position[0],geom_position[1],geom_position[2]);

        //     std::vector<double> dims;
        //     dims.push_back(uniform_random(dynamic_geometries_list["min_dims"].as<double>(),dynamic_geometries_list["max_dims"].as<double>()));
        //     dims.push_back(uniform_random(dynamic_geometries_list["min_dims"].as<double>(),dynamic_geometries_list["max_dims"].as<double>()));
        //     dims.push_back(.2);

        //     obstacle_list.push_back(create_obstacle(new box_t(name,dims[0],dims[1],dims[2],obstacle_pose)));
        //     obstacle_names.push_back(name);

        //     double vel = uniform_random(dynamic_geometries_list["min_vel"].as<double>(),dynamic_geometries_list["max_vel"].as<double>());
        //     obstacle_list.back() -> position_function.init("const", vel, geom_position, geom_orientation);
        // }

		return std::make_pair(std::move(obstacle_names),std::move(obstacle_list));
    }
}