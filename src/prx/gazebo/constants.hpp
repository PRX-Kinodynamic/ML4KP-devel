#pragma once
#include "prx/utilities/general/constants.hpp"

namespace prx
{
	static inline
	bool lib_append_env_variable(std::string env_var, std::string new_val, bool env_var_set = false)
	{

		// THIS IS NOT WORKING... THE IDEA IS TO AUTOMATICALLY POPULATE THE
		// GAZEBO VARIABLES TO WHERE THE PRX-GZ STUFF IS LOCATED...
		// THE VARIABLES GET POPULATED BUT GAZEBO SEEMS TO NOT BE LOCATING THEM...
		// SHOULD LOOK INTO HOW GAZEBO IS SPAWN (CHILD PROCESS? ANOTHER PROCESS?) 
		// FOR NOW, ONLY CHECH IF THE VARIABLE EXISTS :S
		// if (env_var_set) return true;
		// char* path = std::getenv(env_var.c_str());
		// std::string val;
		// if (path != NULL)
		// {
		// 	val = std::string(path) + ":" ;
		// }
		// val = val + new_val;

		// return setenv(env_var.c_str(), val.c_str(), 1) == 0;
		char* path = std::getenv(env_var.c_str());
		if (path == NULL)
		{
			std::cout << env_var << " environmental variable not set, do:\n";
			std::cout << "export " << env_var << "=${" << env_var << "}:" << new_val << std::endl; 
			exit(1);
		}
		return true;
	}


	const std::string worlds_path = lib_path + "resources/worlds/";

	const bool GAZEBO_MODEL_PATH_SET = lib_append_env_variable("GAZEBO_MODEL_PATH", models_path);
	const bool GAZEBO_RESOURCE_PATH_SET = lib_append_env_variable("GAZEBO_RESOURCE_PATH", worlds_path);
	const bool GAZEBO_PLUGIN_PATH = lib_append_env_variable("GAZEBO_PLUGIN_PATH", lib_path + "lib/gazebo_plugins");
	

}	