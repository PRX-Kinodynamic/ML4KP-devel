#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{
	
	planner_functions_t::planner_functions_t(std::string context_name, world_model_context context, param_loader pl)
	{
		// auto context = wm->get_context(context_name);
		auto sg = context.first;
		auto cg = context.second;
		auto state_space = sg->get_state_space();
		auto control_space = sg->get_control_space();

		int min_steps = pl["control_min"].as<int>();
		int max_steps = pl["control_max"].as<int>();

		cost_function = [](const trajectory_t& t, const plan_t& plan)
		{
			return default_cost_function(t,plan);
		};
		distance_function = [](const space_point_t& s1, const space_point_t& s2)
		{
			return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
						(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)));
		};
		sample_state = [state_space](space_point_t& s)
		{
			default_sample_state(s,state_space);
		};
		sample_plan = [control_space,min_steps,max_steps](plan_t& p, space_point_t current) // TODO: incorporate ctrl here instead of space_point_t!!
		{
		  default_sample_plan(p,control_space,min_steps,max_steps);
		};
		valid_state = [state_space,cg](space_point_t& s)
		{
			return default_valid_state(s, state_space,cg);
		};
		valid_check = [state_space,cg](trajectory_t& traj)
		{
			return default_valid_trajectory(traj, state_space,cg);
		};
		propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
		{
			default_propagate(start_state,plan,out_traj,sg);
		};

		h = [this](const space_point_t& s, const space_point_t& s2)
		{
			return default_heuristic_function(s, s2, distance_function);
		};
		expand = [sg,this](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
		{
			default_expand(s, plans, trajs, bn, sg, sample_plan, propagate);
		};
		obstacle_distance_function = [state_space,cg,this](const space_point_t& s)
		{
			return default_obstacle_distance_function(s,state_space,cg);
		};
	}

	void default_sample_state(space_point_t& s,space_t* ss)
	{
		ss->sample(s);
	}

	void default_sample_plan(plan_t& plan, space_t* control_space, int min_steps, int max_steps)
	{
		plan.clear();
		// std::cout<<1/simulation_step<<"\t"<<4/simulation_step<<"\tvs.\t"<<min_steps*simulation_step<<"\t"<<max_steps*simulation_step<<std::endl;
		// plan.append_onto_back(uniform_int_random(min_steps,max_steps)*simulation_step);
		double multiplier = simulation_step >= 1 ? simulation_step : 1./simulation_step;
		// std::cout<<(min_steps/multiplier)<<"\t"<<(max_steps/multiplier)<<"\t"<<multiplier<<std::endl;
		plan.append_onto_back(uniform_int_random(min_steps,max_steps)/multiplier);
		control_space->sample(plan.back().control);
	}

	bool default_valid_trajectory(trajectory_t& traj, space_t* ss,std::shared_ptr<collision_group_t> cg)
	{
   
	  	for(auto&& s : traj)
	    {
	      	if (!default_valid_state(s,ss,cg))
			{
		  		return false;
			}
	    }
	  	return true;
	}
	
	bool default_valid_state(space_point_t& s,space_t* ss,std::shared_ptr<collision_group_t> cg)
	{
		ss->copy_from_point(s);
		if(cg->in_collision() || !ss->satisfies_bounds(s))
		{
			return false;
		}
		return true;
	}

	bool default_time_valid_trajectory(trajectory_t& traj, space_t* ss, std::shared_ptr<collision_group_t> cg, std::shared_ptr<world_model_t> wm, double start_time)
	{
		for (unsigned i = 0; i < traj.size(); i++)
		{
			auto s = traj.at(i);
			if (!default_time_valid_state(s,ss,cg,wm,start_time + i*simulation_step))
			{
				return false;
			}
		}
		return true;
	}

	bool default_time_valid_state(space_point_t& s, space_t* ss, std::shared_ptr<collision_group_t> cg, std::shared_ptr<world_model_t> wm, double current_time)
	{
		ss -> copy_from_point(s);
		wm -> update_all_obstacle_poses(current_time);
		if(cg->in_collision() || !ss->satisfies_bounds(s))
		{
			return false;
		}
		return true;
	}


	void default_stopping_control(space_point_t& s,std::shared_ptr<system_group_t> sg, double t)
	{
		sg ->compute_stopping_maneuver(s,t);
	}

	collision_group_t::pqp_distance_t default_obstacle_distance_function(const space_point_t& s,space_t* ss,std::shared_ptr<collision_group_t> cg)
	{
		ss->copy_from_point(s);
 		return cg->get_distances();
	}

	void default_propagate(space_point_t& start_state, plan_t& plan, trajectory_t& out_traj,std::shared_ptr<system_group_t> sg)
	{
		sg->propagate(start_state,plan,out_traj);
	}

	void default_expand(space_point_t& start_state, std::vector<plan_t*>& plans, 
		std::vector<trajectory_t*>& trajs, int bn, std::shared_ptr<system_group_t> sg, 
		sample_plan_t sp, propagate_t prop)
	{
		plans.clear();
		trajs.clear();
		for(int i=0;i<bn;i++)
		{
			plans.push_back(new plan_t(sg->get_control_space()));
			trajs.push_back(new trajectory_t(sg->get_state_space()));
			sp(*plans.back(), start_state);
			prop(start_state,*plans.back(),*trajs.back());
		}
	}

	std::set<std::pair<std::shared_ptr<plan_t>, std::shared_ptr<trajectory_t>>> 
		default_expand_set(space_point_t& start_state, std::set<std::pair<space_point_t, double>> pts_time_set, 
							std::shared_ptr<system_group_t> sg, propagate_t prop)
	{
		std::set<std::pair<std::shared_ptr<plan_t>, std::shared_ptr<trajectory_t>>> expanded;

		for (auto pair : pts_time_set)
		{
			std::pair<std::shared_ptr<plan_t>, std::shared_ptr<trajectory_t>> p;
			p.first = std::make_shared<plan_t>(sg -> get_control_space());
			p.second = std::make_shared<trajectory_t>(sg -> get_state_space());
			p.first -> clear();
			p.first -> append_onto_back(pair.second);
			sg -> get_control_space() -> copy_point(p.first -> back().control, pair.first);
		// void copy_point(const space_point_t& destination,const space_point_t& source) const;
			// plan.append_onto_back(uniform_int_random(min_steps,max_steps)/multiplier);
		// control_space->sample(plan.back().control);
			prop(start_state, *p.first, *p.second);
			expanded.insert(p);
		}
		return expanded;
	}

	double default_cost_function(const trajectory_t& t, const plan_t& plan)
	{
		return plan.duration();
	}

	double default_heuristic_function(const space_point_t& s, const space_point_t& g, distance_function_t d)
	{
		return d(s,g);
	}

	int default_horizon_function(const int& R)
	{
		//printf("%s R: %d value: %.3f\n", __PRETTY_FUNCTION__, R,  100.0 * R * log10(R));
		return 100.0 * R * log10(R);
	}

	double default_eta_function(const int& R)
	{
		return R * R / 300.0;
	}

	bool default_goal_check(const space_point_t& p, const space_point_t& goal, const double rad)
	{
		prx_assert(rad > 0, "default_goal_check: radius has to be grater than zero!")
		return space_t::euclidean_2d(p, goal) < rad;
	}

}
