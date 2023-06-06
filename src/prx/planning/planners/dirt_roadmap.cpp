#include "prx/planning/planners/dirt_roadmap.hpp"

namespace prx
{
    dirt_roadmap_t::dirt_roadmap_t(const std::string& new_name) : dirt_t(new_name)
	{
		planner_name = "DIRT_Roadmap";
		std::string log_fname = output_path + "log.txt";
		log_fout.open(log_fname.c_str());
		log_fout.close();
	}

    dirt_roadmap_t::~dirt_roadmap_t()
    {
        _reset();
    }

    void dirt_roadmap_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		dirt_t::_link_and_setup_spec(spec);
		//reset is always called before this
		dirt_spec = dynamic_cast<dirt_roadmap_specification_t*>(spec);
		prx_assert(dirt_spec!=nullptr,"DIRT (Roadmap) received an incorrect specification.");

		h = dirt_spec->h;
		roadmap_h = dirt_spec->roadmap_h;
		roadmap_expand = dirt_spec->roadmap_expand;
		node_expand = dirt_spec->node_expand;
	}

    bool dirt_roadmap_t::_preprocess()
	{
		tree.allocate_memory<dirt_roadmap_node_t,rrt_edge_t>(1000);
		return true;
	}

    bool dirt_roadmap_t::_link_and_setup_query(planner_query_t* query)
	{
		rrt_query = dynamic_cast<rrt_query_t*>(query);
		prx_assert(rrt_query!=nullptr,"DIRT received an incorrect query type.");
		dirt_query = dynamic_cast<dirt_roadmap_query_t*>(query);
		prx_assert(dirt_query!=nullptr,"DIRT received an incorrect query type.");
		if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			tree.clear();

			start_vertex = tree.add_vertex<dirt_roadmap_node_t,rrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<dirt_roadmap_node_t>(start_vertex);
			start_node->point = state_space->clone_point(rrt_query->start_state);
			start_node->cost_to_come = 0;
			start_node->dir_radius = 0;
			start_node->checkpoint_time = 0;
			// start_node->time_to_come = 0;
			start_node->is_safety_node = false;
			start_node->cost_to_go = h(start_node->point,dirt_query->goal_state);
			start_node->roadmap_cost_to_go = roadmap_h(start_node->point);
			start_node->blossom_number = dirt_spec->blossom_number;
			start_node->reachable_goal = dirt_spec->start_node_reachable_goal;
			metric->add_node(start_node.get());
			previous_child = start_vertex;
			child_extension = true;
		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;
		simulation_time=0;
		return true;
	}

    void dirt_roadmap_t::_resolve_query(condition_check_t* condition)
	{
		// run for a certain amount of time
		do
		{
			if(!child_extension)
			{
				//sample state
				sample_state(sample_point);

				//find closest
				std::vector<dirt_roadmap_node_t*> nodes;
				node_index_t closest_index;
				double best_distance = PRX_INFINITY;
				{
					auto prox_nodes = metric->radius_and_closest_query(sample_point,max_radius);
					std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(nodes),[]
						(proximity_node_t* prox_node)
						{
							return static_cast<dirt_roadmap_node_t*>(prox_node);
						});
					nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
						[&,this](dirt_roadmap_node_t* node)
					{
						const double test_dist = distance_function(sample_point,node->point);
						if(test_dist<best_distance)
						{
							best_distance = test_dist;
							closest_index = node->get_index();
						}
						if(test_dist<=node->dir_radius)
						{
							return false;
						}
						return true;
					}),nodes.end());
				}
				if(nodes.size()==0)
				{
					auto prox_nodes = metric->radius_and_closest_query(get_vertex(closest_index)->point,max_radius);
					std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(nodes),[]
					(proximity_node_t* prox_node)
					{
						return static_cast<dirt_roadmap_node_t*>(prox_node);
					});
					nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
						[&,this](dirt_roadmap_node_t* node)
					{
						const double test_dist = distance_function(get_vertex(closest_index)->point,node->point);
						if(test_dist<=node->dir_radius)
						{
							return false;
						}
						return true;
					}),nodes.end());
				}
				//select one of the nodes at random
				// prx_assert(nodes.size()>0,"Somehow the nearest nodes in DIRT is empty (should not happen).");
				if(nodes.size()>0)
				{
					previous_child = nodes[uniform_int_random(0,nodes.size()-1)]->get_index();
				}
				else
					previous_child = closest_index;
			}
			child_extension = false;

			auto closest_node = get_vertex(previous_child);

			// PRX_DEBUG_PRINT
			// std::cout << "Expanding " << closest_node->get_parent() << " " << previous_child << ": " << state_space->print_point(closest_node->point,4) << " " << closest_node->reachable_goal << std::endl;

			std::vector<plan_t*> plans;
			std::vector<trajectory_t*> trajs;

			node_expand(closest_node,plans,trajs);
			closest_node->expand_num++;

			for(int i=0;i<plans.size();i++)
			{
				closest_node->edge_generators.push_back(std::make_pair(plans[i],trajs[i]));
			}

			std::pair<plan_t*,trajectory_t*> eg = std::make_pair(nullptr,nullptr);
			double edge_cost;
			double end_heuristic;
			double new_node_dir_radius;
			std::vector<dirt_roadmap_node_t*> dir_updates;

			eg = closest_node->edge_generators.back();
			closest_node->edge_generators.back() = std::make_pair(nullptr,nullptr);
			edge_cost = cost_function(*eg.second,*eg.first);
			end_heuristic = h(eg.second->back(),dirt_query->goal_state);

			// bnb
			if((goal_vertex!=start_vertex && closest_node->cost_to_come + edge_cost + closest_node->cost_to_go > current_solution))
			{
				delete eg.first;
				delete eg.second;
				eg = std::make_pair(nullptr,nullptr);
				iteration_count++;
				continue;
			}


			//pruning condition
			double parent_distance = distance_function(eg.second->back(),closest_node->point);
			new_node_dir_radius = parent_distance;

			auto prox_nodes = metric->radius_and_closest_query(eg.second->back(),std::max(parent_distance,max_radius));
			dir_updates.clear();
			std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(dir_updates),[]
			(proximity_node_t* prox_node)
			{
				return static_cast<dirt_roadmap_node_t*>(prox_node);
			});
			std::for_each(dir_updates.begin(), dir_updates.end(),
					[&,this](dirt_roadmap_node_t* node)
				{
					if( closest_node->cost_to_come+edge_cost+end_heuristic > node->cost_to_come + node->cost_to_go)
					{
						new_node_dir_radius = std::min(new_node_dir_radius,distance_function(eg.second->back(),node->point));
					}
				});

			if(dirt_spec->use_pruning)
			{
				std::for_each(dir_updates.begin(), dir_updates.end(),
						[&,this](dirt_roadmap_node_t* node)
					{
						if( closest_node->cost_to_come+edge_cost+end_heuristic > node->cost_to_come + node->cost_to_go)
						{
							if(new_node_dir_radius + distance_function(node->point,eg.second->back()) < node->dir_radius)
							{
								prx_throw("Pruning not implemented...");
							}
						}
					});
			}

			//validity check
			bool valid = valid_check(*eg.second);
			
			if(!valid)
			{
				// std::cout << "Invalid edge!" << std::endl;
				delete eg.first;
				delete eg.second;
				eg = std::make_pair(nullptr,nullptr);
				iteration_count++;
				continue;
			}

			if (eg.first != nullptr)
			{
				add_edge_to_tree(eg, closest_node, dir_updates, new_node_dir_radius, condition);
				delete eg.first;
				delete eg.second;
			}

			iteration_count++;
		}
		while(!condition->check());
		// print_statistics();
	}

    void dirt_roadmap_t::add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg,
		dirt_roadmap_node_t* closest_node,
		std::vector<dirt_roadmap_node_t*> dir_updates,
		double new_node_dir_radius,
		condition_check_t* condition
	)
	{
		auto node_index = tree.add_vertex<dirt_roadmap_node_t,rrt_edge_t>();
		auto new_tree_node = tree.get_vertex_as<dirt_roadmap_node_t>(node_index);
		new_tree_node->point = state_space->clone_point(eg.second->back());
		new_tree_node->bridge = true;
		new_tree_node->is_safety_node = false;
		edge_index_t edge_index = tree.add_edge(closest_node->get_index(),node_index);
		auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
		new_edge->plan = std::make_shared<plan_t>(*eg.first);
		new_edge->traj = std::make_shared<trajectory_t>(*eg.second);
		new_edge->edge_cost = cost_function(*eg.second,*eg.first);;
		new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
		new_tree_node->cost_to_go = h(eg.second->back(),dirt_query->goal_state);;
		new_tree_node->roadmap_cost_to_go = roadmap_h(eg.second->back());
		new_tree_node->blossom_number = dirt_spec->blossom_number;
		new_tree_node->dir_radius = new_node_dir_radius;

		// new_tree_node->achieved_goal = closest_node->achieved_goal;
		new_tree_node->reachable_goal = closest_node->reachable_goal;

		// log_trajectory(new_tree_node, 'A', condition->time());
	
		max_radius = std::max(max_radius,new_node_dir_radius);
		// EXPERIMENTAL: Try commenting this line out. Behavior seems reasonable, but need to consider theoretical effects
		// get_vertex(start_vertex)->dir_radius = max_radius;

		std::for_each(dir_updates.begin(), dir_updates.end(),
				[&,this](dirt_roadmap_node_t* node)
			{
				const double sibling_distance = distance_function(node->point,new_tree_node->point);
				if( new_tree_node->cost_to_come+new_tree_node->cost_to_go < node->cost_to_come + node->cost_to_go)
				{
					node->dir_radius = std::min(node->dir_radius,sibling_distance);
					if(dirt_spec->use_pruning && node->dir_radius + sibling_distance < new_node_dir_radius)
					{
						prx_throw("Pruning not implemented...");
					}
				}
			});
		if(new_tree_node->cost_to_go < closest_node->cost_to_go || new_tree_node->roadmap_cost_to_go < closest_node->roadmap_cost_to_go)
		{
			child_extension = true;
		}
		previous_child=node_index;
		metric->add_node(new_tree_node.get());
		new_tree_node->bridge = false;
		update_goal(node_index, condition);
	}

    void dirt_roadmap_t::update_goal(node_index_t node_index, condition_check_t* condition)
	{
		auto new_tree_node = tree.get_vertex_as<dirt_roadmap_node_t>(node_index);
		// if(distance_function(dirt_query->goal_state,new_tree_node->point)<dirt_query->goal_region_radius)
		if( dirt_query -> goal_check(new_tree_node->point) )
		{
			if(goal_vertex==start_vertex || tree.get_vertex_as<dirt_roadmap_node_t>(goal_vertex)->cost_to_come > new_tree_node->cost_to_come)
			{
				condition -> report_new_solution();
				current_solution=new_tree_node->cost_to_come;
				current_solution_time = timer.measure();
				current_solution_iters = iteration_count;
				current_solution_sim_time = simulation_time;
				goal_vertex = node_index;
				std::cout <<"[dirt] Found new goal: "<<state_space->print_point(new_tree_node->point,3);
				std::cout <<" cost:"<<new_tree_node->cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << metric->get_nr_nodes();
				std::cout<< " sim time: " << simulation_time << std::endl;
				bnb(start_vertex,current_solution);
				tree.remove_vertices();
			}
		}
	}

	void dirt_roadmap_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

	void dirt_roadmap_t::log_trajectory(std::shared_ptr<dirt_roadmap_node_t> node, char action, double time)\
	{
		std::string log_fname = output_path + "log.txt";
		log_fout.open(log_fname.c_str(), std::fstream::app);

		if (dirt_query -> get_visualization)
		{
			log_fout << action << "," << time;

			double step = 0.1;
			edge_index_t e_idx;
			std::shared_ptr<rrt_edge_t> e_ptr;
			std::shared_ptr<trajectory_t> traj_ptr;
			switch(action)
			{
				case 'A':
					// log_fout << ",i," << node->get_index();
					e_idx = node->get_parent_edge();
					e_ptr = tree.get_edge_as<rrt_edge_t>(e_idx);
					traj_ptr = e_ptr -> traj;

					for (double i = 0; i <= e_ptr->edge_cost; i += 0.1)
					{
						auto s = traj_ptr->at(i);
						log_fout << ",T," << state_space -> print_point(s, 3);
					}
					break;
				case 'D':
					// log_fout << ",i," << node->get_index();
					break;
			}
		}
		log_fout << std::endl;
		log_fout.close();
	}

	void dirt_roadmap_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		auto node = get_vertex(v);
		bool res = delete_flag || node->cost_to_come + node->cost_to_go > cost_bound;
		if(v == goal_vertex)
			res = false;
		std::list<node_index_t> children = node->get_children();
		for(auto child : children)
		{
			bnb(child,cost_bound,res);
		}
		if(res && is_leaf(v))
		{

			//remove the node that was previously there
			if(!node->bridge)
			{
				metric->remove_node(node);
				node->bridge = true;
			}
			for (int man_index = 0; man_index < node->edge_generators.size(); man_index++)
			{
				delete node->edge_generators[man_index].first;
				delete node->edge_generators[man_index].second;
			}
			node->edge_generators.clear();
			node->indices.clear();

			//remove the node
			// tree.remove_vertex(v);
			auto node_ptr = tree.get_vertex_as<dirt_roadmap_node_t>(v);
			tree.mark_vertex_for_removal(v);
		}
	}
}