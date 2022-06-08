#include "prx/planning/planners/dirt_replan.hpp"
namespace prx
{

	dirt_replan_t::dirt_replan_t(const std::string& new_name) : rrt_t(new_name)
	{
		metric = nullptr;
		child_extension = true;
		max_radius = 0;
		planner_name = "DIRT";
	}
	dirt_replan_t::~dirt_replan_t()
	{
		_reset();
	}
	void dirt_replan_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		rrt_t::_link_and_setup_spec(spec);
		//reset is always called before this
		dirt_spec = dynamic_cast<dirt_replan_specification_t*>(spec);
		prx_assert(dirt_spec!=nullptr,"DIRT received an incorrect specification.");

		expand = dirt_spec->expand;
		h = dirt_spec->h;
        horizon = dirt_spec->horizon;
	}
	bool dirt_replan_t::_preprocess()
	{
		tree.allocate_memory<dirt_replan_node_t,rrt_edge_t>(1000);
		return true;
	}
	bool dirt_replan_t::_link_and_setup_query(planner_query_t* query)
	{
		rrt_query = dynamic_cast<rrt_query_t*>(query);
		prx_assert(rrt_query!=nullptr,"DIRT received an incorrect query type.");
		dirt_replan_query = dynamic_cast<dirt_replan_query_t*>(query);
		prx_assert(dirt_replan_query!=nullptr,"DIRT received an incorrect query type.");
		if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			tree.clear();

			start_vertex = tree.add_vertex<dirt_replan_node_t,rrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<dirt_replan_node_t>(start_vertex);
			// std::cout<<rrt_query->start_state<<std::endl;
			start_node->point = state_space->clone_point(rrt_query->start_state);
			start_node->cost_to_come = 0;
			start_node->dir_radius = 0;
			start_node->checkpoint_time = dirt_replan_query->start_time;
			start_node->cost_to_go = h(start_node->point,dirt_replan_query->goal_state);
			start_node->blossom_number = dirt_spec->blossom_number;
			metric->add_node(start_node.get());
			previous_child = start_vertex;
            best_node = start_vertex;
            best_cost = start_node->cost_to_go;
			child_extension = true;
		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;

		// Removed by BNB, Removed by pruning, Removed by collision check, Final
		random_edges_counter = {0,0,0,0};
		blossom_edges_counter = {0,0,0,0};

		return true;
	}

	void dirt_replan_t::_resolve_query(condition_check_t* condition)
	{
		// run for a certain amount of time
		do
		{
			if(!child_extension)
			{
				//sample state
				sample_state(sample_point);

				//find closest
				std::vector<dirt_replan_node_t*> nodes;
				node_index_t closest_index;
				double best_distance = PRX_INFINITY;
				{
					auto prox_nodes = metric->radius_and_closest_query(sample_point,max_radius);
					std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(nodes),[]
						(proximity_node_t* prox_node)
						{
							return static_cast<dirt_replan_node_t*>(prox_node);
						});
					nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
						[&,this](dirt_replan_node_t* node)
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
						return static_cast<dirt_replan_node_t*>(prox_node);
					});
					nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
						[&,this](dirt_replan_node_t* node)
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
				// std::cout << "Node selected for expansion: " << state_space->print_point(get_vertex(previous_child)->point) << std::endl;
			}
			child_extension = false;

			auto closest_node = get_vertex(previous_child);

			bool is_blossom_expand = false;
			if(closest_node->indices.size()==0)
			{
				//if no maneuvers, get some
				closest_node->edge_generators.clear();
				std::vector<plan_t*> plans;
				std::vector<trajectory_t*> trajs;
				expand(closest_node->point,plans,trajs,closest_node->blossom_number,!(closest_node->is_blossom_expand_done));



				if (closest_node->is_blossom_expand_done) closest_node->random_expand = true;
				else closest_node->is_blossom_expand_done = true;

				closest_node->blossom_number=1;
				for(int i=0;i<plans.size();i++)
				{
                    if (plans[i]->duration() + closest_node->checkpoint_time > horizon)
                    {
                        plans[i]->reduce_last_control(plans[i]->duration() + closest_node->checkpoint_time - horizon);
                        propagate(closest_node->point,*plans[i],*trajs[i]);
                    }
                    if (plans[i]->duration() > 0)
                        closest_node->edge_generators.push_back(std::make_pair(plans[i],trajs[i]));
				}

				std::vector<double> pred_values;
				int index = 0;
				for(auto& temp_eg : closest_node->edge_generators)
				{
					pred_values.push_back(h(temp_eg.second->back(),dirt_replan_query->goal_state));
					closest_node->indices.push_back(index++);
				}
				std::sort(closest_node->indices.begin(),closest_node->indices.end(),
					[this,pred_values](const int& a, const int& b)
					{
						return pred_values[a]>pred_values[b];
					});
			}

			// is_blossom_expand = (closest_node->edge_generators.size() > 1);
			is_blossom_expand = (closest_node->is_blossom_expand_done) && !closest_node->random_expand;
			std::pair<plan_t*,trajectory_t*> eg = std::make_pair(nullptr,nullptr);
			double edge_cost;
			double end_heuristic;
			double new_node_dir_radius;
			std::vector<dirt_replan_node_t*> dir_updates;

			if(closest_node->indices.size() == 0)
			{
				// This can happen if you are not using default expand (like curate).
				// Essentially, that procedure fails for some reason, and you end up adding no edge to the tree.
				// So we increment the counter for blossom expand collisions.
				blossom_edges_counter.at(2) += dirt_spec->blossom_number;
			}

			while(closest_node->indices.size()!=0)
			{
				eg = closest_node->edge_generators[closest_node->indices.back()];
				closest_node->edge_generators[closest_node->indices.back()] = std::make_pair(nullptr,nullptr);
				edge_cost = cost_function(*eg.second,*eg.first);
				end_heuristic = h(eg.second->back(),dirt_replan_query->goal_state);
				closest_node->indices.pop_back();

				//bnb
				if((goal_vertex!=start_vertex && 
					closest_node->cost_to_come + edge_cost + closest_node->cost_to_go > current_solution))
				{
					delete eg.first;
					delete eg.second;
					eg = std::make_pair(nullptr,nullptr);
					if (is_blossom_expand)
					{
						blossom_edges_counter.at(0)++;
					}
					else
					{
						random_edges_counter.at(0)++;
					}
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
					return static_cast<dirt_replan_node_t*>(prox_node);
				});
				std::for_each(dir_updates.begin(), dir_updates.end(),
						[&,this](dirt_replan_node_t* node)
					{
						if( closest_node->cost_to_come+edge_cost+end_heuristic > node->cost_to_come + node->cost_to_go)
						{
							new_node_dir_radius = std::min(new_node_dir_radius,distance_function(eg.second->back(),node->point));
						}
					});

				if(dirt_spec->use_pruning)
				{
					bool delete_node = false;
					std::for_each(dir_updates.begin(), dir_updates.end(),
							[&,this](dirt_replan_node_t* node)
						{
							if( !delete_node && closest_node->cost_to_come+edge_cost+end_heuristic > node->cost_to_come + node->cost_to_go)
							{
								if(new_node_dir_radius + distance_function(node->point,eg.second->back()) < node->dir_radius)
								{
									delete_node = true;
								}
							}
						});
					if(delete_node)
					{
						delete eg.first;
						delete eg.second;
						eg = std::make_pair(nullptr,nullptr);
						if (is_blossom_expand)
						{
							blossom_edges_counter.at(1)++;
						}
						else
						{
							random_edges_counter.at(1)++;
						}
						continue;
					}
				}

				//validity check
				bool valid = valid_check(*eg.second);
				bool new_node_added = false;
				double eg_dur = eg.first->duration();
				
				if(!valid)
				{
					delete eg.first;
					delete eg.second;
					eg = std::make_pair(nullptr,nullptr);
					if (is_blossom_expand)
					{
						blossom_edges_counter.at(2)++;
					}
					else
					{
						random_edges_counter.at(2)++;
					}
					continue;
				}
				else
				{
					if (is_blossom_expand)
					{
						blossom_edges_counter.at(3)++;
					}
					else
					{
						random_edges_counter.at(3)++;
					}
				}

				break;
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
		print_statistics();
	}

	void dirt_replan_t::add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg,
		dirt_replan_node_t* closest_node,
		std::vector<dirt_replan_node_t*> dir_updates,
		double new_node_dir_radius,
		condition_check_t* condition
	)
	{
		auto node_index = tree.add_vertex<dirt_replan_node_t,rrt_edge_t>();
		auto new_tree_node = tree.get_vertex_as<dirt_replan_node_t>(node_index);
		new_tree_node->point = state_space->clone_point(eg.second->back());
		new_tree_node->bridge = true;
		edge_index_t edge_index = tree.add_edge(closest_node->get_index(),node_index);
		auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
		new_edge->plan = std::make_shared<plan_t>(*eg.first);
		new_edge->traj = std::make_shared<trajectory_t>(*eg.second);
		new_edge->edge_cost = cost_function(*eg.second,*eg.first);;
		new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
		new_tree_node->cost_to_go = h(eg.second->back(),dirt_replan_query->goal_state);;
		new_tree_node->blossom_number = dirt_spec->blossom_number;
		new_tree_node->dir_radius = new_node_dir_radius;
        new_tree_node->checkpoint_time = closest_node->checkpoint_time + eg.first->duration();
        
        if (new_tree_node->cost_to_go < best_cost)
        {
            best_cost = new_tree_node->cost_to_go;
            best_node = node_index;
        }

		max_radius = std::max(max_radius,new_node_dir_radius);
		// EXPERIMENTAL: Try commenting this line out. Behavior seems reasonable, but need to consider theoretical effects
		// get_vertex(start_vertex)->dir_radius = max_radius;

		std::for_each(dir_updates.begin(), dir_updates.end(),
				[&,this](dirt_replan_node_t* node)
			{
				const double sibling_distance = distance_function(node->point,new_tree_node->point);
				if( new_tree_node->cost_to_come+new_tree_node->cost_to_go < node->cost_to_come + node->cost_to_go)
				{
					node->dir_radius = std::min(node->dir_radius,sibling_distance);
					if(dirt_spec->use_pruning && node->dir_radius + sibling_distance < new_node_dir_radius)
					{
						if(!node->bridge)
						{
							metric->remove_node(node);
							node->bridge = true;
						}
						node_index_t iter = node->get_index();
						while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
						{
							node_index_t next = get_vertex(iter)->get_parent();
							for(int man_index=0;man_index<get_vertex(iter)->indices.size();man_index++)
							{
								delete get_vertex(iter)->edge_generators[get_vertex(iter)->indices[man_index]].first;
								delete get_vertex(iter)->edge_generators[get_vertex(iter)->indices[man_index]].second;
							}
							get_vertex(iter)->edge_generators.clear();
							get_vertex(iter)->indices.clear();
							get_vertex(iter)->is_blossom_expand_done = false;
							get_vertex(iter)->random_expand = false;
							remove_leaf(iter);
							iter = next;
						}
					}
				}
			});
		if(new_tree_node->cost_to_go < closest_node->cost_to_go)
		{
			child_extension = true;
			previous_child=node_index;
		}
		metric->add_node(new_tree_node.get());
		new_tree_node->bridge = false;
		update_goal(node_index,condition);
	}

	void dirt_replan_t::update_goal(node_index_t node_index, condition_check_t* condition)
	{
		auto new_tree_node = tree.get_vertex_as<dirt_replan_node_t>(node_index);
		// if(distance_function(dirt_replan_query->goal_state,new_tree_node->point)<dirt_replan_query->goal_region_radius)
		if( dirt_replan_query -> goal_check(new_tree_node->point) )
		{
			if(goal_vertex==start_vertex || tree.get_vertex_as<dirt_replan_node_t>(goal_vertex)->cost_to_come > new_tree_node->cost_to_come)
			{
				current_solution=new_tree_node->cost_to_come;
				current_solution_time = timer.measure();
				current_solution_iters = iteration_count;
				goal_vertex = node_index;
				std::cout <<"[dirt] Found new goal: "<<state_space->print_point(new_tree_node->point,3);
				std::cout <<" cost:"<<new_tree_node->cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
				condition -> report_new_solution();
				bnb(start_vertex,current_solution);
			}
		}
	}

	std::vector<double> dirt_replan_t::get_statistics()
	{
		// time, iters, nodes, solution quality, first_time, first_iters, current_solution,
		std::vector<double> rrt_statistics = rrt_t::get_statistics();
		std::vector<double> rand_counts(random_edges_counter.begin(),random_edges_counter.end());
		std::vector<double> blossom_counts(blossom_edges_counter.begin(),blossom_edges_counter.end());
		rrt_statistics.insert(rrt_statistics.end(),
			std::make_move_iterator(rand_counts.begin()),
			std::make_move_iterator(rand_counts.end())
		);
		rrt_statistics.insert(rrt_statistics.end(),
			std::make_move_iterator(blossom_counts.begin()),
			std::make_move_iterator(blossom_counts.end())
		);
		return rrt_statistics;
	}

	void dirt_replan_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

    void dirt_replan_t::_fulfill_query()
    {
        if(goal_vertex!=start_vertex)
		{
			//backtrack to get the plan and trajectory
			rrt_query->solution_cost = tree.get_vertex_as<rrt_node_t>(goal_vertex)->cost_to_come;
			std::deque<node_index_t> node_indices;
			node_index_t current_index = goal_vertex;
			while(current_index!=start_vertex)
			{
				node_indices.push_front(current_index);
				current_index = tree[current_index]->get_parent();
			}

			rrt_query->solution_plan = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->plan;
			rrt_query->solution_traj = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->traj;

			for(int i=1;i<node_indices.size();i++)
			{
				rrt_query->solution_traj.resize(rrt_query->solution_traj.size()-1);
				rrt_query->solution_plan += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
				rrt_query->solution_traj += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
			}

		}
		else if (best_node != start_vertex)
		{
			rrt_query->solution_cost = tree.get_vertex_as<rrt_node_t>(best_node)->cost_to_come;
			std::deque<node_index_t> node_indices;
			node_index_t current_index = best_node;
			while(current_index!=start_vertex)
			{
				node_indices.push_front(current_index);
				current_index = tree[current_index]->get_parent();
			}

			rrt_query->solution_plan = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->plan;
			rrt_query->solution_traj = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->traj;

            for(int i=1;i<node_indices.size();i++)
			{
				rrt_query->solution_traj.resize(rrt_query->solution_traj.size()-1);
				rrt_query->solution_plan += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
				rrt_query->solution_traj += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
			}
		}
        else
        {
            rrt_query->solution_cost = 0;
        }
		if(rrt_query->get_visualization)
		{
	        	auto iter_bounds = tree.edges();
	        	for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
	        	{
				rrt_query->tree_visualization.push_back(*tree.get_edge_as<rrt_edge_t>((*iter)->get_index())->traj);
			}
		}
    }

	void dirt_replan_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
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
			for(int man_index=0;man_index<node->indices.size();man_index++)
			{
				delete node->edge_generators[node->indices[man_index]].first;
				delete node->edge_generators[node->indices[man_index]].second;
			}
			node->edge_generators.clear();
			node->indices.clear();

			//remove the node
			tree.remove_vertex(v);
		}
	}
}
