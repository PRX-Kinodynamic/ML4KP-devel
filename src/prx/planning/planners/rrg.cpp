#include "prx/planning/planners/rrg.hpp"

namespace prx
{
	rrg_t::rrg_t(const std::string& new_name) : planner_t(new_name)
	{
		metric = nullptr;
		planner_name = new_name;
	}
	rrg_t::~rrg_t()
	{
		_reset();
	}
	void rrg_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		//reset is always called before this
		rrg_spec = dynamic_cast<rrg_specification_t*>(spec);
		prx_assert(rrg_spec!=nullptr,"RRg received an incorrect specification.");
		distance_function = rrg_spec -> distance_function;
		cost_function = rrg_spec -> cost_function;
		sample_state = rrg_spec -> sample_state;
		sample_plan = rrg_spec -> sample_plan;
		valid_check = rrg_spec -> valid_check;
		steer = rrg_spec -> steer;
		// valid_stop_check = rrg_spec->valid_stop_check;
		// propagate = rrg_spec->propagate;
		// use_replanning = rrg_spec->use_replanning;
		// expand = rrg_spec->expand;


		state_space = rrg_spec->state_space;
		sample_point = state_space->make_point();
		control_space = rrg_spec->control_space;
		metric = new graph_nearest_neighbors_t(distance_function);

		k_rrg = rrg_spec -> k_rrg;
		//we now have spaces and necessary functions
	}

	bool rrg_t::_preprocess()
	{
		graph.allocate_memory<rrg_node_t,rrg_edge_t>(1000);
		return true;
	}

	bool rrg_t::_link_and_setup_query(planner_query_t* query)
	{
		rrg_query = dynamic_cast<rrg_query_t*>(query);
		prx_assert(rrg_query!=nullptr,"RRG received an incorrect query type.");
		if(graph.num_vertices()==0 || !state_space->equal_points(graph.get_vertex_as<rrg_node_t>(start_vertex)->point,rrg_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			graph.clear();
			start_vertex = graph.add_vertex<rrg_node_t,rrg_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = graph.get_vertex_as<rrg_node_t>(start_vertex);
			start_node->point = state_space->clone_point(rrg_query->start_state);
			start_node->cost_to_come = 0;
			metric->add_node(start_node.get());

		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;
		return true;
	}
	
	void rrg_t::_resolve_query(condition_check_t* condition)
	{
		prx_warn_cond(rrg_spec->blossom_number == 1, "RRg only uses a blossom number of 1")
		//run for a certain amount of time
		do
		{
			//sample state
			sample_state(sample_point);
			//find closest
			auto closest_node = static_cast<rrg_node_t*>(metric->single_query(sample_point));
			
			plan_t plan(control_space);
			trajectory_t traj(state_space);
			bool connected = steer(closest_node->point, sample_point, plan, traj);
			if (!connected)
			{
				iteration_count++;
				continue;
			} 

			// std::vector<plan_t*> plans;
			// std::vector<trajectory_t*> trajs;
			// expand(closest_node->point,plans,trajs,rrg_spec->blossom_number,false);
			// plan_t plan(*plans.front());
			// trajectory_t traj(*trajs.front());
			double edge_cost = cost_function(traj,plan);

			double new_cost = closest_node -> cost_to_come + edge_cost;
			double new_duration = closest_node -> duration + plan.duration();
			//collision check && bnb && glc_conds
			if((goal_vertex==start_vertex || closest_node->cost_to_come + edge_cost < current_solution) 
				&& valid_check(traj))
			{
				auto X_near = metric -> multi_query(traj.back(), k_rrg);
				//add node
				auto node_index = graph.add_vertex<rrg_node_t, rrg_edge_t>();
				auto new_tree_node = graph.get_vertex_as<rrg_node_t>(node_index);
				new_tree_node->point = state_space->clone_point(traj.back());
				metric->add_node(new_tree_node.get());
				edge_index_t edge_index = graph.add_edge<rrg_edge_t>(closest_node->get_index(),node_index);
				auto new_edge = graph.get_edge_as<rrg_edge_t>(edge_index);
				new_edge -> plan = std::make_shared<plan_t>(plan);
				new_edge -> traj = std::make_shared<trajectory_t>(traj);
				new_edge -> edge_cost = edge_cost;
				new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;

				new_tree_node -> duration = new_duration;
				for (auto x : X_near)
				{
					auto x_near = static_cast<rrg_node_t*>(x);
					plan_t plan_near(control_space);
					trajectory_t traj_near(state_space);
					connected = steer(new_tree_node -> point, x_near -> point, plan_near, traj_near);
					if (connected)
					{
						edge_index_t edge_index = graph.add_edge<rrg_edge_t>(node_index, x_near -> get_index());						
						auto new_edge = graph.get_edge_as<rrg_edge_t>(edge_index);
						new_edge -> plan = std::make_shared<plan_t>(plan);
						new_edge -> traj = std::make_shared<trajectory_t>(traj);
						new_edge -> edge_cost = edge_cost;
					} 
				}
				update_goal(node_index);
			}
			iteration_count++;
		}
		while(!condition->check());
	}
	void rrg_t::_fulfill_query()
	{
		if(goal_vertex!=start_vertex)
		{
			
			auto cmp = [](std::shared_ptr<rrg_node_t> a, std::shared_ptr<rrg_node_t> b)
			{
				return !(a -> cost_to_come < b -> cost_to_come);
			};

			auto start_node = graph.get_vertex_as<rrg_node_t>(start_vertex);
			// goal_node -> cost_to_come = 0.0;
		
			std::set<node_index_t> visited;
			std::priority_queue<std::shared_ptr<rrg_node_t>, std::vector<std::shared_ptr<rrg_node_t>>, decltype(cmp) > pq(cmp);

			pq.push(start_node);
			int i = 0;
			bool goal_found = false;
			while (!pq.empty() && !goal_found)
			{
				auto u = pq.top();
				pq.pop();

				auto search = visited.find(u -> get_index());

				if (search != std::end(visited)) continue;

				visited.insert(u -> get_index());
				// std::cout << "n: " << u -> index << " " << u -> point << " cost: " << u -> cost_to_go << std::endl;
				auto cost = u -> cost_to_come;
				for (auto e : u -> get_neighbors())
				{
					auto edge = graph.get_edge_as<rrg_edge_t>(e);
					auto nodes_ids = edge -> get_connected_nodes();
					auto candidate_id = nodes_ids.first == u -> get_index() ? nodes_ids.second : nodes_ids.first;
					auto candidate = graph.get_vertex_as<rrg_node_t>(candidate_id);

					if ( cost + edge -> edge_cost <= candidate -> cost_to_come)
					{
						// candidate -> cost_to_go = cost + edge -> value;
						candidate -> prev = u -> get_index();
						candidate -> prev_set = true;
						if (candidate -> get_index() == goal_vertex)
						{
							goal_found = true;
						}
						// goal_found = candidate -> get_index() == goal_vertex;
					}
					pq.push(candidate);

				}
			}
			if (!goal_found)
			{
				prx_throw("Goal not found?");
			}

			// rrg_query -> solution_cost = graph.get_vertex_as<rrg_node_t>(goal_vertex) -> cost_to_come;
			// std::deque<node_index_t> node_indices;
			// node_index_t current_index = goal_vertex;
			// while(current_index!=start_vertex)
			// {
			// 	node_indices.push_front(current_index);
			// 	current_index =  graph.get_vertex_as<rrg_node_t>(current_index) -> prev;// graph[current_index]->get_parent();
			// }

			// rrg_query->solution_plan = *tree.get_edge_as<rrg_edge_t>(graph[node_indices[0]]->get_parent_edge())->plan;
			// rrg_query->solution_traj = *tree.get_edge_as<rrg_edge_t>(graph[node_indices[0]]->get_parent_edge())->traj;

			// for(int i=1;i<node_indices.size();i++)
			// {
			// 	rrg_query->solution_traj.resize(rrt_query->solution_traj.size()-1);
			// 	rrg_query->solution_plan += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
			// 	rrg_query->solution_traj += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
			// }
		}
		else
		{
			rrg_query->solution_cost = 0;
		}
		
		if(rrg_query->get_visualization)
		{
	        auto iter_bounds = graph.edges();
	        for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
	        {
				rrg_query->tree_visualization.push_back(*graph.get_edge_as<rrg_edge_t>((*iter)->get_index())->traj);
			}
		}
	}

	std::vector<std::string> rrg_t::get_statistics_header()
	{
		return {"time", "iters", "nodes", "solution_cost", "solution_time", "solution_iters"};
	}
	std::vector<double> rrg_t::get_statistics()
	{
		//time, iters, nodes, solution quality, first_time, first_iters, current_solution
		return {timer.measure(),
				static_cast<double>(iteration_count),
				static_cast<double>(metric->get_nr_nodes()),
				current_solution,
				current_solution_time,
				static_cast<double>(current_solution_iters)
				};
	}

	void rrg_t::_reset()
	{
		//clear the stuff
		graph.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

	void rrg_t::update_goal(node_index_t node_index)
	{
		auto new_graph_node = graph.get_vertex_as<rrg_node_t>(node_index);
		// if(distance_function(rrg_query->goal_state,new_tree_node->point)<rrg_query->goal_region_radius)
		if(rrg_query->goal_check(new_graph_node -> point))
		{
			if(goal_vertex==start_vertex || graph.get_vertex_as<rrg_node_t>(goal_vertex) -> cost_to_come > new_graph_node -> cost_to_come)
			{
				current_solution = new_graph_node -> cost_to_come;
				current_solution_time = timer.measure();
				current_solution_iters = iteration_count;
				goal_vertex = node_index;
				//std::cout<<"Found new goal: "<<state_space->print_point(new_tree_node->point,3)<<" "<<new_tree_node->cost_to_come<<std::endl;
				std::cout <<"[" + planner_name + "] Found new goal:"<<state_space->print_point(new_graph_node->point,3);
				std::cout <<" cost:"<< new_graph_node -> cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
				// Current RRG is for trees, can it be extended for graphs?
				// bnb(start_vertex,current_solution);
			}
		}
	}

	void rrg_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		auto node = graph.get_vertex_as<rrg_node_t>(v);
		const bool res = delete_flag || node->cost_to_come > cost_bound;
		std::list<node_index_t> children = node -> get_neighbors();
		for(auto child : children)
		{
			bnb(child,cost_bound,res);
		}
		children = node -> get_neighbors();
		if(res && children.empty())
		{
			//remove the node
			metric->remove_node(node.get());
			graph.remove_vertex(v);
		}
	}

	void rrg_t::print_statistics()
	{	
		std::cout <<"[" + planner_name + "]";
				std::cout<< " time:" <<  timer.measure();
				std::cout<< " iter:" << iteration_count;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
	}

}
