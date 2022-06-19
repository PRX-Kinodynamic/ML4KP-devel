#include "prx/planning/planners/prm.hpp"

namespace prx
{
	prm_t::prm_t(const std::string& new_name) : planner_t(new_name)
	{
		metric = nullptr;
		planner_name = new_name;
	}
	prm_t::~prm_t()
	{
		_reset();
	}
	void prm_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		//reset is always called before this
		prm_spec = dynamic_cast<prm_specification_t*>(spec);
        prx_assert(prm_spec!=nullptr,"PRM received an incorrect specification!");
        cost_function = prm_spec->cost_function;
        distance_function = prm_spec->distance_function;
        valid_state = prm_spec->valid_state;
        valid_check = prm_spec->valid_check;
        sample_state = prm_spec->sample_state;
		local_planner = prm_spec->local_planner;

        state_space = prm_spec->state_space;
		control_space = prm_spec->control_space;
        sample_point = state_space->make_point();
        metric = new graph_nearest_neighbors_t(distance_function);

        k = prm_spec->k;
        M = prm_spec->M;
		//we now have spaces and necessary functions
		graph.clear();
		graph.allocate_memory<prm_node_t,prm_edge_t>(M);
	}

bool prm_t::_preprocess()
	{
	
	double val;
        metric -> clear();
        // bulk of the code will go here.
        unsigned iter_count = 0;
        do
        {
            // Sample a state.
            sample_state(sample_point);

            // Collision check
            if (valid_state(sample_point))
            {
				auto node_index = graph.add_vertex<prm_node_t,prm_edge_t>();
				auto node = graph.get_vertex_as<prm_node_t>(node_index);
				node->point = state_space->clone_point(sample_point);
				metric->add_node(node.get());
            }

            iter_count++;
        } while (iter_count < M);

		graph.vertex_list_to_file("output_v.txt");
		
		space_point_t candidate = state_space->make_point();
		
		unsigned trajectory_length = 1.0/simulation_step;

		auto it_pair = graph.vertices();
		
		//std::cout << it_pair.first->get() << std::endl;
		
		for (auto it = it_pair.first; it != it_pair.second; ++it)
		{
			state_space -> copy_point(sample_point, it->get()->point);
			// Gotta do k + 1 otherwise it returns the same node as well.
			auto neighbors = metric->multi_query(sample_point, k + 1);
			
			std::cout << "it " << it->get()->point << std::endl;
			std::cout << "start" << prm_query->start_state << std::endl;
			//std::cout << "sp " << sample_point << std::endl;
			
			for (auto nn : neighbors)
			{
				
				// std::vector<plan_t*> plans;
				// std::vector<trajectory_t*> trajs;
				// trajectory_t local_plan(*trajs.front());
				// plan_t dummy_plan(*plans.front());
				
				trajectory_t local_plan(state_space);
				plan_t dummy_plan(control_space);
				
				auto candidate_node = static_cast<prm_node_t*>(nn);
				state_space -> copy_point(candidate, candidate_node->point);
				local_planner(sample_point, candidate, local_plan, trajectory_length);
				
				if (valid_check(local_plan) && it->get()->get_index() != candidate_node->get_index())
				{	

					val = distance_function(it->get()->point, candidate_node->point);
					edge_index_t edge_index = graph.add_edge(it->get()->get_index(), candidate_node->get_index(), val);
					auto new_edge = graph.get_edge_as<prm_edge_t>(edge_index);
					
					// std::cout << new_edge -> first << "next" << new_edge -> second << std::endl;
					// *new_edge->traj = std::make_shared<trajectory_t>(local_plan);
				}
			}
		}
		graph.edge_list_to_file("output_e.txt");
		

        return true;
	}
	
	
	bool prm_t::_link_and_setup_query(planner_query_t* query)
	{
		prm_query = dynamic_cast<prm_query_t*>(query);
		prx_assert(prm_query!=nullptr,"prm received an incorrect query type.");
		
		if(graph.num_vertices()==0 || !state_space->equal_points(graph.get_vertex_as<prm_node_t>(start_vertex)->point,prm_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			//prm.clear();
			start_vertex = graph.add_vertex<prm_node_t,prm_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = graph.get_vertex_as<prm_node_t>(start_vertex);
			start_node->point = state_space->clone_point(prm_query->start_state);
			//start_node->cost_to_come = 0;
			metric->add_node(start_node.get());

		}
		// timer.reset();
		// iteration_count=0;
		// current_solution=0;
		// current_solution_iters=0;
		// current_solution_time=0;
		
		return true;
	}
	
	void prm_t::_resolve_query(condition_check_t* condition)
	{	
		// graph.get_vertex_as<prm_node_t>(start_vertex)->point
		/*for ( auto i_itr = 1, i_itr<=2, i_itr++)
		{	
			if(i_itr == 1)
			{
				state_space -> copy_point(sample_point, prm_query->start_state);
			}
			
			if (i_itr == 2)
			{
				state_space -> copy_point(sample_point, prm_query->goal_state);
			}
			auto neighbors = metric->multi_query(sample_point, k + 1);
		
			for (auto nn : neighbors)
			{
				
											
				trajectory_t local_plan(state_space);
				plan_t dummy_plan(control_space);
				
				auto candidate_node = static_cast<prm_node_t*>(nn);
				state_space -> copy_point(candidate, candidate_node->point);
				local_planner(sample_point, candidate, local_plan, trajectory_length);
				
				if (valid_check(local_plan) && it->get()->get_index() != candidate_node->get_index())
				{	
					val = distance_function(it->get()->point, candidate_node->point);
					edge_index_t edge_index = graph.add_edge(it->get()->get_index(), candidate_node->get_index(), val);
					auto new_edge = graph.get_edge_as<prm_edge_t>(edge_index);
					
					// std::cout << new_edge -> first << "next" << new_edge -> second << std::endl;
					// *new_edge->traj = std::make_shared<trajectory_t>(local_plan);
				}
			}
		}*/
	}
	void prm_t::_fulfill_query()
	{
		/*if(prm_query->get_visualization)
		{
	         	auto iter_bounds = graph.edges();
	         	for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
	        	{
		 		prm_query->tree_visualization.push_back(*graph.get_edge_as<prm_edge_t>((*iter)->get_index())->traj);
		 	}
		}*/
		
	}

	void prm_t::_reset()
	{
		//clear the stuff
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}
	}
}
