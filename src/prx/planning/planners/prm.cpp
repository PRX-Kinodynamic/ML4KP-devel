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

        state_space = prm_spec->state_space;
		control_space = prm_spec->control_space;
        sample_point = state_space->make_point();
        metric = new graph_nearest_neighbors_t(distance_function);

        k = prm_spec->k;
        M = prm_spec->M;
		//we now have spaces and necessary functions
	}

	bool prm_t::_preprocess()
	{
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
                auto new_node = graph.make_node();
                new_node.get().point = state_space -> make_point();
                state_space -> copy_point(new_node.get().point, sample_point);
				std::cout << "Adding state: " << state_space -> print_point(new_node.get().point,2) << std::endl;
                std::cout << new_node.get().get_index() << std::endl;
				metric->add_node(&new_node.get());
            }
			PRX_DEBUG_PRINT

            iter_count++;
        } while (iter_count < M);

		space_point_t candidate = state_space->make_point();
		trajectory_t local_plan(state_space);
		plan_t dummy_plan(control_space);
		unsigned trajectory_length = 1.0/simulation_step;

		for (auto n : graph.get_nodes())
		{
			state_space -> copy_point(sample_point, n.get().point);
			auto neighbors = metric->multi_query(sample_point, k);

			for (auto nn : neighbors)
			{
				auto candidate_node = static_cast<prm_node_t*>(nn);
				state_space -> copy_point(candidate, candidate_node->point);
				local_plan.clear();
				local_planner(sample_point, candidate, local_plan, trajectory_length);
				if (valid_check(local_plan))
				{
					auto new_edge = graph.make_edge(n.get().get_index(), candidate_node->get_index());
					new_edge.get().cost = cost_function(local_plan,dummy_plan);
				}
			}

			std::cout << neighbors.size() << std::endl;
		}

        return true;
	}

	bool prm_t::_link_and_setup_query(planner_query_t* query)
	{
		prm_query = dynamic_cast<prm_query_t*>(query);
		prx_assert(prm_query!=nullptr,"prm received an incorrect query type.");
		
		return true;
	}
	
	void prm_t::_resolve_query(condition_check_t* condition)
	{
		
	}
	void prm_t::_fulfill_query()
	{
		
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
