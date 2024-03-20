#include "prx/utilities/heuristics/roadmap.hpp"

namespace prx
{
    roadmap_t::roadmap_t(const std::string& new_name) : planner_t(new_name)
    {
        metric = nullptr;
        planner_name = new_name;
    }

    /*
    roadmap_t::~roadmap_t()
    {
        _reset();
    }
    */

    void roadmap_t::_link_and_setup_spec(planner_specification_t* spec)
    {
        roadmap_spec = dynamic_cast<roadmap_specification_t*>(spec);
        prx_assert(roadmap_spec != nullptr, "Roadmap received an incorrect specification.");
        distance_function = roadmap_spec->distance_function;
        sample_state = roadmap_spec->sample_state;
        propagate = roadmap_spec->propagate;

        state_space = roadmap_spec->state_space;
        config_space = roadmap_spec->config_space;

        state_to_config = roadmap_spec->state_to_config;

        sampled_state = state_space->make_point();

        metric = new graph_nearest_neighbors_t(distance_function);
    }

    bool roadmap_t::_preprocess()
    {
        roadmap.allocate_memory<roadmap_node_t, roadmap_edge_t>(1000);
        return true;
    }

    bool roadmap_t::_link_and_setup_query(planner_query_t* query)
    {
        roadmap_query = dynamic_cast<roadmap_query_t*>(query);
        prx_assert(roadmap_query != nullptr, "Roadmap received an incorrect query type.");
        if (roadmap.num_vertices() == 0 || 
        !state_space->equal_points(roadmap.get_vertex_as<roadmap_node_t>(start_vertex)->point, roadmap_query->start_state))
        {
            metric->clear();
            roadmap.clear();
            start_vertex = roadmap.add_vertex<roadmap_node_t, roadmap_edge_t>();
            goal_vertex = start_vertex;
            auto start_node = roadmap.get_vertex_as<roadmap_node_t>(start_vertex);
            start_node->point = state_space->clone_point(roadmap_query->start_state);
            start_node->cost_to_come=0;
        }
        timer.reset();
        iteration_count = 0;
        current_solution = 0;
        current_solution_iters = 0;
        current_solution_time = 0;

        return true;
    }

    void roadmap_t::_resolve_query(condition_check_t* condition)
    {
        do
        {
            sample_state(sampled_state);
            
            space_point_t sampled_config = config_space->make_point();
            state_to_config(sampled_state, sampled_config);
            iteration_count++;

            node_index_t new_node_index = roadmap.add_vertex<roadmap_node_t, roadmap_edge_t>(); 
            auto new_node = roadmap.get_vertex_as<roadmap_node_t>(new_node_index);
            new_node->point = config_space->clone_point(sampled_config);

        }
        while (!condition->check());
    }

    void roadmap_t::_fulfill_query()
    {

    }

    void roadmap_t::_reset()
    {
        // clear the stuff
        roadmap.purge();
        if (metric != nullptr)
        {
            delete metric;
            metric = nullptr;
        }
        config_space = nullptr;
    }
}