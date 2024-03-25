#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"
#include "prx/utilities/defs.hpp"

namespace prx
{   
    class roadmap_node_t : public undirected_node_t
    {
    public:
    roadmap_node_t()
    {
        cost_to_come = 0;
    }
    virtual ~roadmap_node_t()
    {
    }
        double cost_to_come;
        space_point_t state;
    };

    class roadmap_edge_t : public undirected_edge_t
    {
    public:
        roadmap_edge_t()
        {
            // weight = 0;
            collisions = 0;
        }
        virtual ~roadmap_edge_t()
        {
        }

        /*
        void set_weight(double weight){
            this->weight = weight;
        }*/

        // double weight;
        unsigned short collisions;
    };

    class roadmap_specification_t : public planner_specification_t
    {
        public:
            roadmap_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
            {
                _sg = sg;
                state_space = sg->get_state_space();
                config_space = state_space; // TODO: does this actually create a copy?

                distance_function = [](const space_point_t& s1, const space_point_t& s2) { return space_t::euclidean_2d(s1, s2); };
                sample_state = [this](space_point_t& s) { default_sample_state(s, state_space); };

                // Kevin: roadmap class was created with simple path planning in mind, 
                //        so the default implementation will not work
                /*
                propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj) {
                    default_propagate(start_state, plan, out_traj, sg);
                };*/

                // interpolate = [space=this->config_space](space_point_t& start_config, space_point_t& goal_config, trajectory_t& out_traj, int nsteps)
                interpolate = [](space_t* space, space_point_t& start_config, space_point_t& goal_config, trajectory_t& out_traj, int nsteps)
                {
                    default_interpolate(space, start_config, goal_config, out_traj, nsteps);
                };

                state_to_config = [](const space_point_t& state, space_point_t& config){
                    for(int i = 0; i < config->get_dim(); i++){
                        config->at(i) = state->at(i);
                    }
                };

                valid_state = [this, cg](space_point_t& s) { return default_valid_state(s, state_space, cg); };
                valid_check = [&](trajectory_t& traj) { return default_valid_trajectory(traj, valid_state); };
            }
            virtual ~roadmap_specification_t()
            {
            }

            std::shared_ptr<system_group_t> _sg;

            std::function<void(const space_point_t&, space_point_t&)> state_to_config; 

            distance_function_t distance_function;
            interpolate_t interpolate;
            // propagate_t propagate = 0; // interpolation??

            valid_state_t valid_state;
            valid_trajectory_t valid_check;

            sample_state_t sample_state;

            space_t* state_space;
            space_t* config_space;
    };

    class roadmap_query_t : public planner_query_t
    {
        public:
            // control space will not be used in roadmap query
            roadmap_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space, control_space)
            {
                clear_outputs();

                goal_region_radius = 0.5;

                goal_check = [&](space_point_t s) { return default_goal_check(s, goal_state, goal_region_radius); };
            }
            virtual ~roadmap_query_t()
            {
            }

            double goal_region_radius;
    };

    class roadmap_t : public planner_t
    {
        public:
            roadmap_t(const std::string& new_name);

            virtual ~roadmap_t(){
                _reset();
            }

            virtual void _link_and_setup_spec(planner_specification_t* spec) override;
            virtual bool _preprocess() override;
            virtual bool _link_and_setup_query(planner_query_t* query) override;
            virtual void _resolve_query(condition_check_t* condition) override;
            virtual void _fulfill_query() override;
            virtual void _reset() override;

            roadmap_specification_t* roadmap_spec;
            roadmap_query_t* roadmap_query;

        protected:
            // This can be a forward kinematics function
            std::function<void(const space_point_t&, space_point_t&)> state_to_config; 
            void msmo_astar();

            std::string planner_name;

            node_index_t start_vertex;
            node_index_t goal_vertex;

            distance_function_t distance_function;
            interpolate_t interpolate;
            // propagate_t propagate; // interpolation??

            valid_state_t valid_state;
            valid_trajectory_t valid_check;

            sample_state_t sample_state;

            undirected_graph_t roadmap;
            graph_nearest_neighbors_t* metric;
            int k;

            space_t* state_space;
            space_t* config_space; // configuration space of the heuristic roadmap

            space_point_t sampled_state;
            space_point_t sampled_config;

            long unsigned iteration_count;
            timer_t timer;

            double current_solution;
            long unsigned current_solution_iters;
            double current_solution_time;

            int print_statistics_count;

    };
}