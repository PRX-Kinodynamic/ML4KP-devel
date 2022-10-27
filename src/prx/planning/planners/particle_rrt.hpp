#pragma once

#include "prx/planning/planners/rrt.hpp"

#include "prx/planning/planner_functions/robust_planner_functions.hpp"

namespace prx
{
    class particle_rrt_node_t : public tree_node_t
    {
        public:

        std::vector<space_point_t> points;

        particle_rrt_node_t()
        {
        }

        virtual ~particle_rrt_node_t(){}
    };

    class particle_rrt_edge_t : public tree_edge_t
    {
        public:

        particle_rrt_edge_t()
        {
            edge_cost = 0;
        }

        virtual ~particle_rrt_edge_t(){}

        std::shared_ptr<plan_t> plan;
        std::vector<std::shared_ptr<trajectory_t>> trajs;
        double edge_cost;
    };

    class particle_rrt_specification_t : public rrt_specification_t
    {
        public:
        particle_rrt_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : rrt_specification_t(sg,cg)
        {
            prx_throw("Not implemented yet");
        }

        virtual ~particle_rrt_specification_t(){}

        propagate_particles_t propagate_particles;
        valid_particles_t valid_particles;
        compute_reachable_set_t compute_reachable_set;
    };

    class particle_rrt_query_t : public rrt_query_t
    {
        public:
        particle_rrt_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
        {
            prx_throw("Not implemented yet");
        }

        virtual ~particle_rrt_query_t(){}

        goal_check_particles_t goal_check_particles;
    };

    class particle_rrt_t : public rrt_t 
    {
        public:
        particle_rrt_t(const std::string& new_name);
        virtual ~particle_rrt_t();

        protected:
        virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

        virtual void update_goal(node_index_t node_index) override final;

        particle_rrt_specification_t* rrt_spec;
        particle_rrt_query_t* rrt_query;

        propagate_particles_t propagate_particles;
        valid_particles_t valid_particles;
        compute_reachable_set_t compute_reachable_set;

        private:
        tree_t nominal_tree;
        // Mapping from nominal tree nodes to particle tree nodes
        std::unordered_map<node_index_t, node_index_t> nominal_to_particle;
    };
}