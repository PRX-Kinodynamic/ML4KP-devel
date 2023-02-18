#pragma once

#include "prx/planning/planners/dirt.hpp"

namespace prx
{
    class dirt_roadmap_node_t : public dirt_node_t
    {
        public:
        dirt_roadmap_node_t() : dirt_node_t()
        {
        }
        virtual ~dirt_roadmap_node_t() = default;
        std::vector<unsigned*> roadmap_path_indices;
		int expand_number;
		bool greedy_child;
    };
    class dirt_roadmap_specification_t : public dirt_specification_t
    { 
    public:
        dirt_roadmap_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : dirt_specification_t(sg,cg)
        {
        }
        virtual ~dirt_roadmap_specification_t() = default;

        roadmap_expand_t roadmap_expand;
    };

    class dirt_roadmap_query_t : public dirt_query_t
    {
    public:
        dirt_roadmap_query_t(space_t* state_space, space_t* control_space) : dirt_query_t(state_space, control_space)
        {
        }
        virtual ~dirt_roadmap_query_t() = default;
    };

    class dirt_roadmap_t : public dirt_t
    {
    public:
		dirt_roadmap_t(const std::string& new_name);
		virtual ~dirt_roadmap_t();

	protected:

		virtual void update_goal(node_index_t node_index) override;

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

		dirt_roadmap_specification_t* dirt_spec;
		dirt_roadmap_query_t* dirt_query;


		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;


	private:
        heuristic_function_t h;
        roadmap_expand_t roadmap_expand;

		double max_radius;
		bool child_extension;
		node_index_t previous_child;

		void add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg,
		dirt_roadmap_node_t* closest_node,
		std::vector<dirt_roadmap_node_t*> dir_updates,
		double new_node_dir_radius,
		condition_check_t* condition
		);

		dirt_roadmap_node_t* get_vertex(node_index_t v) const
		{
			return tree.get_vertex_as<dirt_roadmap_node_t>(v).get();
		}

		bool is_leaf(node_index_t v)
		{
			return (get_vertex(v)->get_children().empty());
		}

		void remove_leaf(node_index_t v)
		{
			prx_assert(is_leaf(v),"Trying to remove a tree node that is not a leaf!");

			if(!get_vertex(v)->bridge)
			{
				metric->remove_node(get_vertex(v));
				get_vertex(v)->bridge = true;
			}
			tree.remove_vertex(v);
		}

		bool is_best_goal(node_index_t v) const
		{
			node_index_t new_v = goal_vertex;
			while(get_vertex(new_v)->get_parent()!=new_v)
			{
				if(new_v == v)
					return true;
				new_v = get_vertex(new_v)->get_parent();
			}
			return false;

		}

    };
}