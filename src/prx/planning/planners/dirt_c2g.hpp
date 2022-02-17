#pragma once

#include "prx/planning/planners/dirt.hpp"

namespace prx
{
    typedef std::function<std::vector<double> (const std::vector<space_point_t>, const space_point_t&)> batch_heuristic_function_t;

	class dirt_c2g_specification_t : public dirt_specification_t
    {
        public:

        dirt_c2g_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : dirt_specification_t(sg,cg)
		{
			batch_h = [this](const std::vector<space_point_t> s1, const space_point_t& s)
            {
                std::vector<double> result = {0.0};
                return result;
            };
		}
		virtual ~dirt_c2g_specification_t(){}

        batch_heuristic_function_t batch_h;
    };

    class dirt_c2g_query_t : public dirt_query_t
	{
	public:
		dirt_c2g_query_t(space_t* state_space, space_t* control_space) : dirt_query_t(state_space,control_space)
		{
		}
		virtual ~dirt_c2g_query_t(){}

	};
    
    class dirt_c2g_t : public rrt_t
	{
	public:
		dirt_c2g_t(const std::string& new_name);
		virtual ~dirt_c2g_t();

		std::vector<long unsigned> random_edges_counter, blossom_edges_counter;

	protected:

		virtual void update_goal(node_index_t node_index) override;

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

		virtual std::vector<double> get_statistics() override;

		dirt_c2g_specification_t* dirt_spec;
		dirt_c2g_query_t* dirt_query;


		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;


	private:

		heuristic_function_t h;
        batch_heuristic_function_t batch_h;
		expand_t expand;

		double max_radius;
		bool child_extension;
		node_index_t previous_child;

		void add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg, dirt_node_t* closest_node, 
            std::vector<dirt_node_t*> dir_updates, double new_node_dir_radius, double node_h);

		dirt_node_t* get_vertex(node_index_t v) const
		{
			return tree.get_vertex_as<dirt_node_t>(v).get();
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
