#include "prx/planning/planners/particle_rrt.hpp"

namespace prx
{
    particle_rrt_t::particle_rrt_t(const std::string& new_name) : rrt_t(new_name)
    {
        metric = nullptr;
        planner_name = new_name;
    }
    particle_rrt_t::~particle_rrt_t()
    {
        _reset();
    }
    void particle_rrt_t::_link_and_setup_spec(planner_specification_t* spec)
    {
        rrt_t::_link_and_setup_spec(spec);
        rrt_spec = dynamic_cast<particle_rrt_specification_t*>(spec);
        prx_assert(rrt_spec!=nullptr,"particleRRT received an incorrect specification.");

        propagate_particles = rrt_spec->propagate_particles;
        valid_particles = rrt_spec->valid_particles;
        valid_trajectories = rrt_spec->valid_trajectories;

        num_particles = rrt_spec->num_particles;
    }
    bool particle_rrt_t::_preprocess()
    {
        nominal_tree.allocate_memory<rrt_node_t,rrt_edge_t>(1000);
        tree.allocate_memory<particle_rrt_node_t,particle_rrt_edge_t>(1000);
        return true;
    }
    bool particle_rrt_t::_link_and_setup_query(planner_query_t* query)
    {
        rrt_query = dynamic_cast<particle_rrt_query_t*>(query);
        prx_assert(rrt_query!=nullptr,"particleRRT received an incorrect query type.");
        if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
        {
            //clear existing data structure
            metric->clear();
            tree.clear();
            nominal_tree.clear();

            start_vertex = nominal_tree.add_vertex<rrt_node_t,rrt_edge_t>();
            goal_vertex = start_vertex;
            auto start_node = nominal_tree.get_vertex_as<rrt_node_t>(start_vertex);
            start_node->point = state_space->clone_point(rrt_query->start_state);
            start_node->cost_to_come = 0;
            metric->add_node(start_node.get());

            // Add start state to particle tree.
            node_index_t start_particle_vertex = tree.add_vertex<particle_rrt_node_t,particle_rrt_edge_t>();
            auto start_particle_node = tree.get_vertex_as<particle_rrt_node_t>(start_particle_vertex);
            for (int i = 0; i < num_particles; i++)
            {
                start_particle_node->points.push_back(state_space->clone_point(rrt_query->start_state));
            }

            // Add the mapping.
            nominal_to_particle[start_vertex] = start_particle_vertex;
        }
        timer.reset();
        iteration_count = 0; 
        current_solution = 0;
        current_solution_iters = 0;
        current_solution_time = 0;
        return true;
    }

    void particle_rrt_t::_resolve_query(condition_check_t* condition)
    {
        double edge_cost;
        double new_cost;
        do 
        {
            // Sample a node.
            sample_state(sample_point);
            // Find the nearest node.
            auto closest_node = static_cast<rrt_node_t*>(metric->single_query(sample_point));
            node_index_t closest_node_index = closest_node->get_index();
            node_index_t closest_particle_index = nominal_to_particle[closest_node_index];
            auto closest_particle_node = tree.get_vertex_as<particle_rrt_node_t>(closest_particle_index);

            std::vector<plan_t*> plans;
			std::vector<trajectory_t*> trajs;
            std::vector<plan_t*> particle_plans;
            std::vector<trajectory_t*> particle_trajs;
            expand(closest_node->point, plans, trajs, 1, false);
            plan_t plan(*plans.front());
            trajectory_t traj(*trajs.front());
            edge_cost = cost_function(traj, plan);
            new_cost = closest_node->cost_to_come + edge_cost;
            if (goal_vertex == start_vertex || new_cost < current_solution)
            {
                for (int i = 0; i < num_particles; i++)
                {
                    particle_plans.push_back(new plan_t(plan));
                }
                // Now propagate the particles;
                propagate_particles(closest_particle_node->points, particle_plans, particle_trajs);

                if (valid_trajectories(particle_trajs))
                {
                    auto nominal_node_index = nominal_tree.add_vertex<rrt_node_t,rrt_edge_t>();
                    auto nominal_node = nominal_tree.get_vertex_as<rrt_node_t>(nominal_node_index);
                    
                    nominal_node->point = state_space -> make_point();
                    for (int i = 0; i < num_particles; i++)
                    {
                        nominal_node->point->add_multiply(1.0/num_particles, particle_trajs[i]->back());
                    }

                    nominal_node->cost_to_come = new_cost;
                    metric->add_node(nominal_node.get());

                    auto particle_node_index = tree.add_vertex<particle_rrt_node_t,particle_rrt_edge_t>();
                    auto particle_node = tree.get_vertex_as<particle_rrt_node_t>(particle_node_index);
                    for (auto t : particle_trajs)
                    {
                        particle_node->points.push_back(state_space->clone_point(t->back()));
                    }

                    edge_index_t edge_index = nominal_tree.add_edge(closest_node_index, nominal_node_index);
                    auto edge = nominal_tree.get_edge_as<rrt_edge_t>(edge_index);
                    edge->plan = std::make_shared<plan_t>(plan);
                    edge->traj = std::make_shared<trajectory_t>(traj);
                    edge->edge_cost = edge_cost;

                    edge_index = tree.add_edge(closest_particle_index, particle_node_index);
                    auto particle_edge = tree.get_edge_as<particle_rrt_edge_t>(edge_index);
                    particle_edge->plan = std::make_shared<plan_t>(plan);
                    for (auto t : particle_trajs)
                    {
                        particle_edge->trajs.push_back(std::make_shared<trajectory_t>(*t));
                    }
                    particle_edge->edge_cost = edge_cost;

                    // Add the mapping.
                    nominal_to_particle[nominal_node_index] = particle_node_index;
                    update_goal(nominal_node_index);
                }
            }
            iteration_count++;
        } while(!condition->check());
    }

    void particle_rrt_t::update_goal(node_index_t node_index)
    {
        auto new_tree_node = nominal_tree.get_vertex_as<rrt_node_t>(node_index);
        auto new_particle_node = tree.get_vertex_as<particle_rrt_node_t>(nominal_to_particle[node_index]);
        if (rrt_query->goal_check_particles(new_particle_node->points))
        {
            if (goal_vertex == start_vertex || new_tree_node->cost_to_come < current_solution)
            {
                goal_vertex = node_index;
                current_solution = new_tree_node->cost_to_come;
                current_solution_iters = iteration_count;
                current_solution_time = timer.measure();
                std::cout <<"[" + planner_name + "] Found new goal:"<<state_space->print_point(new_tree_node->point,3);
				std::cout <<" cost:"<<new_tree_node->cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
                // TODO: Implement BNB
            }
        }
    }

    void particle_rrt_t::_fulfill_query()
    {
        if (goal_vertex != start_vertex)
        {
            rrt_query->solution_cost = nominal_tree.get_vertex_as<rrt_node_t>(goal_vertex)->cost_to_come;
            std::deque<node_index_t> node_indices;
            node_index_t current_index = goal_vertex;
            while (current_index != start_vertex)
            {
                node_indices.push_front(current_index);
                current_index = nominal_tree[current_index]->get_parent();
            }

            rrt_query->solution_plan = *nominal_tree.get_edge_as<rrt_edge_t>(nominal_tree[node_indices[0]]->get_parent_edge())->plan;

			for(int i=1;i<node_indices.size();i++)
			{
				rrt_query->solution_plan += *nominal_tree.get_edge_as<rrt_edge_t>(nominal_tree[node_indices[i]]->get_parent_edge())->plan;
			}
        }
        else
        {
            rrt_query->solution_cost = 0;
        }
        if (rrt_query->get_visualization)
        {
            auto iter_bounds = nominal_tree.edges();
            for (auto iter = iter_bounds.first; iter != iter_bounds.second; ++iter)
            {
                rrt_query->tree_visualization.push_back(*nominal_tree.get_edge_as<rrt_edge_t>((*iter)->get_index())->traj);
            }
        }
    }

    void particle_rrt_t::_reset()
    {
        tree.purge(); 
        nominal_tree.purge();
        nominal_to_particle.clear();
        if (metric != nullptr)
        {
            delete metric;
            metric = nullptr;
        }
    }
}