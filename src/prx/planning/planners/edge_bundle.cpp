#include "prx/planning/planners/edge_bundle.hpp"

namespace prx
{
    edge_bundle_planner_t::edge_bundle_planner_t(const std::string& new_name) : planner_t(new_name)
    {
        metric = nullptr;
        planner_name = new_name;
    }

    edge_bundle_planner_t::~edge_bundle_planner_t()
    {
        _reset();
    }

    void edge_bundle_planner_t::_reset()
    {
        tree.purge();
        bundle.clear();
        if(metric!=nullptr)
            delete metric;
        metric = nullptr;
    }

    void edge_bundle_planner_t::_link_and_setup_spec(planner_specification_t* spec)
    {
        this->spec = dynamic_cast<edge_bundle_planner_specification_t*>(spec);
        prx_assert(this->spec!=nullptr,"EB received an incorrect specification.");
        
        cost_function = this->spec->cost_function;
        distance_function = this->spec->distance_function;
        sample_state = this->spec->sample_state;
        sample_plan = this->spec->sample_plan;
        valid_check = this->spec->valid_check;
        valid_state = this->spec->valid_state;
        propagate = this->spec->propagate;
        h = this->spec->h;

        num_edges = this->spec->num_edges;
        neighborhood_radius = this->spec->neighborhood_radius;

        state_space = this->spec->state_space;
        sample_point = state_space->make_point();
        control_space = this->spec->control_space;
        metric = new graph_nearest_neighbors_t(distance_function);
    }

    bool edge_bundle_planner_t::_preprocess()
    {
        metric->clear();
        tree.allocate_memory<edge_bundle_planner_node_t,edge_bundle_planner_edge_t>(1000);

        unsigned counter = 0;

        trajectory_t traj(state_space);
        plan_t plan(control_space);

        for (int i = 0; i < num_edges; ++i)
        {
            traj.clear();plan.clear();
            sample_state(sample_point);
            sample_plan(plan,sample_point);
            propagate(sample_point,plan,traj);
            if (valid_check(traj))
            {
                edge_bundle_node_t* node = new edge_bundle_node_t();
                node->point = state_space -> clone_point(sample_point);
                node->traj = std::make_shared<trajectory_t>(traj);
                node->plan = std::make_shared<plan_t>(plan);
                node->set_index(counter);
                bundle.insert(std::make_pair(counter,node));
                counter++;
                metric->add_node(node);
            }
            output_progress_bar(i*1.0/num_edges);
        }

        std::cout << "Constructed an edge bundle with " << counter << " edges." << std::endl;
        
        return true;
    }

    bool edge_bundle_planner_t::_link_and_setup_query(planner_query_t* query)
    {
        this->query = dynamic_cast<edge_bundle_planner_query_t*>(query);
        prx_assert(this->query!=nullptr,"EB received an incorrect query type.");

        start_vertex = tree.add_vertex<edge_bundle_planner_node_t,edge_bundle_planner_edge_t>();
        goal_vertex = start_vertex;
        auto start_node = tree.get_vertex_as<edge_bundle_planner_node_t>(start_vertex);
        start_node->point = state_space->clone_point(this->query->start_state);
        start_node->cost_to_come = 0;
        start_node->cost_to_go = h(start_node->point,this->query->goal_state);

        auto astar_node = new astar_node_t(start_vertex,start_node->cost_to_come,start_node->cost_to_go);
        open_set.insert(astar_node);

        previous_child = start_vertex;
        child_extension = false;

        timer.reset();
        iteration_count = 0;
        current_solution = 0;
        current_solution_iters = 0;
        current_solution_time = 0;
        return true;
    }

    void edge_bundle_planner_t::_resolve_query(condition_check_t* condition)
    {
        double edge_cost;
        do
        {
            if (!child_extension)
            {
                auto astar_node = open_set.remove_min();
                previous_child = astar_node->vertex;
            }
            child_extension = false;
            auto tree_node = tree.get_vertex_as<edge_bundle_planner_node_t>(previous_child);

            std::vector<edge_bundle_node_t*> neighbors;
            auto prox_nodes = metric -> radius_query(tree_node->point,neighborhood_radius);
            std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(neighbors),[](proximity_node_t* node)
            {
                return static_cast<edge_bundle_node_t*>(node);
            });

            
            trajectory_t traj(state_space);
            double current_best_cost_to_go = tree_node->cost_to_go;

            for (int i = 0; i < neighbors.size(); ++i)
            {
                auto neighbor = neighbors[i];

                traj.clear(); 
                propagate(tree_node->point,*(neighbor->plan),traj);
                edge_cost = cost_function(traj,*(neighbor->plan));
                if (valid_check(traj) && (goal_vertex == start_vertex || tree_node->cost_to_come + edge_cost < current_solution))
                {
                    auto node_index = tree.add_vertex<edge_bundle_planner_node_t,edge_bundle_planner_edge_t>();
                    auto new_tree_node = tree.get_vertex_as<edge_bundle_planner_node_t>(node_index);
                    new_tree_node->point = state_space->clone_point(traj.back());
                    new_tree_node->cost_to_come = tree_node->cost_to_come + edge_cost;
                    new_tree_node->cost_to_go = h(new_tree_node->point,query->goal_state);

                    edge_index_t edge_index = tree.add_edge(tree_node->get_index(),node_index);
                    auto new_edge = tree.get_edge_as<edge_bundle_planner_edge_t>(edge_index);
                    new_edge->plan = std::make_shared<plan_t>(*(neighbor->plan));
                    new_edge->traj = std::make_shared<trajectory_t>(traj);
                    new_edge->edge_cost = edge_cost;

                    auto new_astar_node = new astar_node_t(node_index,new_tree_node->cost_to_come,new_tree_node->cost_to_go);
                    open_set.insert(new_astar_node);

                    if (new_tree_node->cost_to_go < current_best_cost_to_go)
                    {
                        child_extension = true;
                        current_best_cost_to_go = new_tree_node->cost_to_go;
                        previous_child = node_index;
                    }

                    update_goal(node_index,condition);
                }
            }


            iteration_count++;
        }while(!condition->check() && !open_set.empty());

    }

    void edge_bundle_planner_t::_fulfill_query()
    {
        if (goal_vertex != start_vertex)
        {
            query->solution_cost = tree.get_vertex_as<edge_bundle_planner_node_t>(goal_vertex)->cost_to_come;
            std::deque<node_index_t> node_indices;
            node_index_t current_index = goal_vertex;
            while (current_index != start_vertex)
            {
                node_indices.push_front(current_index);
                current_index = tree[current_index]->get_parent();
            }

            query->solution_plan = *tree.get_edge_as<edge_bundle_planner_edge_t>(tree[node_indices[0]]->get_parent_edge())->plan;
            query->solution_traj = *tree.get_edge_as<edge_bundle_planner_edge_t>(tree[node_indices[0]]->get_parent_edge())->traj;
            for (int i = 1; i < node_indices.size(); ++i)
            {
                query->solution_traj.resize(query->solution_traj.size()-1);
                query->solution_plan += *tree.get_edge_as<edge_bundle_planner_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
                query->solution_traj += *tree.get_edge_as<edge_bundle_planner_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
            }
        }

        else
        {
            query->solution_cost=0.0;
        }

        if (query->get_visualization)
        {
            auto iter_bounds = tree.edges();
            for (auto iter = iter_bounds.first; iter != iter_bounds.second; ++iter)
            {
                query->tree_visualization.push_back(*tree.get_edge_as<edge_bundle_planner_edge_t>((*iter)->get_index())->traj);
            }
        }
    }

    void edge_bundle_planner_t::update_goal(node_index_t node_index, condition_check_t* condition)
    {
        auto new_tree_node = tree.get_vertex_as<edge_bundle_planner_node_t>(node_index);
        if (query->goal_check(new_tree_node->point))
        {
            if (goal_vertex == start_vertex || tree.get_vertex_as<edge_bundle_planner_node_t>(goal_vertex)->cost_to_come > new_tree_node->cost_to_come)
            {
                goal_vertex = node_index;
                current_solution = new_tree_node->cost_to_come;
                current_solution_iters = iteration_count;
                current_solution_time = timer.measure();
                goal_vertex = node_index;
                std::cout <<"[" + planner_name + "] Found new goal:"<<state_space->print_point(new_tree_node->point,3);
				std::cout <<" cost:"<<new_tree_node->cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << tree.num_vertices() <<std::endl;
                bnb(start_vertex,current_solution);
            }
        }
    }

    void edge_bundle_planner_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
    {
        auto node = tree.get_vertex_as<edge_bundle_planner_node_t>(v);
        const bool res = delete_flag || node->cost_to_come > cost_bound;
        std::list<node_index_t> children = node->get_children();
        for (auto child : children)
        {
            bnb(child,cost_bound,res);
        }
        children = node->get_children();
        if (res && children.empty())
        {
            open_set.remove(node->astar_node);
            tree.remove_vertex(v);
        }
    }
}