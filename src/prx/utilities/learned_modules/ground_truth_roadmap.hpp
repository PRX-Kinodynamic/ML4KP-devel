#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

using namespace prx;

struct ground_truth_vertex_t
{
    space_point_t point;
};

struct ground_truth_edge_t
{
    node_index_t end;
    double cost;
    trajectory_t* traj;
};

class ground_truth_roadmap_t
{
    private:
        std::unordered_map<node_index_t,ground_truth_vertex_t*> vertices;
        std::unordered_map<node_index_t,double> costs_to_goal;
        std::unordered_map<node_index_t, std::vector<ground_truth_edge_t>> edges;
        node_index_t vertex_counter;
        std::vector<node_index_t> path;
        std::vector<std::vector<node_index_t>> components;

    protected:
        int max_failures, num_failures;
        space_point_t pt;
        std::vector<double> pt_vec;
        double cost;

        std::vector<node_index_t> a_indices, d_indices;
        std::unordered_map<node_index_t, double> a_costs, d_costs;

    public:
        std::vector<space_point_t> verification_set;
        ground_truth_roadmap_t() : max_failures(100), num_failures(0), vertex_counter(0) {}
        ~ground_truth_roadmap_t() {}
    
    void set_max_failures(int max_failures) { this->max_failures = max_failures; }

    space_point_t get_point(node_index_t index) { return vertices[index]->point; }
    
    void run_verification(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        std::vector<unsigned> points_to_remove;

        for (unsigned i = 0; i < verification_set.size(); i++)
        {
            spec.state_space -> copy_point(pt, verification_set[i]);
            get_indices(query, spec, controller);
            if (a_indices.size() > 1 && d_indices.size() > 1)     
                points_to_remove.push_back(i);
        }

        for (int i = points_to_remove.size() - 1; i >= 0; i--)
        {
            verification_set.erase(verification_set.begin() + points_to_remove[i]);
        }
    }
    
    void get_indices(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        a_indices.clear();
        d_indices.clear();

        a_costs.clear();
        d_costs.clear();

        for (auto v : vertices)
        {
            spec.state_space -> copy_point(query.goal_state, v.second -> point);
            spec.state_space -> copy_point(query.start_state, pt);

            controller.fulfill_query(query, spec);

            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
            {
                a_indices.push_back(v.first);
                a_costs[v.first] = query.solution_traj.size();
            }
            
            query.clear_outputs();

            spec.state_space -> copy_point(query.goal_state, pt);
            spec.state_space -> copy_point(query.start_state, v.second -> point);

            controller.fulfill_query(query, spec);

            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
            {
                d_indices.push_back(v.first);
                d_costs[v.first] = query.solution_traj.size();
            }

            query.clear_outputs();
        }
    }

    node_index_t get_best_node(space_point_t s,rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        a_indices.clear();

        for (auto v_idx : path)
        {
            auto v = vertices[v_idx];
            spec.state_space -> copy_point(query.goal_state, v -> point);
            spec.state_space -> copy_point(query.start_state, s);

            controller.fulfill_query(query, spec);

            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
            {
                return v_idx;
                // a_indices.push_back(v_idx);
            }
            
            query.clear_outputs();
        }

        // double min_cost = PRX_INFINITY;
        // node_index_t min_index = -1;

        // for (auto i : a_indices)
        // {
        //     if (costs_to_goal[i] < min_cost)
        //     {
        //         min_cost = costs_to_goal[i];
        //         min_index = i;
        //     }
        // }

        return -1;
    }

    std::pair<node_index_t, node_index_t> find_closest_indices(std::vector<node_index_t> c1, std::vector<node_index_t> c2, rrt_specification_t& spec, double max_dist=10)
    {
        double min_dist = std::numeric_limits<double>::max();
        node_index_t min_index_1 = -1;
        node_index_t min_index_2 = -1;
        for (auto i : c1)
        {
            for (auto j : c2)
            {
                double dist = spec.distance_function(vertices[i] -> point, vertices[j] -> point);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_index_1 = i;
                    min_index_2 = j;
                }
            }
        }
        if (min_dist > max_dist) return std::make_pair(-1, -1);
        return std::make_pair(min_index_1, min_index_2);
    }

    bool check_edge_exists(node_index_t d, node_index_t a)
    {
        if (edges.find(d) == edges.end()) return false;
        for (auto e : edges[d])
        {
            if (e.end == a) return true;
        }
        return false;
    }

    bool check_connected(node_index_t d, node_index_t a)
    {
        // If d and a are connected, DFS on the graph starting from d should contain a.
        std::vector<node_index_t> stack;
        std::unordered_set<node_index_t> visited;

        stack.push_back(d);
        visited.insert(d);

        while (!stack.empty())
        {
            node_index_t curr = stack.back();
            stack.pop_back();

            if (curr == a) return true;

            if (edges.find(curr) != edges.end())
            {
                for (auto e : edges[curr])
                {
                    if (visited.find(e.end) == visited.end())
                    {
                        stack.push_back(e.end);
                        visited.insert(e.end);
                    }
                }
            }
        }
        return false;
    }

    void add_edge(node_index_t s, node_index_t t, double cost)
    {
        prx_assert(s != t, "Cannot add edge between the same node.");
        if (edges.find(s) == edges.end())
        {
            edges[s] = std::vector<ground_truth_edge_t>();
        }
        ground_truth_edge_t e;
        e.end = t;
        e.cost = cost;
        edges[s].push_back(e);
    }
    
    void build_roadmap(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller, bool verify = false)
    {
        pt = spec.state_space -> make_point();
        do
        {
            if (verify)
            {
                run_verification(query, spec, controller);
                std::cout << "Size of verification set: " << verification_set.size() << std::endl;
                if (verification_set.size() == 0)
                {
                    std::cout << "All nodes are verified." << std::endl;
                    break;
                }
            }
            do
            {
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            pt_vec.clear();
            spec.state_space -> copy_vector_from_point(pt_vec,pt);

            get_indices(query,spec,controller);

            bool changed_graph = false;

            if (a_indices.size() == 0 || d_indices.size() == 0)
            {
                auto v = new ground_truth_vertex_t();
                v -> point = spec.state_space -> clone_point(pt);
                vertices.insert(std::make_pair(vertex_counter, v));

                for (auto a : a_indices)
                {
                    // cost = spec.distance_function(vertices[a] -> point, pt);
                    cost = a_costs[a];
                    add_edge(vertex_counter, a, cost);
                }

                for (auto d : d_indices)
                {
                    // cost = spec.distance_function(vertices[d] -> point, pt);
                    cost = d_costs[d];
                    add_edge(d, vertex_counter, cost);
                }

                changed_graph = true;
                vertex_counter++;
            }

            else
            {
                bool vertex_created = false;
                for (auto d : d_indices)
                {
                    for (auto a : a_indices)
                    {
                        if (d == a) continue;
                        // Check if d and a are connected by any path.
                        bool connected = check_connected(d, a);
                        if (!connected)
                        {
                            query.clear_outputs();
                            spec.state_space -> copy_point(query.start_state, vertices[d] -> point);
                            spec.state_space -> copy_point(query.goal_state, pt);

                            controller.fulfill_query(query, spec);

                            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                            {
                                spec.state_space -> copy_point(query.start_state, query.solution_traj.back());
                                spec.state_space -> copy_point(query.goal_state, vertices[a] -> point);
                                trajectory_t buffer_traj(query.solution_traj);
                                query.clear_outputs();

                                controller.fulfill_query(query, spec);

                                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                                {
                                    if (!vertex_created)
                                    {
                                        auto v = new ground_truth_vertex_t();
                                        v -> point = spec.state_space -> clone_point(pt);
                                        vertices.insert(std::make_pair(vertex_counter, v));
                                        vertex_created = true;
                                        vertex_counter++;
                                    }
                                    // cost = spec.distance_function(vertices[d] -> point, pt);
                                    cost = d_costs[d];
                                    add_edge(d, vertex_counter-1, cost);

                                    // cost = spec.distance_function(vertices[a] -> point, pt);
                                    cost = a_costs[a];
                                    add_edge(vertex_counter-1, a, cost);

                                    changed_graph = true;
                                }

                            }
                        }
                    }
                }
            }

            if (!changed_graph)
            {
                num_failures++;
                output_progress_bar(1.0*num_failures/max_failures);
            }

        } while (num_failures < max_failures); 
    }

    void remove_edge(node_index_t s, node_index_t t)
    {
        edges[s].erase(std::remove_if(edges[s].begin(), edges[s].end(), [t](ground_truth_edge_t e) { return e.end == t; }), edges[s].end());
    }

    void remove_vertex(node_index_t v)
    {
        for (auto e : edges[v])
        {
            remove_edge(e.end, v);
        }
        // Locate the vertex in other vertices' edges.
        for (auto e : edges)
        {
            for (auto edge : e.second)
            {
                if (edge.end == v)
                {
                    remove_edge(e.first, v);
                }
            }
        }
        edges.erase(v);
        vertices.erase(v);
    }

    std::string print_vertices(space_t* space)
    {
        std::stringstream out(std::stringstream::out);
        for (auto v : vertices)
        {
            out << v.first << "," << space -> print_point(v.second -> point,4) << std::endl;
        }
        return out.str();
    }

    std::string print_edges()
    {
        std::stringstream out(std::stringstream::out);
        for (auto e : edges)
        {
            for (auto e2 : e.second)
            {
                out << e.first << "," << e2.end << "," << e2.cost << std::endl;
            }
        }
        return out.str();
    }

    void refine_roadmap(rrt_specification_t& spec)
    {
        node_index_t c1_idx, c2_idx;
        do
        {
            c1_idx = uniform_int_random(0, components.size()-1);
            c2_idx = uniform_int_random(0, components.size()-1);
        } while (c1_idx == c2_idx);

        auto c1 = components[c1_idx];
        auto c2 = components[c2_idx];

        auto min_indices = find_closest_indices(c1, c2, spec);
        node_index_t c1_min = min_indices.first;
        node_index_t c2_min = min_indices.second;


        if (c1_min != -1 && c2_min != -1 && !check_edge_exists(c1_min, c2_min))
        {
            std::cout << "Adding dummy edge from " << c1_min << " to " << c2_min << std::endl;

            if (edges.find(c1_min) == edges.end())
            {
                edges[c1_min] = std::vector<ground_truth_edge_t>();
            }

            ground_truth_edge_t e;
            e.end = c2_min;
            e.cost = 10.0 * spec.distance_function(vertices[c1_min] -> point, vertices[c2_min] -> point);

            edges[c1_min].push_back(e);
        }
    }

    node_index_t add_goal(space_point_t g, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, g);
        spec.state_space -> copy_vector_from_point(pt_vec,pt);

        get_indices(query, spec, controller);

        // Add the goal as a vertex.
        auto v = new ground_truth_vertex_t();
        v -> point = spec.state_space -> clone_point(pt);
        vertices.insert(std::make_pair(vertex_counter, v));

        if (d_indices.size() == 0)
        {
            return -1;
        }

        for (auto d : d_indices)
        {
            // cost = spec.distance_function(vertices[d] -> point, pt);
            cost = d_costs[d];
            add_edge(d, vertex_counter, cost);
        }

        vertex_counter++;

        return vertex_counter-1;
    }

    node_index_t add_start(space_point_t s, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, s);
        spec.state_space -> copy_vector_from_point(pt_vec,pt);

        get_indices(query, spec, controller);

        // Add the goal as a vertex.
        auto v = new ground_truth_vertex_t();
        v -> point = spec.state_space -> clone_point(pt);
        vertices.insert(std::make_pair(vertex_counter, v));

        if (a_indices.size() == 0)
        {
            return -1;
        }

        for (auto a : a_indices)
        {
            // cost = spec.distance_function(vertices[a] -> point, pt);
            cost = a_costs[a];
            add_edge(vertex_counter, a, cost);
        }

        vertex_counter++;

        return vertex_counter-1;
    }

    bool is_connected()
    {
        // Check if roadmap is fully connected.
        // Tarjan's algorithm for finding strongly connected components.
        std::vector<node_index_t> indices;
        std::vector<node_index_t> lowlinks;
        std::vector<node_index_t> stack;
        std::vector<bool> on_stack;
        
        components.clear();

        node_index_t index = 0;
        for (auto v : vertices)
        {
            indices.push_back(-1);
            lowlinks.push_back(-1);
            on_stack.push_back(false);
        }

        for (auto v : vertices)
        {
            if (indices[v.first] == -1)
            {
                strongconnect(v.first, index, indices, lowlinks, stack, on_stack, components);
            }
        }

        if (components.size() > 1)
        {
            std::cout << "Roadmap is not fully connected." << std::endl;
            std::cout << "Found " << components.size() << " components." << std::endl;
            for (auto c : components)
            {
                std::cout << "Component: ";
                for (auto v : c)
                {
                    std::cout << v << " ";
                }
                std::cout << std::endl;
            }
            return false;
        }
        else
        {
            std::cout << "Roadmap is fully connected." << std::endl;
            return true;
        }
    }

    void strongconnect(node_index_t v, node_index_t& index, std::vector<node_index_t>& indices, std::vector<node_index_t>& lowlinks, std::vector<node_index_t>& stack, std::vector<bool>& on_stack, std::vector<std::vector<node_index_t>>& components)
    {
        indices[v] = index;
        lowlinks[v] = index;
        index++;
        stack.push_back(v);
        on_stack[v] = true;

        if (edges.find(v) != edges.end())
        {
            for (auto e : edges[v])
            {
                node_index_t w = e.end;
                if (indices[w] == -1)
                {
                    strongconnect(w, index, indices, lowlinks, stack, on_stack, components);
                    lowlinks[v] = std::min(lowlinks[v], lowlinks[w]);
                }
                else if (on_stack[w])
                {
                    lowlinks[v] = std::min(lowlinks[v], indices[w]);
                }
            }
        }

        if (lowlinks[v] == indices[v])
        {
            std::vector<node_index_t> component;
            node_index_t w;
            do
            {
                w = stack.back();
                stack.pop_back();
                on_stack[w] = false;
                component.push_back(w);
            }
            while (w != v);
            components.push_back(component);
        }
    }

    std::vector<node_index_t> get_shortest_path(node_index_t s, node_index_t g)
    {
        // Apply Dijkstra's with a priority queue.
        std::priority_queue<std::pair<double, node_index_t>, std::vector<std::pair<double, node_index_t>>, std::greater<std::pair<double, node_index_t>>> pq;
        std::map<node_index_t, node_index_t> prev;
        std::map<node_index_t, double> dist;
        
        for (auto v : vertices)
        {
            dist[v.first] = std::numeric_limits<double>::infinity();
            costs_to_goal[v.first] = std::numeric_limits<double>::infinity();
            prev[v.first] = -1;
        }

        dist[s] = 0;
        pq.push(std::make_pair(0, s));

        while (!pq.empty())
        {
            auto u = pq.top();
            pq.pop();

            if (u.second == g)
            {
                break;
            }

            if (edges.find(u.second) != edges.end())
            {
                for (auto e : edges[u.second])
                {
                    double alt = dist[u.second] + e.cost;
                    if (alt < dist[e.end])
                    {
                        dist[e.end] = alt;
                        prev[e.end] = u.second;
                        pq.push(std::make_pair(alt, e.end));
                    }
                }
            }
        }

        node_index_t curr = g;
        path.clear();
        while (curr != -1)
        {
            costs_to_goal[curr] = dist[g] - dist[curr];
            path.push_back(curr);
            curr = prev[curr];
        }

        // std::reverse(path.begin(), path.end());

        return path;
    }
};