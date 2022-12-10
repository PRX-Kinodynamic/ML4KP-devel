#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

using namespace prx;

struct landmark_vertex_t
{
    space_point_t point;
};

struct landmark_edge_t
{
    node_index_t end;
    double cost;
    trajectory_t* traj;
};

class landmark_roadmap_t
{
    private:
        std::unordered_map<node_index_t,landmark_vertex_t*> vertices;
        std::unordered_map<node_index_t,double> costs_to_goal;
        std::unordered_map<node_index_t, std::vector<landmark_edge_t*>> edges;
        std::unordered_map<node_index_t, std::unordered_map<node_index_t, double>> distances;
        node_index_t vertex_counter, edge_counter;
        std::vector<node_index_t> path;
        std::vector<std::vector<node_index_t>> components;

    protected:
        space_point_t pt;
        std::vector<double> pt_vec;
        double cost;
        double stretch_factor;

        std::vector<node_index_t> a_indices, d_indices;
        std::unordered_map<node_index_t, double> a_costs, d_costs;

    public:
        std::vector<space_point_t> verification_set;
        landmark_roadmap_t() : vertex_counter(0), edge_counter(0), stretch_factor(3.0) {}
        ~landmark_roadmap_t() {}
    
    space_point_t get_point(node_index_t index) { return vertices[index]->point; }
    
    void set_stretch_factor(double factor) { stretch_factor = factor; }
    
    std::vector<std::pair<node_index_t, node_index_t>> get_all_edges()
    {
        std::vector<std::pair<node_index_t, node_index_t>> all_edges;
        for(auto& v : vertices)
        {
            for(auto& e : edges[v.first])
            {
                all_edges.push_back(std::make_pair(v.first, e->end));
            }
        }
        return all_edges;
    }
    
    void get_indices(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        a_indices.clear();
        d_indices.clear();

        a_costs.clear();
        d_costs.clear();

        for (auto v : vertices)
        {
            // a_indices -> all vertices that can be reached from the considered point
            spec.state_space -> copy_point(query.start_state, pt);
            spec.state_space -> copy_point(query.goal_state, v.second -> point);

            if (!query.goal_check(query.start_state))
            {
                controller.fulfill_query(query, spec);

                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                {
                    a_indices.push_back(v.first);
                    a_costs[v.first] = (query.solution_traj.size()-1)*simulation_step;
                }
            }
            
            query.clear_outputs();

            // d_indices -> all vertices that can reach the considered point
            spec.state_space -> copy_point(query.start_state, v.second -> point);
            spec.state_space -> copy_point(query.goal_state, pt);

            if (!query.goal_check(query.start_state))
            {
                controller.fulfill_query(query, spec);

                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                {
                    d_indices.push_back(v.first);
                    d_costs[v.first] = (query.solution_traj.size()-1)*simulation_step;
                }
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

    node_index_t get_nearest_reachable_node(space_point_t s,rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, s);

        get_indices(query, spec, controller);

        double min_dist = PRX_INFINITY;
        node_index_t min_index = -1;

        for (auto i : a_indices)
        {
            double dist = spec.distance_function(vertices[i] -> point, s);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }

        return min_index;
    }

    bool check_edge_exists(node_index_t d, node_index_t a)
    {
        if (edges.find(d) == edges.end()) return false;
        for (auto e : edges[d])
        {
            if (e->end == a) return true;
        }
        return false;
    }

    double get_edge_cost(node_index_t d, node_index_t a)
    {
        for (auto e : edges[d])
        {
            if (e->end == a) return e->cost;
        }
        return PRX_INFINITY;
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
                    if (visited.find(e->end) == visited.end())
                    {
                        stack.push_back(e->end);
                        visited.insert(e->end);
                    }
                }
            }
        }
        return false;
    }

    void add_edge(node_index_t s, node_index_t t, double cost)
    {
        prx_assert(s != t, "Cannot add edge between the same node->");
        if (edges.find(s) == edges.end())
        {
            edges[s] = std::vector<landmark_edge_t*>();
        }
        landmark_edge_t* e = new landmark_edge_t();
        e->end = t;
        e->cost = cost;
        edges[s].push_back(e);
        edge_counter++;
    }

    double get_path_cost(node_index_t s, node_index_t t)
    {
        std::priority_queue<std::pair<double, node_index_t>, std::vector<std::pair<double, node_index_t>>, std::greater<std::pair<double, node_index_t>>> pq;
        std::map<node_index_t, node_index_t> prev;
        std::map<node_index_t, double> dist;

        for (auto v : vertices)
        {
            dist[v.first] = PRX_INFINITY;
            prev[v.first] = -1;
        }

        dist[s] = 0;
        pq.push(std::make_pair(0, s));

        while (!pq.empty())
        {
            auto curr = pq.top();
            pq.pop();

            if (curr.second == t) break;

            if (edges.find(curr.second) != edges.end())
            {
                for (auto e : edges[curr.second])
                {
                    double alt = dist[curr.second] + e->cost;
                    if (alt < dist[e->end])
                    {
                        dist[e->end] = alt;
                        prev[e->end] = curr.second;
                        pq.push(std::make_pair(alt, e->end));
                    }
                }
            }
        }

        return dist[t];
    }
    
    void build_roadmap(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        pt = spec.state_space -> make_point();
        std::vector<node_index_t> unconsidered;
        for (int i = 0; i < verification_set.size(); i++) unconsidered.push_back(i);

        do
        {
            // Sample a random point from the verification set.
            int idx = uniform_int_random(0, unconsidered.size() - 1);
            node_index_t v_idx = unconsidered[idx];
            unconsidered.erase(unconsidered.begin() + idx);

            spec.state_space -> copy_point(pt, verification_set[v_idx]);

            get_indices(query,spec,controller);

            if (a_indices.size() == 0 || d_indices.size() == 0)
            {
                auto v = new landmark_vertex_t();
                v -> point = spec.state_space -> clone_point(pt);
                vertices.insert(std::make_pair(vertex_counter, v));

                for (auto a : a_indices)
                {
                    cost = a_costs[a];
                    if (!check_edge_exists(vertex_counter,a))
                    {
                        add_edge(vertex_counter, a, cost);
                    }
                }

                for (auto d : d_indices)
                {
                    cost = d_costs[d];
                    if (!check_edge_exists(d, vertex_counter))
                    {
                        add_edge(d, vertex_counter, cost);
                    }
                }

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
                        bool connected = check_connected(d, a);
                        if (!connected || (connected && get_path_cost(d, a) > stretch_factor * (a_costs[a] + d_costs[d])))
                        {
                            // if (connected)
                            // {
                            //     std::cout << "Original path cost: " << get_path_cost(d, a) << std::endl;
                            // }
                            query.clear_outputs();
                            spec.state_space -> copy_point(query.start_state, vertices[d] -> point);
                            spec.state_space -> copy_point(query.goal_state, pt);

                            bool add_flag = true;
                            
                            add_flag &= !query.goal_check(query.start_state);

                            controller.fulfill_query(query, spec);

                            if (add_flag && spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                            {
                                spec.state_space -> copy_point(query.start_state, query.solution_traj.back());
                                spec.state_space -> copy_point(query.goal_state, vertices[a] -> point);
                                trajectory_t buffer_traj(query.solution_traj);
                                query.clear_outputs();

                                add_flag &= !query.goal_check(query.start_state);

                                controller.fulfill_query(query, spec);

                                if (add_flag && spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0)
                                {
                                    if (!vertex_created)
                                    {
                                        auto v = new landmark_vertex_t();
                                        v -> point = spec.state_space -> clone_point(pt);
                                        vertices.insert(std::make_pair(vertex_counter, v));
                                        vertex_created = true;
                                        vertex_counter++;
                                    }
                                    // cost = spec.distance_function(vertices[d] -> point, pt);
                                    cost = d_costs[d];
                                    if (!check_edge_exists(d, vertex_counter - 1))
                                    {
                                        add_edge(d, vertex_counter - 1, cost);
                                    }

                                    // cost = spec.distance_function(vertices[a] -> point, pt);
                                    cost = a_costs[a];
                                    if (!check_edge_exists(vertex_counter - 1, a))
                                    {
                                        add_edge(vertex_counter - 1, a, cost);
                                    }
                                }

                                // if (connected && add_flag)
                                // {
                                //     std::cout << "New path cost: " << get_path_cost(d,a) << std::endl;
                                // }

                            }
                        }
                    }
                }
            }
            if (unconsidered.size() % 10 == 0)
            {
                std::cout << unconsidered.size() << " configurations remaining." << std::endl;
                std::cout << "Vertices: " << vertices.size() << std::endl;
                std::cout << "Edges: " << edge_counter << std::endl;
            }
        } while (unconsidered.size() > 0);

        /*
        std::vector<std::pair<node_index_t, node_index_t>> edges_to_remove;
        for (auto a : vertices)
        {
            for (auto b : vertices)
            {
                if (!check_edge_exists(a.first, b.first)) continue;
                // Get all nodes that have a as a child and b as a parent.
                bool to_remove = true;
                std::vector<node_index_t> in_a;
                std::vector<node_index_t> out_b;
                for (auto e : edges[b.first])
                {
                    out_b.push_back(e->end);
                }

                for (auto e : edges)
                {
                    for (auto f : e.second)
                    {
                        if (f->end == a.first)
                        {
                            in_a.push_back(e.first);
                        }
                    }
                }

                if (in_a.size() == 0 || out_b.size() == 0) continue;

                double a_b_cost = get_edge_cost(a.first, b.first);
                // Temporarily set the edge cost to inf.
                for (auto e : edges[a.first])
                {
                    if (e->end == b.first)
                    {
                        e->cost = PRX_INFINITY;
                        break;
                    }
                }

                // Check path cost from a to b.
                double path_cost_without = get_path_cost(a.first, b.first);
                if (path_cost_without < stretch_factor * a_b_cost)
                {
                    to_remove = false;
                }

                // Reset the edge cost.
                for (auto e : edges[a.first])
                {
                    if (e->end == b.first)
                    {
                        e->cost = a_b_cost;
                        break;
                    }
                }
                if (to_remove)
                {
                    std::cout << "Removing edge " << a.first << " -> " << b.first << std::endl;
                    edges_to_remove.push_back(std::make_pair(a.first, b.first));
                }

            }
        }

        for (auto e : edges_to_remove)
        {
            remove_edge(e.first, e.second);
        }
        */

        for (auto v : vertices)
        {
            // Check if the vertex only has incoming edges.
            bool only_incoming = true;
            for (auto e : edges)
            {
                for (auto ee : e.second)
                {
                    if (ee->end == v.first)
                    {
                        only_incoming = false;
                        break;
                    }
                }
                if (!only_incoming) break;
            }

            if (only_incoming) remove_vertex(v.first);
        }

        for (auto v : vertices)
        {
            // Check if the vertex only has outgoing edges.
            bool only_outgoing = true;
            for (auto e : edges[v.first])
            {
                only_outgoing = false;
                break;
            }

            if (only_outgoing) remove_vertex(v.first);
        }

    }

    void remove_edge(node_index_t s, node_index_t t)
    {
        for (auto e = edges[s].begin(); e != edges[s].end();)
        {
            if ((*e)->end == t)
            {
                delete *e;
                e = edges[s].erase(e);
                edge_counter--;
                return;
            }
            else
            {
                ++e;
            }
        }
    }

    void remove_vertex(node_index_t v)
    {
        // std::cout << "Removing vertex " << v << std::endl;
        for (auto e : edges[v])
        {
            remove_edge(e->end, v);
        }
        // Locate the vertex in other vertices' edges.
        for (auto e : edges)
        {
            for (auto edge : e.second)
            {
                if (edge->end == v)
                {
                    remove_edge(e.first, v);
                }
            }
        }
        edges.erase(v);
        // Free the memory.
        delete vertices[v];
        vertices.erase(v);
    }

    void get_all_pairs_shortest_paths()
    {
        // Initialize the distance matrix.
        for (auto v : vertices)
        {
            for (auto vv : vertices)
            {
                if (v.first == vv.first)
                {
                    distances[v.first][vv.first] = 0;
                }
                else
                {
                    distances[v.first][vv.first] = std::numeric_limits<double>::infinity();
                }
            }    
        }
        
        // Apply Floyd-Warshall algorithm.
        for (auto v : vertices)
        {
            for (auto e : edges[v.first])
            {
                distances[v.first][e->end] = e->cost;
            }
        }

        for (auto k : vertices)
        {
            for (auto i : vertices)
            {
                for (auto j : vertices)
                {
                    if (distances[i.first][k.first] + distances[k.first][j.first] < distances[i.first][j.first])
                    {
                        distances[i.first][j.first] = distances[i.first][k.first] + distances[k.first][j.first];
                    }
                }
            }
        }

        // Print the distance matrix.
        // for (auto v : vertices)
        // {
        //     for (auto vv : vertices)
        //     {
        //         std::cout << distances[v.first][vv.first] << " ";
        //     }
        //     std::cout << std::endl;
        // }
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
                out << e.first << "," << e2->end << "," << e2->cost << std::endl;
            }
        }
        return out.str();
    }

    std::string print_edge_traj(node_index_t s, node_index_t t,rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller, bool verify = false)
    {
        if (check_edge_exists(s,t) && s != t)
        {
            query.clear_outputs();

            spec.state_space -> copy_point(query.start_state, vertices[s] -> point);
            spec.state_space -> copy_point(query.goal_state, vertices[t] -> point);

            controller.fulfill_query(query, spec);

            // if (!spec.valid_check(query.solution_traj))
            // {
            //     std::cout << "Invalid trajectory was added!" << std::endl;
            //     std::cout << s << " " << t << std::endl;
            //     return "";
            // }

            return query.solution_traj.print();
        }
        return "";
    }

    node_index_t add_goal(space_point_t g, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, g);
        spec.state_space -> copy_vector_from_point(pt_vec,pt);

        get_indices(query, spec, controller);

        // Add the goal as a vertex.
        auto v = new landmark_vertex_t();
        v -> point = spec.state_space -> clone_point(pt);
        vertices.insert(std::make_pair(vertex_counter, v));

        if (a_indices.size() == 0)
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
        auto v = new landmark_vertex_t();
        v -> point = spec.state_space -> clone_point(pt);
        vertices.insert(std::make_pair(vertex_counter, v));

        if (d_indices.size() == 0)
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
                node_index_t w = e->end;
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
        // Apply Dijkstra's with a priority queue->
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
                    double alt = dist[u.second] + e->cost;
                    if (alt < dist[e->end])
                    {
                        dist[e->end] = alt;
                        prev[e->end] = u.second;
                        pq.push(std::make_pair(alt, e->end));
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