#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/reachable_region_vertex.hpp"

using namespace prx;

struct reachable_region_edge_t
{
    unsigned end;
    double cost;
    bool real;
};

class reachable_region_roadmap_t
{
    private:
    std::unordered_map<node_index_t, reachable_region_vertex_t*> vertices;
    std::unordered_map<node_index_t, double> costs_to_goal;
    std::unordered_map<node_index_t, std::vector<reachable_region_edge_t>> edges;
    node_index_t vertex_counter;

    std::vector<std::vector<node_index_t>> components;

    protected:
    bool verify_edge;
    int max_failures;
    int num_failures;
    std::vector<double> pt_vec;
    space_point_t pt;
    std::shared_ptr<param_loader> param_ptr;
    std::shared_ptr<rrt_query_t> query;
    std::shared_ptr<rrt_specification_t> spec;

    std::vector<node_index_t> a_indices, d_indices;

    public:
    reachable_region_roadmap_t() : max_failures(100), num_failures(0), vertex_counter(0) {}
    ~reachable_region_roadmap_t() {}

    reachable_region_roadmap_t(param_loader params)
    {
        num_failures = 0;
        vertex_counter = 0;
        init(params);
    }

    void init(param_loader params)
    {
        max_failures = params["num_failures"].as<int>();
        verify_edge = params["verify_edge"].as<bool>();

        param_ptr = std::make_shared<param_loader>(params);
    }

    void get_indices()
    {
        a_indices.clear();
        d_indices.clear();
        for (auto v : vertices)
        {
            if (v.second -> is_accessible_from(pt_vec))
                a_indices.push_back(v.first);
            if (v.second -> can_depart_to(pt_vec))
                d_indices.push_back(v.first);
        }
    }

    space_point_t get_point(node_index_t index)
    {
        return vertices[index] -> get_point();
    }

    std::vector<node_index_t> get_accessible_nodes(std::vector<double> p)
    {
        std::vector<node_index_t> accessible;
        for (auto v : vertices)
        {
            if (v.second -> is_accessible_from(p))
                accessible.push_back(v.first);
        }
        return accessible;
    }

    node_index_t get_nearest_accessible_node(space_point_t point, rrt_specification_t& spec)
    {
        double min_dist = std::numeric_limits<double>::max();
        double dist;
        node_index_t nearest = -1;
        pt_vec.clear();
        spec.state_space -> copy_vector_from_point(pt_vec,point);
        auto accessible = get_accessible_nodes(pt_vec);
        for (auto v : accessible)
        {
            auto node = vertices[v];
            dist = spec.distance_function(point, node -> get_point());
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest = v;
            }
        }
        return nearest;
    }

    node_index_t get_lowest_cost_accessible_node(space_point_t point, rrt_specification_t& spec)
    {
        double min_cost = std::numeric_limits<double>::max();
        double cost;
        node_index_t lowest = -1;
        pt_vec.clear();
        spec.state_space -> copy_vector_from_point(pt_vec,point);
        auto accessible = get_accessible_nodes(pt_vec);
        for (auto v : accessible)
        {
            cost = costs_to_goal[v];
            if (cost < min_cost)
            {
                min_cost = cost;
                lowest = v;
            }
        }
        return lowest;
    }

    std::vector<node_index_t> get_departable_nodes(std::vector<double> p)
    {
        std::vector<node_index_t> departable;
        for (auto v : vertices)
        {
            if (v.second -> can_depart_to(p))
                departable.push_back(v.first);
        }
        return departable;
    }

    node_index_t get_nearest_departable_node(space_point_t point, rrt_specification_t& spec)
    {
        double min_dist = std::numeric_limits<double>::max();
        double dist;
        node_index_t nearest = -1;
        pt_vec.clear();
        spec.state_space -> copy_vector_from_point(pt_vec,point);
        auto departable = get_departable_nodes(pt_vec);
        for (auto v : departable)
        {
            auto node = vertices[v];
            dist = spec.distance_function(point, node -> get_point());
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest = v;
            }
        }
        return nearest;
    }

    bool check_connected(const std::vector<node_index_t> d_indices, const std::vector<node_index_t> a_indices)
    {
        // Check if any two indices d in d_indices and a in a_indices are connected.
        for (auto d : d_indices)
        {
            for (auto a : a_indices)
            {
                if (edges.find(d) != edges.end())
                {
                    for (auto e : edges[d])
                    {
                        if (e.end == a)
                            return true;
                    }
                }
            }
        }
        return false;
    }

    bool attempt_add_vertex(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        auto region = new reachable_region_vertex_t(*param_ptr);
        bool success = region -> construct_vertex(pt, controller, query, spec);
        if (!success)
        {
            delete region; 
            return false;
        }
        vertices.insert(std::make_pair(vertex_counter, region));
        std::cout << "Added new vertex: " << vertex_counter << std::endl;
        vertex_counter++;
        return true;
    }

    bool attempt_add_bridge_edge(node_index_t d, node_index_t a, rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        if (d == a) return false;

        std::cout << "Attempting to add bridge edge from " << d << " to " << a << std::endl;
        
        auto d_pt = vertices[d] -> get_point();
        auto a_pt = vertices[a] -> get_point();

        if (verify_edge)
        {
            query.clear_outputs();
            spec.state_space -> copy_point(query.start_state, d_pt);
            spec.state_space -> copy_point(query.goal_state, pt);
            controller.fulfill_query(query, spec);

            if (!spec.valid_check(query.solution_traj))
            {
                std::cout << "Bridge edge from " << d << " to " << a << " failed to verify." << std::endl;
                return false;
            }

            spec.state_space -> copy_point(query.start_state, query.solution_traj.back());
            spec.state_space -> copy_point(query.goal_state, a_pt);
            query.clear_outputs();
            controller.fulfill_query(query, spec);

            if (!spec.valid_check(query.solution_traj))
            {
                std::cout << "Bridge edge from " << d << " to " << a << " failed to verify." << std::endl;
                return false;
            }

        }

        bool success = attempt_add_vertex(query, spec, controller);
        if (!success) return false;

        node_index_t b = vertex_counter - 1;

        reachable_region_edge_t e;
        e.end = b;
        e.cost = 1.0 * spec.distance_function(d_pt, pt);
        e.real = true;

        if (edges.find(d) == edges.end())
        {
            edges.insert(std::make_pair(d, std::vector<reachable_region_edge_t>()));
        }

        edges[d].push_back(e);

        e.end = a;
        e.cost = 1.0 * spec.distance_function(pt, a_pt);
        if (edges.find(b) == edges.end())
        {
            edges.insert(std::make_pair(b, std::vector<reachable_region_edge_t>()));
        }

        edges[b].push_back(e);

        std::cout << "Added bridge edge: " << d << " -> " << vertex_counter - 1 << " -> " << a << std::endl;

        return true;
    }

    bool attempt_add_direct_edge(node_index_t d, node_index_t a, rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        prx_assert(d != a, "Attempting to add direct edge between same vertex.");
        
        auto d_pt = vertices[d] -> get_point();
        auto a_pt = vertices[a] -> get_point();

        query.clear_outputs();
        spec.state_space -> copy_point(query.start_state, d_pt);
        spec.state_space -> copy_point(query.goal_state , a_pt);
        controller.fulfill_query(query, spec);

        if (!spec.valid_check(query.solution_traj))
        {
            std::cout << "Direct edge from " << d << " to " << a << " failed to verify." << std::endl;
            return false;
        }

        reachable_region_edge_t e;
        e.end = a;
        e.cost = 1.0 * spec.distance_function(d_pt, a_pt);
        e.real = true;

        if (edges.find(d) == edges.end())
        {
            edges.insert(std::make_pair(d, std::vector<reachable_region_edge_t>()));
        }

        edges[d].push_back(e);

        std::cout << "Added direct edge: " << d << " -> " << a << std::endl;

        return true;
    }

    node_index_t find_closest_in_set(std::vector<node_index_t> indices, node_index_t found_idx, rrt_specification_t& spec)
    {
        double min_dist = std::numeric_limits<double>::max();
        node_index_t min_index = -1;
        for (auto i : indices)
        {
            double dist = spec.distance_function(vertices[i] -> get_point(), pt);
            if (dist < min_dist && i != found_idx)
            {
                min_dist = dist;
                min_index = i;
            }
        }
        return min_index;
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
                double dist = spec.distance_function(vertices[i] -> get_point(), vertices[j] -> get_point());
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

    bool check_real_edge_exists(node_index_t d, node_index_t a)
    {
        if (edges.find(d) == edges.end()) return false;
        for (auto e : edges[d])
        {
            if (e.end == a && e.real) return true;
        }
        return false;
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

    void build_roadmap(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        pt = spec.state_space -> make_point();
        do
        {
            do
            {
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            pt_vec.clear();
            spec.state_space -> copy_vector_from_point(pt_vec,pt);

            get_indices();

            bool changed_graph = true;

            if (a_indices.size() == 0 || d_indices.size() == 0)
            {
                changed_graph &= attempt_add_vertex(query, spec, controller);
            }

            else
            {
                bool connected = check_connected(d_indices, a_indices);
                if (!connected)
                {
                    unsigned a_index = a_indices[uniform_int_random(0,a_indices.size()-1)];
                    unsigned d_index = d_indices[uniform_int_random(0,d_indices.size()-1)];

                    changed_graph &= attempt_add_bridge_edge(d_index, a_index, query, spec, controller);
                }
                else
                {
                    node_index_t min_d = find_closest_in_set(d_indices, -1, spec);
                    node_index_t min_a = find_closest_in_set(a_indices, min_d, spec);

                    if (min_d != -1 && min_a != -1)
                    {
                        if (check_edge_exists(min_d, min_a))
                        {
                            num_failures++;
                            continue;
                        }
                        bool attempt_direct_edge = attempt_add_direct_edge(min_d, min_a, query, spec, controller);

                        if (!attempt_direct_edge)
                        {
                            changed_graph &= attempt_add_bridge_edge(min_d, min_a, query, spec, controller);
                        }
                    }
                    else
                    {
                        changed_graph = false;
                    }
                }
            }

            if (!changed_graph)
            {
                num_failures++;
            }

        } while (num_failures < max_failures);

        // // Remove all vertices that do not have edges out.
        // std::vector<node_index_t> to_remove;
        // for (auto v : vertices)
        // {
        //     if (edges.find(v.first) == edges.end())
        //     {
        //         to_remove.push_back(v.first);
        //     }
        // }

        // // Remove any vertices that do not have edges coming in.
        // for (auto v : vertices)
        // {
        //     if (edges.find(v.first) != edges.end())
        //     {
        //         for (auto e : edges[v.first])
        //         {
        //             if (std::find(to_remove.begin(), to_remove.end(), e.end) != to_remove.end())
        //             {
        //                 to_remove.push_back(v.first);
        //             }
        //         }
        //     }
        // }

        // for (auto r : to_remove)
        // {
        //     delete vertices[r];
        //     vertices.erase(r);
        //     edges.erase(r);
        // }
    }

    void refine_roadmap(rrt_specification_t& spec)
    {
        // Sample two connected components, find the closest vertices in each, and attempt to add an edge between them.
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
                edges[c1_min] = std::vector<reachable_region_edge_t>();
            }

            reachable_region_edge_t e;
            e.end = c2_min;
            e.cost = 10.0 * spec.distance_function(vertices[c1_min] -> get_point(), vertices[c2_min] -> get_point());
            e.real = false;

            edges[c1_min].push_back(e);
        }
    }

    node_index_t add_goal(space_point_t g, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, g);
        spec.state_space -> copy_vector_from_point(pt_vec,pt);

        get_indices();

        // Add goal to graph.
        bool success = attempt_add_vertex(query, spec, controller);
        if (!success)
        {
            prx_throw("That shouldn't have happened...");
        }

        if (d_indices.size() == 0)
        {
            prx_throw("Deal with this in a bit...");
        }
        else
        {
            for (auto d : d_indices)
            {
                if (edges.find(d) == edges.end())
                {
                    edges[d] = std::vector<reachable_region_edge_t>();
                }

                reachable_region_edge_t e;
                e.end = vertex_counter-1;
                e.cost = 1.0 * spec.distance_function(vertices[d] -> get_point(), vertices[vertex_counter-1] -> get_point());
                e.real = false;

                edges[d].push_back(e);
            }   
        }

        return vertex_counter-1;
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

        std::vector<node_index_t> path;
        node_index_t curr = g;
        while (curr != -1)
        {
            costs_to_goal[curr] = dist[g] - dist[curr];
            path.push_back(curr);
            curr = prev[curr];
        }

        std::reverse(path.begin(), path.end());

        return path;
    }

    std::string print_vertices(space_t* space)
    {
        std::stringstream out(std::stringstream::out);
        for (auto v : vertices)
        {
            out << v.first << "," << v.second -> print_point(space) << "," << costs_to_goal[v.first] << std::endl;
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
                out << e.first << "," << e2.end << "," << e2.real << "," << e2.cost << std::endl;
            }
        }
        return out.str();
    }

    void get_validation_accuracy()
    {
        // Get the depart and access val accuracy, averaged across vertices.
        double d_acc = 0.0;
        double a_acc = 0.0;

        for (auto v : vertices)
        {
            d_acc += v.second -> get_depart_validation_accuracy();
            a_acc += v.second -> get_access_validation_accuracy();
        }

        d_acc /= vertices.size();
        a_acc /= vertices.size();

        std::cout << "Depart validation accuracy: " << d_acc << std::endl;
        std::cout << "Access validation accuracy: " << a_acc << std::endl;
   }

};