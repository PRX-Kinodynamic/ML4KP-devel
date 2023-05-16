#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/planning/reachable_roadmap.hpp"

using namespace prx;



class strict_reachable_roadmap_t
{
    private:
        std::unordered_map<node_index_t,ground_truth_vertex_t*> vertices;
        std::unordered_map<node_index_t,double> costs_to_goal;
        std::unordered_map<node_index_t, std::vector<ground_truth_edge_t*>> edges;
        std::vector<std::pair<node_index_t, node_index_t>> all_edges;
        node_index_t vertex_counter;
        std::vector<node_index_t> path;
        component_index_t component_counter;
        std::unordered_map<component_index_t, std::unordered_set<node_index_t>*> components;
        std::unordered_map<node_index_t, component_index_t> component_map;

    protected:
        int max_failures, num_failures;
        bool collect_reachability;
        space_point_t pt;
        std::vector<double> pt_vec;
        double cost;
        std::string reachability_file_path;

        std::vector<node_index_t> v_indices;
        std::unordered_set<component_index_t> c_indices;
        std::unordered_map<node_index_t, double> a_costs;
        std::unordered_map<node_index_t, double> d_costs;

    public:
        strict_reachable_roadmap_t() : max_failures(100), num_failures(0), vertex_counter(0), collect_reachability(false) {}
        ~strict_reachable_roadmap_t() {
            for(auto c : components) delete c.second;

        }
    
    void set_max_failures(int max_failures) { this->max_failures = max_failures; }
    void set_collect_reachability(bool collect_reachability, std::string path) { this->collect_reachability = collect_reachability; this->reachability_file_path = path;}

    space_point_t get_point(node_index_t index) { return vertices[index]->point; }
    
    std::pair<std::vector<std::pair<node_index_t, node_index_t>>::iterator,std::vector<std::pair<node_index_t, node_index_t>>::iterator> get_all_edges(){
        // Re-compute all edges.
        all_edges.clear();
        for (auto e : edges)
        {
            for (auto e2 : e.second)
            {
                all_edges.push_back(std::make_pair(e.first, e2->end));
            }
        }
        return std::make_pair(all_edges.begin(), all_edges.end());
    }

    void get_indices(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller){
        c_indices.clear();
        v_indices.clear();

        a_costs.clear();
        d_costs.clear();

        for (auto c : components){
            for (auto v : *(c.second)){
                bool arriveable = false;
                bool departable = false;
                double a_cost;
                double d_cost;
                
                spec.state_space -> copy_point(query.goal_state, vertices[v] -> point);
                spec.state_space -> copy_point(query.start_state, pt);
                controller.fulfill_query(query, spec);

                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0){
                    arriveable = true;
                    a_cost = (query.solution_traj.size()-1)*simulation_step;
                }
                
                query.clear_outputs();
                spec.state_space -> copy_point(query.goal_state, pt);
                spec.state_space -> copy_point(query.start_state, vertices[v] -> point);
                controller.fulfill_query(query, spec);

                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 0){
                    departable = true;
                    d_cost = (query.solution_traj.size()-1)*simulation_step;
                }

                if(arriveable && departable){
                    v_indices.push_back(v);
                    c_indices.insert(c.first);
                    a_costs[v] = a_cost;
                    d_costs[v] = d_cost;
                    query.clear_outputs();
                    break;
                }
            
                query.clear_outputs();
            }
        }
    }

    bool check_edge_exists(node_index_t d, node_index_t a){
        if (edges.find(d) == edges.end()) return false;
        for (auto e : edges[d]){
            if (e->end == a) return true;
        }
        return false;
    }

    bool check_connected(node_index_t d, node_index_t a){
        // If d and a are connected, DFS on the graph starting from d should contain a.
        std::vector<node_index_t> stack;
        std::unordered_set<node_index_t> visited;

        stack.push_back(d);
        visited.insert(d);

        while (!stack.empty()){
            node_index_t curr = stack.back();
            stack.pop_back();

            if (curr == a) return true;

            if (edges.find(curr) != edges.end()){
                for (auto e : edges[curr]){
                    if (visited.find(e->end) == visited.end()){
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
        //std::cout << s << "->"<<t<<":"<<cost << std::endl;
        prx_assert(s != t, "Cannot add edge between the same node.");
        //prx_assert(cost != 0, "Cannot add edge with cost 0.");
        if (edges.find(s) == edges.end())
        {
            edges[s] = std::vector<ground_truth_edge_t*>();
        }
        ground_truth_edge_t* e = new ground_truth_edge_t;
        e->end = t;
        e->cost = cost;
        edges[s].push_back(e);
    }
    
    void build_roadmap(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller, bool verify = false)
    {
        std::ofstream outfile;
        outfile.open(reachability_file_path); // append instead of overwrite 
        outfile<<""; 
        outfile.close();

        pt = spec.state_space -> make_point();
        do
        {
            do
            {
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            pt_vec.clear();
            spec.state_space -> copy_vector_from_point(pt_vec,pt);

            get_indices(query,spec,controller);

            if (v_indices.size() == 0)  // sample connected to no guards, becomes a guard
            {
                auto v = new ground_truth_vertex_t();
                v -> point = spec.state_space -> clone_point(pt);         //add new vertex to map
                vertices.insert(std::make_pair(vertex_counter, v));
                
                std::unordered_set<node_index_t>* component = new std::unordered_set<node_index_t>{vertex_counter};
                components.insert(std::make_pair(component_counter, component));                             //add new component to components
                component_map.insert(std::make_pair(vertex_counter,component_counter));

                if(collect_reachability) record_visibility(num_failures, spec, query, controller);
                vertex_counter++;
                component_counter++;
                num_failures = 0;
            }
            else if(c_indices.size() == 1) // sample connected to exactly one guard, discarded
            {
                num_failures++;
                output_progress_bar(1.0*num_failures/max_failures);
            }
            else                           // sample connected to multiple guards, becomes connection, merges guards
            {
                bool add_node = false;
                for(auto av : v_indices){
                    for(auto dv : v_indices){
                        if(av != dv){
                            if(!check_connected(dv,av)){
                                add_node = true;
                            }
                        }
                    }
                }
                if(add_node)
                {
                    auto vertex = new ground_truth_vertex_t();
                    vertex -> point = spec.state_space -> clone_point(pt);         //add new vertex to map
                    vertices.insert(std::make_pair(vertex_counter, vertex));

                    std::unordered_set<node_index_t>* component = new std::unordered_set<node_index_t>{vertex_counter};
                    components.insert(std::make_pair(component_counter, component));                             //add new component to components
                    component_map.insert(std::make_pair(vertex_counter,component_counter));

                    

                    for (auto v : v_indices)
                    {
                        add_edge(v, vertex_counter, a_costs[v]);

                        add_edge(vertex_counter, v, d_costs[v]);
                    
                        if (component_map[v] != component_counter){
                            auto old_c = component_map[v];
                            for(auto v2 : *(components[old_c]))                 //merge v's component with sample's component
                            {
                                component_map[v2] = component_counter;
                            }
                            components[component_counter]->merge(*(components[old_c]));
                            components.erase(old_c);
                        }
                    }

                    if(collect_reachability) record_visibility(num_failures, spec, query, controller);
                    vertex_counter++;
                    component_counter++;
                    num_failures = 0;
                }
            }
            

        } while (num_failures < max_failures); 
        
    }


    bool test_reachable(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        
        //get largest component
        int max_size = 0;
        component_index_t largest_component;
        for (auto c : components)
        {
            if((c.second)->size() > max_size){
                largest_component = c.first;
                max_size=(c.second)->size();
            }
        }

        for (auto v : *(components[largest_component]))
        {
            query.clear_outputs();
            spec.state_space -> copy_point(query.start_state, vertices[v] -> point);
            spec.state_space -> copy_point(query.goal_state, pt);
            controller.fulfill_query(query, spec);

            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 1)
            {
                query.clear_outputs();
                spec.state_space -> copy_point(query.goal_state, vertices[v] -> point);
                spec.state_space -> copy_point(query.start_state, pt);
                controller.fulfill_query(query, spec);

                if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 1)
                {
                    return true;
                }
            }
            query.clear_outputs();
        }
        return false;
    }

    void record_visibility(int n_try, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller){
        int num_v_samples = 1000;
        double a_count = 0.0;
        
        for(int i = 0; i<num_v_samples; i++){
            pt = spec.state_space -> make_point();
            do{
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            if(test_reachable(query, spec, controller)) a_count++;
        }

        double reachability = a_count/num_v_samples;
        double p_visibility = 0; 
        if(n_try != 0) p_visibility = 1.0- (1.0/n_try);

        char line[100];
        sprintf(line, "%ld,%f,%f\n", vertex_counter, p_visibility, reachability);
        std::ofstream outfile;
        outfile.open(reachability_file_path, std::ios_base::app); // append instead of overwrite 
        outfile<<line; 
        outfile.close();

    }

    void remove_edge(node_index_t s, node_index_t t)
    {
        for (auto e = edges[s].begin(); e != edges[s].end();)
        {
            if ((*e)->end == t)
            {
                delete *e;
                e = edges[s].erase(e);
            }
            else
            {
                ++e;
            }
        }
    }

    void remove_vertex(node_index_t v)
    {
        std::cout << "Removing vertex " << v << std::endl;
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

    std::string print_edge_traj(node_index_t s, node_index_t t,rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller, bool verify = false)
    {
        if (check_edge_exists(s,t) && s != t)
        {
            query.clear_outputs();

            spec.state_space -> copy_point(query.start_state, vertices[s] -> point);
            spec.state_space -> copy_point(query.goal_state, vertices[t] -> point);
            //std::cout<<query.start_state<<std::endl;
            //std::cout<<query.goal_state<<std::endl;

            controller.fulfill_query(query, spec);
            //std::cout<<query.solution_traj.size()<<std::endl;
            if (query.solution_traj.size() == 0)
            {
                std::cout << "empty traj: ";
                std::cout << s << " " << t << std::endl;
                return "";
            }

            return query.solution_traj.print();
        }
        return "";
    }

    void print_components(){
        std::cout << "-- Components --" << std::endl;
        for (auto c : components)
        {
            std::cout << "component: ";
            for (auto v : *(c.second))
            {
                std::cout << v;
                std::cout << " ";
            }
            std::cout<<std::endl;
        }
        std::cout << "--  --" << std::endl;
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

    node_index_t add_goal(space_point_t g, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller)
    {
        spec.state_space -> copy_point(pt, g);
        spec.state_space -> copy_vector_from_point(pt_vec,pt);

        get_indices(query, spec, controller);

        // Add the goal as a vertex.
        auto v = new ground_truth_vertex_t();
        v -> point = spec.state_space -> clone_point(pt);
        vertices.insert(std::make_pair(vertex_counter, v));

        if (v_indices.size() == 0)
        {
            return -1;
        }

        for (auto d : v_indices)
        {
            
            add_edge(d, vertex_counter, d_costs[d]);
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

        if (v_indices.size() == 0)
        {
            return -1;
        }

        for (auto a : v_indices)
        {
            // cost = spec.distance_function(vertices[a] -> point, pt);
            add_edge(vertex_counter, a, a_costs[a]);
        }

        vertex_counter++;

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

        std::reverse(path.begin(), path.end());

        return path;
    }

    double get_edge_len(node_index_t s, node_index_t t){
        if (edges.find(s) == edges.end()){
            prx_throw("invalid edge");
        } 
            
        for (auto e : edges[s])
        {
            if (e->end == t){
                return e->cost;
            }
        }
        prx_throw("invalid edge");
        return 0.0;
    }

    std::string print_path(node_index_t s, node_index_t g, rrt_specification_t& spec, unsigned precision = 3){
        std::vector<node_index_t> path_ids = get_shortest_path(s,g);
        std::stringstream out(std::stringstream::out);

        for(node_index_t id : path_ids){
            out << spec.state_space->print_point(vertices[id]->point, precision) << std::endl;
        }

        return out.str();
        
    }

};