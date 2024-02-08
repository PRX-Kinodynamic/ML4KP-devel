#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/planning/strict_reachable_roadmap.hpp"

using namespace prx;

typedef long unsigned int component_index_t;

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


class reachable_roadmap_t
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
        std::unordered_map<component_index_t, std::unordered_set<component_index_t>*> Fw;
        std::unordered_map<component_index_t, std::unordered_set<component_index_t>*> Bw;

    protected:
        int max_failures, num_failures;
        bool collect_reachability;
        std::string reachability_file_path;
        space_point_t pt;
        space_point_t pt2;
        std::vector<double> pt_vec;
        double cost;
        int precision;

        std::vector<node_index_t> a_indices;
        std::vector<node_index_t> d_indices;
        std::unordered_map<node_index_t, double> a_costs;
        std::unordered_map<node_index_t, double> d_costs;

    public:
        reachable_roadmap_t() : max_failures(10), num_failures(0), vertex_counter(0), component_counter(0), collect_reachability(false) {}
        ~reachable_roadmap_t() {
            for(auto c : components) delete c.second;
            for(auto c : Fw) delete c.second;
            for(auto c : Bw) delete c.second;

        }
    
    void set_max_failures(int max_failures) { this->max_failures = max_failures; }
    void set_precision(int precision) { this->precision = precision; }
    void set_collect_reachability(bool collect_reachability, std::string path) { this->collect_reachability = collect_reachability; this->reachability_file_path = path;}

    space_point_t get_point(node_index_t index) { return vertices[index]->point; }

    space_point_t sample_around(space_point_t center, rrt_query_t& query, rrt_specification_t& spec){
        std::vector<double> mutand = std::vector<double>(spec.state_space->get_dimension());
        double radius = query.goal_region_radius;

        do
        {
            spec.state_space->copy_vector_from_point(mutand, center);
            for (double i : mutand){
                i = i + radius*(2.*(rand()/(RAND_MAX + 1.))-1.);
            }
            spec.state_space->copy_point_from_vector(pt2, mutand);

        } while (!spec.valid_state(pt2) || spec.distance_function(center,pt2)>radius);

        return pt2;

    }
    
    std::pair<std::vector<std::pair<node_index_t, node_index_t>>::iterator,std::vector<std::pair<node_index_t, node_index_t>>::iterator> get_all_edges()
    {
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

    void get_indices(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
    {
        a_indices.clear();
        d_indices.clear();
        a_costs.clear();
        d_costs.clear();

        for (auto c : components)
        {
            bool arriveable = false;
            bool departable = false;
            for (auto v : *(c.second))
            {
                
                double a_cost;
                double d_cost;
                space_point_t sample;
                
                if(!arriveable){
                    query.clear_outputs();
                    spec.state_space -> copy_point(query.start_state, vertices[v] -> point);
                    spec.state_space -> copy_point(query.goal_state, pt);
                    controller.fulfill_query(query, spec);

                    if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 1)
                    {
                        double a_cost = (query.solution_traj.size()-1)*simulation_step;
                        bool verified = true;
                        for(int i = 0; i<precision; i++){
                            
                            sample = sample_around(vertices[v] -> point, query, spec);

                            query.clear_outputs();
                            spec.state_space -> copy_point(query.start_state, sample);
                            spec.state_space -> copy_point(query.goal_state, pt);
                            controller.fulfill_query(query, spec);

                            if (!spec.valid_check(query.solution_traj) || query.solution_traj.size() <= 1){
                                verified = false;
                                break;
                            }
                        }
                        if(verified){
                            arriveable = true;
                            a_indices.push_back(v);
                            a_costs[v] = a_cost;
                        }
                        
                    }
                }
                if(!departable){
                    
                    query.clear_outputs();
                    spec.state_space -> copy_point(query.start_state, pt);
                    spec.state_space -> copy_point(query.goal_state, vertices[v] -> point);
                    controller.fulfill_query(query, spec);

                    if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 1)
                    {

                        double d_cost = (query.solution_traj.size()-1)*simulation_step;
                        bool verified = true;
                        for(int i = 0; i<precision; i++){
                            sample = sample_around(vertices[v] -> point, query, spec);

                            query.clear_outputs();
                            spec.state_space -> copy_point(query.start_state, sample);
                            spec.state_space -> copy_point(query.goal_state, pt);
                            controller.fulfill_query(query, spec);

                            if (!spec.valid_check(query.solution_traj) || query.solution_traj.size() <= 1){
                                verified = false;
                                break;
                            }
                        }
                        if(verified){
                            departable = true;
                            d_indices.push_back(v);
                            d_costs[v] = d_cost;
                        }
                        
                    }
                }
                if(arriveable && departable)
                {
                    break;
                }
            
                query.clear_outputs();
            }
        }
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
        prx_assert(s != t, "Cannot add edge between the same node.");
        prx_assert(cost != 0, "Cannot add edge with cost 0.");
        if (edges.find(s) == edges.end())
        {
            edges[s] = std::vector<ground_truth_edge_t*>();
        }
        ground_truth_edge_t* e = new ground_truth_edge_t;
        e->end = t;
        e->cost = cost;
        edges[s].push_back(e);
    }

    void merge_components(component_index_t t, component_index_t s){
        for(auto v : *(components[s]))                
        {
            component_map[v] = t;
        }
        Fw[s]->erase(s);
        Bw[s]->erase(s);
        Fw[t]->erase(s);
        Bw[t]->erase(s);
        for(auto c : *Fw[s]){
            //update bw
            Bw[c]->erase(s);
        }
        for(auto c : *Bw[s]){
            //update fw
            Fw[c]->erase(s);
        }
        Fw[t]->merge(*Fw[s]);
        Bw[t]->merge(*Bw[s]);
        Fw.erase(s);
        Bw.erase(s);
        components[t]->merge(*(components[s]));
        components.erase(s);
    }
    
    void build_roadmap(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller, bool verify = false){       
        std::ofstream outfile;
        outfile.open(reachability_file_path); // append instead of overwrite 
        outfile<<""; 
        outfile.close();

        int num_g = 0;
        int num_c = 0;
        int num_ft = 0;

        pt = spec.state_space -> make_point();
        pt2 = spec.state_space -> make_point();
        do
        {
            do
            {
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            pt_vec.clear();
            spec.state_space -> copy_vector_from_point(pt_vec,pt);

            get_indices(query,spec,controller);

            if (a_indices.size() == 0 || d_indices.size() == 0)  // sample connected to no nodes, becomes a guard
            {
                auto vertex = new ground_truth_vertex_t();
                vertex -> point = spec.state_space -> clone_point(pt);         //add new vertex to map
                vertices.insert(std::make_pair(vertex_counter, vertex));

                std::unordered_set<node_index_t>* component = new std::unordered_set<node_index_t>{vertex_counter};
                std::unordered_set<component_index_t>* forward_set  = new std::unordered_set<component_index_t>{component_counter};
                std::unordered_set<component_index_t>* backward_set = new std::unordered_set<component_index_t>{component_counter};
                components.insert(std::make_pair(component_counter, component));                             //add new component to components
                component_map.insert(std::make_pair(vertex_counter,component_counter));
                Fw.insert(std::make_pair(component_counter, forward_set));
                Bw.insert(std::make_pair(component_counter, backward_set));
                
                vertex_counter++;

                if(collect_reachability) record_visibility(num_failures, spec, query, controller);
                
                component_counter++;
                num_failures = 0;   
            }
            else // sample connected to multiple guards, becomes connection, merges guards
            {
                bool add_node = false;
                auto arrivals = new std::unordered_set<node_index_t>;
                auto departures = new std::unordered_set<node_index_t>;
                auto merges = new std::unordered_set<component_index_t>;

                //std::cout<<"non Guard: a: "<<a_indices.size()<<" d:"<<d_indices.size()<<" = "<< spec.state_space -> print_point(pt) <<std::endl;

                for(auto av : a_indices){
                    for(auto dv : d_indices){
                        component_index_t ac = component_map[av];
                        component_index_t dc = component_map[dv];
                        if(ac == dc){                             // S fully connected to component
                            merges->insert(ac);
                            arrivals->insert(av);
                            departures->insert(dv);
                        }
                        else if(!(Fw[ac]->contains(dc)))
                        {
                            add_node = true;
                            arrivals->insert(av);
                            departures->insert(dv);
                            if(Bw[ac]->contains(dc)){                // loop closure
                                for( auto c : *Bw[ac]){
                                    if(Fw[dc]->contains(c)) merges->insert(c);
                                }
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
                    std::unordered_set<component_index_t>* forward_set  = new std::unordered_set<component_index_t>{component_counter};
                    std::unordered_set<component_index_t>* backward_set = new std::unordered_set<component_index_t>{component_counter};
                    components.insert(std::make_pair(component_counter, component));                             //add new component to components
                    component_map.insert(std::make_pair(vertex_counter,component_counter));
                    Fw.insert(std::make_pair(component_counter, new std::unordered_set<component_index_t>(forward_set->begin(), forward_set->end())));
                    Bw.insert(std::make_pair(component_counter, new std::unordered_set<component_index_t>(backward_set->begin(), backward_set->end())));
                    
                    for (auto a : *arrivals)
                    {
                        cost = a_costs[a];
                        add_edge(a, vertex_counter, cost);

                        backward_set->insert(component_map[a]);
                    }
                    for (auto d : *departures)
                    {
                        cost = d_costs[d];
                        add_edge(vertex_counter, d, cost);
                        forward_set->insert(component_map[d]);
                    }
                    for (auto c : *merges)
                    {
                        //std::cout<<"Merging "<<c<<" into "<<component_counter<<std::endl;
                        forward_set->erase(c);
                        backward_set->erase(c);
                        merge_components(component_counter,c);
                        //print_BwFw();
                    }
                    //print_components();
                    // update forward and backward sets 
                    // this step assumes an acyclic graph, should be satisfied by merge step
                    for (auto A : *backward_set){
                        //std::cout<<"Backward Element A: "<<A<<std::endl;
                        for (auto C : *Bw[A]){
                            //std::cout<<"*BW[A] "<<C<<std::endl;
                            Bw[component_counter]->insert(C);
                        }
                    }
                    for (auto D : *forward_set){
                        //std::cout<<"Forward Element D:"<<D<<std::endl;
                        for (auto C : *Fw[D]){
                            //std::cout<<"*FW[D] "<<C<<std::endl;
                            Fw[component_counter]->insert(C);
                        }
                    }

                    for (auto B : *Bw[component_counter]){
                        //std::cout<<"Backward Element B:"<<B<<std::endl;
                        for (auto C : *Fw[component_counter]){
                            //std::cout<<"FW[B] <-"<<C<<std::endl;
                            Fw[B]->insert(C);
                        }
                    }
                    
                    for (auto F : *Fw[component_counter]){
                        //std::cout<<"Forward Element F:"<<F<<std::endl;
                        for (auto C : *Bw[component_counter]){
                            //std::cout<<"Bw[F] <-"<<C<<std::endl;
                            Bw[F]->insert(C);
                        }
                    }

                    vertex_counter++;
                    
                    if(collect_reachability) record_visibility(num_failures, spec, query, controller);

                    delete forward_set;
                    delete backward_set;
                    component_counter++;
                    num_failures = 0;
                        
                }else{
                    //num_ft++;
                    //std::cout<<"Guards: "<<num_g<<"; connections: "<< num_c <<"; failures total"<< num_ft <<std::endl;
                    num_failures++;
                    output_progress_bar(1.0*num_failures/max_failures);
                }
                delete arrivals;
                delete departures;
                delete merges;
            }

            

        } while (num_failures < max_failures); 
        
    }

    bool test_arriveable(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
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
                return true;
            }
            query.clear_outputs();
        }
        return false;
    }

    bool test_departable(rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller)
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
            spec.state_space -> copy_point(query.start_state, pt);
            spec.state_space -> copy_point(query.goal_state, vertices[v] -> point);
            controller.fulfill_query(query, spec);

            if (spec.valid_check(query.solution_traj) && query.solution_traj.size() > 1)
            {
                return true;
            }
            query.clear_outputs();
        }
        return false;
    }

    void record_visibility(int n_try, rrt_specification_t& spec, rrt_query_t& query, learned_controller_t controller){
        int num_v_samples = 1000;
        double a_count = 0.0;
        double d_count = 0.0;
        
        
        for(int i = 0; i<num_v_samples; i++){
            pt = spec.state_space -> make_point();
            do{
                spec.sample_state(pt);
            } while (!spec.valid_state(pt));

            if(test_arriveable(query, spec, controller)) a_count++;
            if(test_departable(query, spec, controller)) d_count++;
        }

        double arriveability = a_count/num_v_samples;
        double departability = d_count/num_v_samples;
        double p_visibility = 0; 
        if(n_try != 0) p_visibility = 1.0- (1.0/n_try);

        char line[100];
        sprintf(line, "%ld,%f,%f,%f\n", vertex_counter, p_visibility, arriveability, departability);
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
    
    void print_BwFw(){
        std::cout<<"printing BwFw"<<std::endl;
        for (auto c : Bw)
        {
            std::cout << "Bw["<<c.first<<"]: ";
            for (auto v : *(c.second))
            {
                std::cout << v;
                std::cout << " ";
            }
            std::cout<<std::endl;
        }
        for (auto c : Fw)
        {
            std::cout << "Fw["<<c.first<<"]: ";
            for (auto v : *(c.second))
            {
                std::cout << v;
                std::cout << " ";
            }
            std::cout<<std::endl;
        }
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

        if (a_indices.size() == 0)
        {
            return -1;
        }

        for (auto a : a_indices)
        {
            // cost = spec.distance_function(vertices[d] -> point, pt);
            cost = a_costs[a];
            add_edge(a, vertex_counter, cost);
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

        if (d_indices.size() == 0)
        {
            return -1;
        }

        for (auto d : d_indices) // only need to depart from start.
        {
            // cost = spec.distance_function(vertices[a] -> point, pt);
            cost = d_costs[d];
            add_edge(vertex_counter, d, cost);
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
    

    //attempts to execute path with given controller
    //should the path fail, returns a negative value equal to the number of subgoals reached (including start)
    //should the path succeed, return total cost of the path.
    double execute_path(std::vector<node_index_t> path,rrt_query_t& query, rrt_specification_t& spec, learned_controller_t controller){

        space_point_t cur_state = get_point(path.front());
        space_point_t next_goal;
        double path_len = 0.0;
        double goals_reached = -1.0;

        for (int i = 1; i<path.size(); i++) {
            next_goal = get_point(path[i]);
            
            //std::cout<< "Current point: "<<spec.state_space -> print_point(cur_state)<<std::endl;
            //std::cout<< "next subgoal: "<<spec.state_space -> print_point(next_goal)<<std::endl;


            query.clear_outputs();
            spec.state_space -> copy_point(query.start_state, cur_state);
            spec.state_space -> copy_point(query.goal_state, next_goal);
            controller.fulfill_query(query, spec);
            
            if (!spec.valid_check(query.solution_traj) || query.solution_traj.size() <= 1){
                return goals_reached;
            }
            path_len += (query.solution_traj.size()-1)*simulation_step;
            cur_state = query.solution_traj.back();
            goals_reached--;
        }
        
        return path_len;
    }

    double get_edge_len(node_index_t s, node_index_t t)
    {
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