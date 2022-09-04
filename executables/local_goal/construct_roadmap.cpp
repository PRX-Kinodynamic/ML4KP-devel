#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/reachable_region_vertex.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("This executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        params.print();
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            std::vector <double> diff = {a->at(0)-b->at(0),a->at(1)-b->at(1),
            norm_angle_pi(a->at(2)-b->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum);
        };

        learned_controller_t controller(params);

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };

        std::unordered_map<unsigned, reachable_region_vertex_t*> vertices;
        std::unordered_map<unsigned, std::vector<unsigned>> edges;
        unsigned vertex_counter = 0;
        bool verify_edge = params["verify_edge"].as<bool>();

        const int max_failures = 1000;
        int failures = 0;

        space_point_t pt = ss -> make_point();
        std::vector<double> pt_vec;

        do
        {
            // Sample a free state.
            do
            {
                ss -> sample(pt);
            } while (!dirt_spec.valid_state(pt));
            pt_vec.clear();
            ss -> copy_vector_from_point(pt_vec,pt);

            std::vector<unsigned> a_indices, d_indices;

            for (auto v : vertices)
            {
                auto region = v.second;

                if (region -> is_accessible_from(pt_vec))
                    a_indices.push_back(v.first);

                if (region -> can_depart_to(pt_vec))
                    d_indices.push_back(v.first);
            }

            if (a_indices.size() == 0 || d_indices.size() == 0)
            {
                // Create a new region
                auto region = new reachable_region_vertex_t(params);
                bool success = region -> construct_vertex(pt, controller, dirt_query, dirt_spec);
                if (!success) 
                {
                    delete region;
                    failures++;
                    continue;
                }
                vertices.insert(std::make_pair(vertex_counter,region));
                std::cout << "Added new vertex " << vertex_counter << " " << ss -> print_point(pt) << std::endl;
                vertex_counter++;
                continue;
            }
            else
            {
                // Check if any two indices a_index in a_indices and d_index in d_indices are connected.
                bool connected = false;
                for (auto a_index : a_indices)
                {
                    for (auto d_index : d_indices)
                    {
                        if (edges.find(a_index) != edges.end())
                        {
                            if (std::find(edges[a_index].begin(),edges[a_index].end(),d_index) != edges[a_index].end())
                            {
                                connected = true;
                                break;
                            }
                        }
                    }
                    if (connected) break;
                }

                if (!connected)
                {
                    // Pick a_index and d_index from a_indices and d_indices respectively.
                    unsigned a_index = a_indices[uniform_int_random(0,a_indices.size()-1)];
                    unsigned d_index = d_indices[uniform_int_random(0,d_indices.size()-1)];
                    
                    // Verify that an edge is possible.
                    if (verify_edge)
                    {
                        auto d_region_pt = vertices[d_index] -> get_point();
                        auto a_region_pt = vertices[a_index] -> get_point();

                        dirt_query.clear_outputs();
                        ss -> copy_point(dirt_query.start_state,d_region_pt);
                        ss -> copy_point(dirt_query.goal_state,pt);
                        controller.fulfill_query(dirt_query, dirt_spec);

                        if (!dirt_spec.valid_check(dirt_query.solution_traj))
                        {
                            failures++;
                            continue;
                        }

                        ss -> copy_point(dirt_query.start_state,dirt_query.solution_traj.back());
                        ss -> copy_point(dirt_query.goal_state,a_region_pt);
                        dirt_query.clear_outputs();
                        controller.fulfill_query(dirt_query, dirt_spec);

                        if (!dirt_spec.valid_check(dirt_query.solution_traj))
                        {
                            failures++;
                            continue;
                        }
                    }
                    
                    // Create a new region.
                    auto region = new reachable_region_vertex_t(params);
                    bool success = region -> construct_vertex(pt, controller, dirt_query, dirt_spec);
                    if (!success) 
                    {
                        delete region;
                        failures++;
                        continue;
                    }
                    vertices.insert(std::make_pair(vertex_counter,region));
                    std::cout << "Added new vertex " << vertex_counter << " " << ss -> print_point(pt) << std::endl;

                    // Add edge between d_index and vertex_counter.
                    if (edges.find(d_index) == edges.end())
                    {
                        edges.insert(std::make_pair(d_index,std::vector<unsigned>()));
                    }
                    edges[d_index].push_back(vertex_counter);

                    // Add edge between vertex_counter and a_index.
                    if (edges.find(vertex_counter) == edges.end())
                    {
                        edges.insert(std::make_pair(vertex_counter,std::vector<unsigned>()));
                    }
                    edges[vertex_counter].push_back(a_index);

                    std::cout << "Added new edge " << d_index << " " << vertex_counter << std::endl;
                    std::cout << "Added new edge " << vertex_counter << " " << a_index << std::endl;
                    
                    vertex_counter++;
                    
                    continue;
                }
                else
                {
                    // Find closest node in d_indices to pt.
                    double min_dist_d = std::numeric_limits<double>::infinity();
                    unsigned min_index_d = -1;
                    for (auto d_index : d_indices)
                    {
                        auto d_region_pt = vertices[d_index] -> get_point();
                        double dist = dirt_spec.distance_function(d_region_pt,pt);
                        if (dist < min_dist_d)
                        {
                            min_dist_d = dist;
                            min_index_d = d_index;
                        }
                    }

                    // Find closest node in a_indices to pt.
                    double min_dist_a = std::numeric_limits<double>::infinity();
                    unsigned min_index_a = -1;
                    for (auto a_index : a_indices)
                    {
                        auto a_region_pt = vertices[a_index] -> get_point();
                        double dist = dirt_spec.distance_function(a_region_pt,pt);
                        if (dist < min_dist_a && a_index != min_index_d)
                        {
                            min_dist_a = dist;
                            min_index_a = a_index;
                        }
                    }

                    // @aravind: What does this mean?
                    if (min_index_a == -1 || min_index_d == -1)
                    {
                        failures++;
                        continue;
                    }

                    // Check if edge between min_index_d and min_index_a exists.
                    bool edge_exists = false;
                    if (edges.find(min_index_d) != edges.end())
                    {
                        if (std::find(edges[min_index_d].begin(),edges[min_index_d].end(),min_index_a) != edges[min_index_d].end())
                        {
                            edge_exists = true;
                        }
                    }

                    if (edge_exists) 
                    {
                        failures++;
                        continue;
                    }

                    auto d_region_pt = vertices[min_index_d] -> get_point();
                    auto a_region_pt = vertices[min_index_a] -> get_point();

                    dirt_query.clear_outputs();
                    ss -> copy_point(dirt_query.start_state,d_region_pt);
                    ss -> copy_point(dirt_query.goal_state,a_region_pt);
                    controller.fulfill_query(dirt_query, dirt_spec);
                    if (dirt_spec.valid_check(dirt_query.solution_traj))
                    {
                        // Add edge between min_index_d and min_index_a.
                        if (edges.find(min_index_d) == edges.end())
                        {
                            edges.insert(std::make_pair(min_index_d,std::vector<unsigned>()));
                        }
                        edges[min_index_d].push_back(min_index_a);

                        std::cout << "Added new edge " << min_index_d << " " << min_index_a << std::endl;
                    }
                    else
                    {
                        if (verify_edge)
                        {
                            // Attempt to connect via pt.
                            dirt_query.clear_outputs();
                            ss -> copy_point(dirt_query.start_state,d_region_pt);
                            ss -> copy_point(dirt_query.goal_state,pt);
                            controller.fulfill_query(dirt_query, dirt_spec);
                            
                            if (!dirt_spec.valid_check(dirt_query.solution_traj))
                            {
                                failures++;
                                continue;
                            }

                            ss -> copy_point(dirt_query.start_state,dirt_query.solution_traj.back());
                            ss -> copy_point(dirt_query.goal_state,a_region_pt);
                            dirt_query.clear_outputs();
                            controller.fulfill_query(dirt_query, dirt_spec);

                            if (!dirt_spec.valid_check(dirt_query.solution_traj))
                            {
                                failures++;
                                continue;
                            }
                        }

                        // Create a new region.
                        auto region = new reachable_region_vertex_t(params);
                        bool success = region -> construct_vertex(pt, controller, dirt_query, dirt_spec);
                        if (!success) 
                        {
                            delete region;
                            failures++;
                            continue;
                        }
                        vertices.insert(std::make_pair(vertex_counter,region));
                        std::cout << "Added new vertex " << vertex_counter << " " << ss -> print_point(pt) << std::endl;

                        // Add edge between min_index_d and vertex_counter.
                        if (edges.find(min_index_d) == edges.end())
                        {
                            edges.insert(std::make_pair(min_index_d,std::vector<unsigned>()));
                        }
                        edges[min_index_d].push_back(vertex_counter);

                        // Add edge between vertex_counter and min_index_a.
                        if (edges.find(vertex_counter) == edges.end())
                        {
                            edges.insert(std::make_pair(vertex_counter,std::vector<unsigned>()));
                        }
                        edges[vertex_counter].push_back(min_index_a);

                        std::cout << "Added new edge " << min_index_d << " " << vertex_counter << std::endl;
                        std::cout << "Added new edge " << vertex_counter << " " << min_index_a << std::endl;

                        vertex_counter++;

                        continue;
                    }
                    
                }
            }

            failures++;
            
        } while (failures < max_failures);

        std::cout << "Finished constructing the graph." << std::endl;
        
        // Output graph to file.
        std::string vertex_fname = output_path + "vertices.txt";
        std::string edge_fname = output_path + "/edges.txt";

        std::ofstream vertex_file(vertex_fname);
        std::ofstream edge_file(edge_fname);

        for (auto v : vertices)
        {
            auto region = v.second;
            vertex_file << v.first << "," << region -> print_point(ss) << std::endl;

            if (edges.find(v.first) != edges.end())
            {
                for (auto e : edges[v.first])
                {
                    edge_file << v.first << "," << e << std::endl;
                }
            }
        }

        vertex_file.close();
        edge_file.close();
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
#else
int main() {}
#endif