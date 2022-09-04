#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/termination_classifier.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

using namespace prx;

class reachable_region_vertex_t
{
    private:
    space_point_t point;
    termination_classifier_t access_classifier, depart_classifier;

    protected:
    unsigned num_trajectories;
    bool sample_around;

    public:
    reachable_region_vertex_t() {}
    ~reachable_region_vertex_t() {}

    reachable_region_vertex_t(param_loader params)
    {
        init(params);
    }

    void init(param_loader params)
    {
        num_trajectories = params["num_trajectories"].as<unsigned>();
        sample_around = params["sample_around"].as<bool>();
    
        access_classifier.init(params);
        depart_classifier.init(params);
    }

    std::string print_point(space_t* space)
    {
        return space -> print_point(point);
    }

    space_point_t get_point()
    {
        return point;
    }

    bool construct_vertex(space_point_t point, learned_controller_t controller, rrt_query_t planner_query, rrt_specification_t planner_spec)
    {
        this->point = planner_spec.state_space -> clone_point(point);

        std::cout << "Constructing vertex for: " << point << std::endl;

        space_point_t current = planner_spec.state_space -> make_point();
        std::vector<double> row;

        std::vector<std::vector<double>> access_data, depart_data;
        std::vector<double> access_labels, depart_labels;

        planner_spec.state_space -> copy_point(planner_query.start_state,point);
        for (int i = 0; i < num_trajectories; i++)
        {
            planner_query.clear_outputs();
            do 
            {
                planner_spec.state_space -> sample(planner_query.goal_state);
            } while (!planner_spec.valid_state(planner_query.goal_state) &&
            !(sample_around && planner_spec.distance_function(planner_query.start_state,planner_query.goal_state) > 5.0));

            controller.fulfill_query(planner_query, planner_spec);

            planner_spec.state_space -> copy_point(current,planner_query.solution_traj.back());
            if (!planner_query.goal_check(current))
            {
                for (unsigned j = 0; j < planner_query.solution_traj.size(); j += controller.get_control_duration()/simulation_step)
                {
                    planner_spec.state_space -> copy_point(current,planner_query.solution_traj[j]);
                    row.clear();
                    planner_spec.state_space -> copy_vector_from_point(row, current);
                    access_data.push_back(row);
                    access_labels.push_back(0);
                }
                continue;
            }

            for (unsigned j = 0; j < planner_query.solution_traj.size(); j += controller.get_control_duration()/simulation_step)
            {
                planner_spec.state_space -> copy_point(current,planner_query.solution_traj[j]);
                
                if (planner_spec.valid_state(current))
                {
                    row.clear();
                    planner_spec.state_space -> copy_vector_from_point(row, current);
                    depart_data.push_back(row);
                    depart_labels.push_back(1);
                }
                else
                {
                    for (unsigned k = j; k < planner_query.solution_traj.size(); k += controller.get_control_duration()/simulation_step)
                    {
                        planner_spec.state_space -> copy_point(current,planner_query.solution_traj[k]);
                        row.clear();
                        planner_spec.state_space -> copy_vector_from_point(row, current);
                        depart_data.push_back(row);
                        depart_labels.push_back(0);
                    }
                    break;
                }
            }
        }

        // Check if all elements of depart_labels are the same.
        bool all_same_depart = true;
        for (unsigned i = 1; i < depart_labels.size(); i++)
        {
            if (depart_labels[i] != depart_labels[0])
            {
                all_same_depart = false;
                break;
            }
        }

        depart_classifier.train(depart_data, depart_labels);

        planner_spec.state_space -> copy_point(planner_query.goal_state,point);
        for (int i = 0; i < num_trajectories; i++)
        {
            planner_query.clear_outputs();
            do
            {
                planner_spec.state_space -> sample(planner_query.start_state);
            } while (!planner_spec.valid_state(planner_query.start_state) && 
            !(sample_around && planner_spec.distance_function(planner_query.start_state,planner_query.goal_state) > 5.0));

            controller.fulfill_query(planner_query, planner_spec);

            planner_spec.state_space -> copy_point(current,planner_query.solution_traj.back());
            if (!planner_query.goal_check(current))
            {
                for (unsigned j = 0; j < planner_query.solution_traj.size(); j += controller.get_control_duration()/simulation_step)
                {
                    planner_spec.state_space -> copy_point(current,planner_query.solution_traj[j]);
                    row.clear();
                    planner_spec.state_space -> copy_vector_from_point(row, current);
                    access_data.push_back(row);
                    access_labels.push_back(0);
                }
                continue;
            }

            for (unsigned j = planner_query.solution_traj.size() - 1; j >= 0; j -= controller.get_control_duration()/simulation_step)
            {
                planner_spec.state_space -> copy_point(current,planner_query.solution_traj[j]);
                
                if (planner_spec.valid_state(current))
                {
                    row.clear();
                    planner_spec.state_space -> copy_vector_from_point(row, current);
                    access_data.push_back(row);
                    access_labels.push_back(1);
                }
                else
                {
                    for (unsigned k = j; k >= 0; k -= controller.get_control_duration()/simulation_step)
                    {
                        planner_spec.state_space -> copy_point(current,planner_query.solution_traj[k]);
                        row.clear();
                        planner_spec.state_space -> copy_vector_from_point(row, current);
                        access_data.push_back(row);
                        access_labels.push_back(0);
                        if (k == 0) break;
                    }
                    break;
                }
                if (j == 0) break;
            }
        } 

        // Check if all elements of access_labels are the same.
        bool all_same_access = true;
        for (unsigned i = 1; i < access_labels.size(); i++)
        {
            if (access_labels[i] != access_labels[0])
            {
                all_same_access = false;
                break;
            }
        }

        access_classifier.train(access_data, access_labels);

        if (all_same_depart || all_same_access) return false;

        return true;
    }

    bool is_accessible_from(std::vector<double> point)
    {
        return access_classifier.predict(point);
    }

    bool can_depart_to(std::vector<double> point)
    {
        return depart_classifier.predict(point);
    }
};
