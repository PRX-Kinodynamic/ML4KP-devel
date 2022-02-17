#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/heuristics/medial_axis.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/controllers/learned_functions.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "../wavefront/wavefront.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"
#include "prx/simulation/plants/treaded_vehicle.hpp"
#include "prx/simulation/plants/plants.hpp"

#include <torch/torch.h>
#include <torch/script.h>

#include <fstream>

using namespace prx;

#ifdef __cpp_lib_filesystem
    #include <boost/filesystem.hpp>
    namespace fs = boost::filesystem;
#else
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace boost;

const double map_resolution = 0.1;
const double max_vel = 0.98;

// const double map_resolution = 0.05;
#define CHECK_VAR_SET(VAR) variables.exists(VAR) ? variables[VAR].as<std::string>() : throw prx_assert_t("CHECK_VAR_SET", __FILE__, __LINE__, (prx_assert_t::stream_t() << "Variable: \"" << VAR <<"\" not found!"))
#define VAR_TO_BOOL_AND_CHECK(VAR)   STR_TO_BOOL  (variables[VAR].as<std::string>())
#define VAR_TO_INT_AND_CHECK(VAR)    STR_TO_INT   (variables[VAR].as<std::string>())
#define VAR_TO_DOUBLE_AND_CHECK(VAR) STR_TO_DOUBLE(variables[VAR].as<std::string>())

auto variables = param_loader("/home/kushal/dirtmp/resources/input_files/planner_evaluations/sac_plus_her.yaml");
std::vector<std::vector<bool>> extracted_map;
std::vector<double> problem;
std::vector<double> lower;
std::vector<double> upper;
system_ptr_t plant = nullptr;
medial_axis_t* medial_axis = nullptr;
std::string param_file = "planner_evaluations/maps_evaluation.yaml";

std::vector<double> extract_problem(const std::string problem_fname, int problem)
{
    std::cout << "problem file" << problem_fname << std::endl;
    std::vector<double> problem_vec;
    std::ifstream ifs(problem_fname);
    std::string line;
    
    for (int i = 0; i <= problem; ++i)
    {
        std::getline(ifs, line);
    }

    std::istringstream ss(line);
    std::string token, num;
    while(std::getline(ss, token, ',')) 
    {
        problem_vec.push_back(stod(token));
    }
    return problem_vec;
}

std::vector<std::vector<bool>> extract_map(const std::string map_fname)
{
    std::cerr << "map_fname = " << map_fname << std::endl;
    std::vector<std::vector<bool>> extracted_map;
    std::ifstream ifs(map_fname);
    std::string line;
    int height, width;
    
    std::getline(ifs,line); // Assuming map type is octile, so skip
    
    std::getline(ifs,line);
    int i = 0;
    for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    height = std::atoi(line.c_str());
    std::cout << "Height: " << height;

    std::getline(ifs,line);    
    for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    width = std::atoi(line.c_str());
    std::cout << " Width: " << width << std::endl;

    std::getline(ifs,line); // This is the line that says "map"

    for (i = 0; i < height; i++)
    {
        std::vector<bool> map_line;
        std::getline(ifs,line);
        for (int j = 0; j < width; j++)
        {
            //if (line[j] == '.') map_line.push_back(true);
            //else map_line.push_back(false);
            map_line.push_back(line[j] == '.');
        }
        extracted_map.push_back(map_line);
    }

    return extracted_map;
}

void init_city_maps_tests(int problem_num)
{
    printf("~~~~~ INITILIZING CITY MAPS TESTS ~~~~~\n");
    init_random(std::stoi(variables["random_seed"].as<std::string>()));
    
    const std::string map_fname = input_path + "maps/" + variables["map_name"].as<std::string>();
    const std::string pro_fname = input_path + "maps/" + variables["map_name"].as<std::string>() + ".problems";
    // problem = extract_problem(pro_fname, std::stoi(variables["problem_num"].as<std::string>()));
    problem = extract_problem(pro_fname, problem_num);
    extracted_map = extract_map(map_fname);
    std::cout << "Extracted\n";
    int height = extracted_map.size();
    int width  = extracted_map[0].size();

    std::cout << "Extracted\n";
    std::string plant_name = "treaded_vehicle";
    plant = system_factory_t::create_system(plant_name, plant_name);
    // plant = system_factory_t::create_system(variables["plant_name"].as<std::string>());
    std::cout << "Extracted\n";
        
    lower = {-width*0.5*map_resolution,-height*0.5*map_resolution, -PRX_PI, -0.7, -0.7};
    upper = {(width*0.5-1)*map_resolution, (height*0.5-1)*map_resolution, PRX_PI, 0.7, 0.7};
    plant->set_state_space_bounds(lower,upper);

    // variables["max_speed"] = std::to_string(system_factory_t::get_system_max_velocity(variables["plant_name"].as<std::string>(), plant));
}

void tree_to_html(three_js_group_t* vis_group, planner_query_t p_query,
        space_t* state_space )
{
    if (STR_TO_BOOL(variables["visualize"].as<std::string>()))
    {
        for(auto& traj : p_query.tree_visualization)
        {
            if(traj != p_query.solution_traj)
                vis_group->add_vis_infos(info_geometry_t::FULL_LINE, traj, "treaded_vehicle/body", state_space);
        }
        if(p_query.solution_traj.size()!=0)
        {
            vis_group->add_vis_infos(info_geometry_t::FULL_LINE, p_query.solution_traj, 
                "treaded_vehicle/body", state_space, "0xFF0000");
        }

        double timestamp=0;
        for(auto state : p_query.solution_traj)
        {
            state_space -> copy_from_point(state);
            vis_group->snapshot_state(timestamp);
            timestamp+=simulation_step;
        }
        if(p_query.solution_traj.size()==0)
        {
            state_space -> copy_from_point(p_query.start_state);
            vis_group->snapshot_state(timestamp);
            timestamp+=simulation_step;
        }

        vis_group->output_html("output.html");

    }
}

void tree_to_txt(planner_query_t p_query, 
    std::string traj_file_name = lib_path + "out/sln.txt", std::string tree_file_name = lib_path + "out/tree.txt")
{
    if(! STR_TO_BOOL(variables["visualize"].as<std::string>())) return;

    int h, w, h_prev, w_prev;
    std::string fn_sg = lib_path + "out/start_goal.txt";
    std::ofstream ofs_sg;
    ofs_sg.open(fn_sg, std::ofstream::out);
    //printf("%s\n", fn_sg.c_str() );
    h = (p_query.start_state->at(1) - lower[1])/map_resolution;
    w = (p_query.start_state->at(0) - lower[0])/map_resolution;
    ofs_sg << w << " " << h << " ";

    h = (p_query.goal_state->at(1) - lower[1])/map_resolution;
    w = (p_query.goal_state->at(0) - lower[0])/map_resolution;
    ofs_sg << w << " " << h << std::endl;
    ofs_sg.close();

    std::string fn_tree =  tree_file_name;
    std::ofstream ofs_tree;
    ofs_tree.open(fn_tree.c_str(), std::ofstream::out);

    bool first = true;
    for (int i = 0; i < p_query.tree_visualization.size(); i++)
    {
        first = true;
        for(auto s : p_query.tree_visualization[i])
        {
            h_prev = h;
            w_prev = w;
            h = (s -> at(1) - lower[1])/map_resolution;
            w = (s -> at(0) - lower[0])/map_resolution;
            if(!first)
            {
                ofs_tree << w_prev << " " << h_prev << " " << w << " " << h << std::endl;
            }
            first = false;
        }
    }
    ofs_tree.close();

    std::string fn_sln = traj_file_name;
    std::ofstream ofs_sln;
    ofs_tree.open(fn_sln.c_str(), std::ofstream::out);
    if(p_query.solution_traj.size()!=0)
    {
        first = true;
        for(auto s : p_query.solution_traj)
        {
            h_prev = h;
            w_prev = w;
            h = (s -> at(1) - lower[1])/map_resolution;
            w = (s -> at(0) - lower[0])/map_resolution;
            if(!first)
            {
                ofs_tree << w_prev << " " << h_prev << " " << w << " " << h << std::endl;
            }
            first = false;
        }

    }
    else
    {
        ofs_tree << "0 0 0 0" << std::endl;
    }
    ofs_sln.close();
}

int main(int argc, char* argv[])
{
    po::options_description desc("Second Order Controller - Full");
    desc.add_options()
            ("help,h", "produce help message")
            ("goal,g", po::value<std::string>()->default_value("upper_right"), "Location of Goal Point")
            ("problem,p", po::value<std::string>()->default_value("0"), "0 for no pruning, 1 will include pruning")
            ("output_dir,o", po::value<std::string>()->default_value("out/"), "Path to Output Directory")
            ("blossom,b", po::value<std::string>()->default_value("1"), "Expansions for Blossom")
            ("type,t", po::value<std::string>()->default_value("0"), "Heuristic To Use")
            ("controller,c", po::value<std::string>()->default_value("0"), "Use Learned controller")
            ("window_size,w", po::value<std::string>()->default_value("20"), "Window")
            ("weight,s", po::value<std::string>()->default_value("0.5"), "Weight")
    ;
    // Read arguments
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
            std::cout << desc << std::endl;
            return 1;
    }
    try
    {
        init_random(101193);
        torch::manual_seed(123456);
        torch::NoGradGuard no_grad;
        simulation_step = 0.1; 
        torch::Device device(torch::kCPU);
        
        torch::jit::script::Module controller;

        if (std::stoi(vm["controller"].as<std::string>()) == 1){
            std::string network_path = lib_path + "resources/models/Second_Order_Drive/Theta_Deterministic.pt";
            // std::string network_path = "/home/kushal/Downloads/Theta_Deterministic.pt";
            try
            {
                controller = torch::jit::load(network_path);
            }
            catch(const c10::Error& e)
            {
                std::cerr << "Error loading the model\n";
                return -1;
            }
        }

        torch::jit::script::Module heuristic_model;

        if (std::stoi(vm["type"].as<std::string>()) == 1){
            // std::string network_path = "/home/kushal/c2g/Second_Order_Theta/torchscript_model.pt";
            std::string network_path = "/home/kushal/dirtmp/resources/models/Second_Order_Drive/Theta_Deterministic.pt";
            try
            {
                heuristic_model = torch::jit::load(network_path);
            }
            catch(const c10::Error& e)
            {
                std::cerr << "Error loading the model\n";
                return -1;
            }
        }

        torch::jit::script::Module lg_model;

        if (std::stoi(vm["controller"].as<std::string>()) == 1){
            // std::string network_path = "/home/kushal/Downloads/local_model_obs.pt";
            // std::string network_path = "/home/kushal/Downloads/local_model_recollect.pt";
            // std::string network_path = "/home/kushal/Downloads/local_model_random.pt";
            // std::string network_path = "/home/kushal/Downloads/local_model_berlin_overfit_rlg.pt";
            std::string network_path = "/home/kushal/Downloads/Denver/mlp_without_obstacles_random.pt";
            try
            {
                lg_model = torch::jit::load(network_path);
            }
            catch(const c10::Error& e)
            {
                std::cerr << "Error loading the model\n";
                return -1;
            }
        }

        std::vector<std::string> obstacle_names;
        std::vector<std::shared_ptr<movable_object_t>> obstacle_list;
        std::vector<double> start_state;
        std::vector<double> goal_state;
 
        int problem_num = std::stoi(vm["problem"].as<std::string>());
        std::cerr << "SET START AND GOAL\n";
        init_city_maps_tests(problem_num);
        std::cerr << "SET START AND GOAL\n";
        start_state = {problem[0], problem[1]};
        goal_state = {problem[2], problem[3]};

        std::string plant_name = "treaded_vehicle";
        // auto plant = system_factory_t::create_system(plant_name, plant_name);
        // prx_assert(plant != nullptr, "Plant is nullptr!");

        world_model_t<> world_model({plant},{obstacle_list});
        world_model.create_context("disk_context",{plant_name},{obstacle_names});

        // std::vector<double> lower = {-10.0,-10.0,-PRX_PI, -0.7, -0.7};
        // std::vector<double> upper = { 10.0, 10.0,PRX_PI, 0.7, 0.7}; 
        // plant->set_state_space_bounds(lower,upper);
        std::vector <double> action_upper = {0.2, 0.2};

        auto context = world_model.get_context("disk_context");
        context.first -> get_state_space() -> set_bounds(lower, upper);

        dirt_c2g_t dirt_c2g("dirt_c2g");
        dirt_c2g_specification_t dirt_c2g_spec(context.first,context.second);   //plant, obstacles

        int min_steps = 5;
        int max_steps = 15;
        dirt_c2g_spec.min_control_steps = min_steps;
        dirt_c2g_spec.max_control_steps = max_steps;

        dirt_c2g_spec.blossom_number=std::stoi(vm["blossom"].as<std::string>());
        int window_size = std::stoi(vm["window_size"].as<std::string>());

        dirt_c2g_spec.valid_state = [&](space_point_t& s)
        {
            int h = (s->at(1) - lower[1])/map_resolution;
            int w = (s->at(0) - lower[0])/map_resolution;
            return extracted_map.at(h).at(w);
        };

        dirt_c2g_spec.valid_check = [&dirt_c2g_spec](trajectory_t& traj)
        {
            for (auto&& s : traj)
            {
                if (!dirt_c2g_spec.valid_state(s)) return false;
            }
            return true;
        };

        // std::cout << vm["prune"].as<std::string>() << std::endl;
        // if(vm["prune"].as<std::string>() =="0")
        dirt_c2g_spec.use_pruning = false;
        // else
        //     dirt_c2g_spec.use_pruning = true;

        dirt_c2g_query_t dirt_c2g_query(context.first->get_state_space(), context.first->get_control_space());
        dirt_c2g_query.start_state = context.first->get_state_space()->make_point();
        dirt_c2g_query.goal_state  = context.first->get_state_space()->make_point();

        dirt_c2g_query.start_state -> at(0) = start_state[0];
        dirt_c2g_query.start_state -> at(1) = start_state[1];

        dirt_c2g_query.goal_state -> at(0) = goal_state[0];
        dirt_c2g_query.goal_state -> at(1) = goal_state[1];

        std::cerr << "SET START AND GOAL\n";

        std::vector <double> goal_vec;
        goal_vec.push_back((dirt_c2g_query.goal_state->at(0)-lower[0])/map_resolution);
        goal_vec.push_back((dirt_c2g_query.goal_state->at(1)-lower[1])/map_resolution);

        mapping_f func = [&](int i, int j, space_point_t p)
        {
            p -> at(0) = ( i * map_resolution + lower[0] ) ;
            p -> at(1) = ( j * map_resolution + lower[1] ) ;
        };

        medial_axis_t* medial_axis = new medial_axis_t();
        medial_axis -> init();
        medial_axis -> set_map((upper[0] - lower[0])/map_resolution, (upper[1] - lower[1])/map_resolution,
            dirt_c2g_spec.valid_state, func, context.first->get_state_space());

        medial_axis -> compute_close_vector_field();
        medial_axis -> set_goal(goal_vec); 
        medial_axis -> find_medial_axis();
        medial_axis -> prepare_graph();
        // medial_axis -> compute_far_vector_field();
        // std::string integrated_file = lib_path + "out/passage_1.txt";
        // medial_axis -> integrated_vf_to_file(integrated_file);

        auto get_local_goal = [&](space_point_t& s, space_point_t& s2){
            double h = (s -> at(1) - lower[1])/map_resolution;
            double w = (s -> at(0) - lower[0])/map_resolution;

            std::complex<double> vp = medial_axis -> integrated_vector_at(w,h,100,2,true);
            std::complex<double> vf = medial_axis -> far_vector_at(w,h,false);

            if (sqrt(vp.real()*vp.real() + vp.imag()*vp.imag()) > PRX_INFINITY)
            {
                if (sqrt(vf.real()*vf.real() + vf.imag()*vf.imag()) < PRX_INFINITY)
                    vp = vf;
                else
                {
                    std::cout << "Vfar was inf too " << std::endl;
                    std::complex<double> default_cmplx(goal_vec[0]-w,goal_vec[1]-h);
                    vp = default_cmplx;
                }
            
            }

            double x1 = s->at(0) + vp.real() * map_resolution;
            double y1 = s->at(1) + vp.imag() * map_resolution;

            h = (y1 - lower[1])/map_resolution;
            w = (x1 - lower[0])/map_resolution;

            vp = medial_axis -> integrated_vector_at(w,h,100,2,true);
            vf = medial_axis -> far_vector_at(w,h,false);

            if (sqrt(vp.real()*vp.real() + vp.imag()*vp.imag()) > PRX_INFINITY)
            {
                if (sqrt(vf.real()*vf.real() + vf.imag()*vf.imag()) < PRX_INFINITY)
                    vp = vf;
                else
                {
                    std::cout << "Vfar was inf too " << std::endl;
                    std::complex<double> default_cmplx(goal_vec[0]-w,goal_vec[1]-h);
                    vp = default_cmplx;
                }
            
            }

            double x2 = x1 + vp.real() * map_resolution;
            double y2 = y1 + vp.imag() * map_resolution;

            space_point_t lg = context.first->get_state_space()->make_point();
            lg -> at(0) = x1;
            lg -> at(1) = y1;
            lg -> at(2) = std::atan2(y2-y1,x2-x1);

            return lg;
        };
        
        if (std::stoi(vm["controller"].as<std::string>()) == 1){
            dirt_c2g_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
            {
                // std::cerr << "Inside Expand\n";
                space_t* state_space = context.first->get_state_space();
                space_t* control_space = context.first->get_control_space();
                trajectory_t traj(state_space);
                plan_t plan(control_space);

                if (blossom_expand)
                {
                    at::Tensor controller_input = torch::zeros({bn,8},device);
                    for (int i = 0; i < bn; i++)
                    {
                        for (int j = 0; j < 5; j++) controller_input[i][j] = s -> at(j);
                        if (i==bn-1){
                            std::pair <double, double> current_position = std::make_pair(s->at(0), s->at(1));
                            space_point_t lg = get_local_goal(s, dirt_c2g_query.goal_state);
                            controller_input[i][5] = lg->at(0);
                            controller_input[i][6] = lg->at(1);
                            controller_input[i][7] = lg->at(2);
                            // std::cout << "current_position = " << state_space->print_point(s, 2) << std::endl;
                            // std::cout << "next_position = " << state_space->print_point(lg, 2) << std::endl;;
                            // std::cin.get();
                        }
                        else{
                            controller_input[i][5] = uniform_random(lower[0], upper[0]);
                            controller_input[i][6] = uniform_random(lower[1], upper[1]);
                            controller_input[i][7] = uniform_random(-PRX_PI, PRX_PI);
                        }   
                    }
                    std::vector<torch::jit::IValue> controller_inputs;
                    controller_inputs.push_back(controller_input);
                    at::Tensor controller_output = controller.forward(controller_inputs).toTensor();
                    for (int i = 0; i < bn; i++)
                    {
                        plan.clear();
                        traj.clear();
                        // You have to change this part.
                        plan.append_onto_back(1.0);
                        for(int j=0; j<2; j++){
                            plan.back().control->at(j) = -action_upper[j] + action_upper[j]*(controller_output[i][j].item<double>() + 1.0);
                        }
                        dirt_c2g_spec.propagate(s,plan,traj);
                        plans.push_back(new plan_t(plan));
                        trajs.push_back(new trajectory_t(traj));
                    }
                }
                else
                {
                    plan.clear();
                    traj.clear();
                    dirt_c2g_spec.sample_plan(plan,s);
                    dirt_c2g_spec.propagate(s,plan,traj);
                    plans.push_back(new plan_t(plan));
                    trajs.push_back(new trajectory_t(traj));
                }
            };
        }

        dirt_c2g_query.goal_region_radius = 0.5;    //epsilon for goal check
        dirt_c2g_query.get_visualization = true;    //for visualization

        dirt_c2g_query.goal_check = [&](space_point_t s)
        {
            std::vector <double> diff = {dirt_c2g_query.goal_state->at(0)-s->at(0),
            dirt_c2g_query.goal_state->at(1)-s->at(1),
            norm_angle_pi(dirt_c2g_query.goal_state->at(2)-s->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum) < dirt_c2g_query.goal_region_radius; 
        };

        dirt_c2g_spec.h = [&](const space_point_t& s, const space_point_t& s2)
        {
            if(vm["type"].as<std::string>() == "0"){
                // std::pair <double, double> position = std::make_pair(s->at(0), s->at(1));
                // std::pair <int, int> cell = wavefront.getCell(position);
                // return wavefront.hMap[cell.first][cell.second];
                return space_t::euclidean_2d(s, s2) / max_vel ;
            }
        };

        dirt_c2g_spec.batch_h = [&](const std::vector<space_point_t> s1, const space_point_t& s)
        {
            if(std::stoi(vm["type"].as<std::string>()) == 0){
                std::vector<double> result;
                for (int idx = 0; idx < s1.size(); idx++)
                {
                    // std::pair <double, double> position = std::make_pair(s1[idx]->at(0), s1[idx]->at(1));
                    // std::pair <int, int> cell = wavefront.getCell(position);
                    // result.push_back(wavefront.hMap[cell.first][cell.second]);
                    result.push_back(space_t::euclidean_2d(s1[idx], s) / max_vel );
                }
                return result;
            }
        };

        dirt_c2g.link_and_setup_spec(&dirt_c2g_spec);
        dirt_c2g.preprocess();
        dirt_c2g.link_and_setup_query(&dirt_c2g_query);

        condition_check_t checker("time", 0.1); 

        std::ofstream fout;
        int stats_runs = 10;        //number of runs
        std::cerr << "HERE\n";
        std::string out_dir = "out/";
        for (int i = 0; i < stats_runs; i++)
        {
            dirt_c2g.link_and_setup_spec(&dirt_c2g_spec);
            dirt_c2g.preprocess();
            dirt_c2g.link_and_setup_query(&dirt_c2g_query);
            planner_statistics_t stats;
            stats.link_planner(&dirt_c2g);
            stats.link_criterion(&checker);
            stats.repeat_data_gathering(100);

            std::string full_filename = vm["output_dir"].as<std::string>()+"/c2g_"+std::to_string(i)+".txt";
            fout.open(full_filename);
            fout<<stats.serialize() << std::endl;
            fout.close();

            dirt_c2g.fulfill_query();
            // tree_to_txt(dirt_c2g_query);
            // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
            // tree_to_html(vis_group, dirt_c2g_query, context.first->get_state_space());
            dirt_c2g_query.clear_outputs();
            dirt_c2g.reset();
        }
    }
    catch(const prx_assert_t& e)
    {
        std::cout << e.get_message() << '\n';
    }
    std::cout << "End of program" << std::endl;
}