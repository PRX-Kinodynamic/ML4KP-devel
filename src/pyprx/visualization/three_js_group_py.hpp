#include <iostream>
#include <boost/python.hpp>
#include "prx/visualization/three_js_group.hpp"


void (prx::three_js_group_t::*update_vis_infos_0_0)(prx::info_geometry_t, std::vector<prx::trajectory_t>&, std::string, prx::space_t*, std::string) = &prx::three_js_group_t::add_vis_infos;
void (prx::three_js_group_t::*update_vis_infos_1_0)(prx::info_geometry_t, const prx::trajectory_t&, std::string, prx::space_t*, std::string color)  = &prx::three_js_group_t::add_vis_infos;
void (prx::three_js_group_t::*update_vis_infos_2_0)(prx::info_geometry_t, std::vector<prx::vector_t>, std::string, double)                          = &prx::three_js_group_t::add_vis_infos;
// void (prx::three_js_group_t::*update_vis_infos_2_2)(prx::info_geometry_t, std::vector<prx::vector_t>, std::string)                                  = &prx::three_js_group_t::add_vis_infos;
// void (prx::three_js_group_t::*update_vis_infos_2_3)(prx::info_geometry_t, std::vector<prx::vector_t>)                                               = &prx::three_js_group_t::add_vis_infos;
void (prx::three_js_group_t::*add_detailed_vis_infos_0)(prx::info_geometry_t, const prx::trajectory_t&, std::string, prx::space_t*, std::string)    = &prx::three_js_group_t::add_detailed_vis_infos;

struct three_js_group_wrap : prx::three_js_group_t, wrapper<prx::three_js_group_t>
{
    three_js_group_wrap(const std::vector<prx::system_ptr_t>& in_plants) 
        : three_js_group_t(in_plants){};
    three_js_group_wrap(const std::vector<prx::system_ptr_t>& in_plants,const std::vector<std::shared_ptr<prx::movable_object_t>>& in_obstacles)
        : three_js_group_t(in_plants, in_obstacles){};


    void update_vis_infos_0_1(prx::info_geometry_t info_type, std::vector<prx::trajectory_t>& tree_vis, std::string body_name, prx::space_t* state_space)
    {
        add_vis_infos(info_type, tree_vis, body_name, state_space);
    }
    void update_vis_infos_1_1(prx::info_geometry_t info_type, const prx::trajectory_t& traj, std::string body_name, prx::space_t* state_space)
    {
        add_vis_infos(info_type, traj, body_name, state_space);
    }
    void update_vis_infos_2_1(prx::info_geometry_t info_type, std::vector<prx::vector_t> positions, std::string color)
    {
        add_vis_infos(info_type, positions, color);
    }
    void update_vis_infos_2_2(prx::info_geometry_t info_type, std::vector<prx::vector_t> positions)
    {
        add_vis_infos(info_type, positions);
    }   
    void add_detailed_vis_infos_1(prx::info_geometry_t info_type, const prx::trajectory_t& traj, std::string body_name, prx::space_t* state_space)
    {
        add_detailed_vis_infos(info_type, traj, body_name, state_space);
    }


};


void pyprx_visualization_three_js_group_py()
{

	enum_<prx::info_geometry_t>("info_geometry")
        .value("LINE", prx::info_geometry_t::LINE)
        .value("QUAD", prx::info_geometry_t::QUAD)
        .value("FULL_LINE", prx::info_geometry_t::FULL_LINE)
        .value("CIRCLE", prx::info_geometry_t::CIRCLE)
        .export_values()
        ;

   	class_<prx::three_js_group_t>("three_js_group", init<std::vector<prx::system_ptr_t>>())
        .def(init<std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>())
        .def("add_vis_infos", update_vis_infos_0_0)
        .def("add_vis_infos", &three_js_group_wrap::update_vis_infos_0_1)
        .def("add_vis_infos", update_vis_infos_1_0)
        .def("add_vis_infos", &three_js_group_wrap::update_vis_infos_1_1)
        .def("add_vis_infos", update_vis_infos_2_0)
        .def("add_vis_infos", &three_js_group_wrap::update_vis_infos_2_1)
        .def("add_vis_infos", &three_js_group_wrap::update_vis_infos_2_2)
        // .def("add_vis_infos", add_detailed_vis_infos_0)
        // .def("add_vis_infos", update_vis_infos_2_3)
        .def("add_detailed_vis_infos", add_detailed_vis_infos_0)
        .def("add_detailed_vis_infos", &three_js_group_wrap::add_detailed_vis_infos_1)
        .def("add_animation", &prx::three_js_group_t::add_animation)
        .def("update_vis_infos", &prx::three_js_group_t::update_vis_infos)
        .def("snapshot_state", &prx::three_js_group_t::snapshot_state)
   		.def("output_html", &prx::three_js_group_t::output_html)
   		;
}
