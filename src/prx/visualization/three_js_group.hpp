#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/utilities/geometry/geometry.hpp"
#include "prx/utilities/general/type_conversions.hpp"

#include <vector>

namespace prx
{

enum class info_geometry_t
{
  LINE = 0,
  QUAD = 1,
  FULL_LINE = 2,
  CIRCLE = 3,
  SPHERE = 3
};

class three_js_group_t
{
public:
  three_js_group_t(const three_js_group_t&) = default;
  three_js_group_t(const std::vector<system_ptr_t>& in_plants,
                   const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles = {});
  ~three_js_group_t();

  void add_vis_infos(info_geometry_t info_type, std::vector<trajectory_t>& tree_vis, std::string body_name,
                     space_t* state_space, std::string color = "0x000000");

  void add_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t* state_space,
                     std::string color = "0x000000");

  void add_vis_infos(info_geometry_t info_type, std::vector<vector_t> positions, std::string color = "0x000000",
                     double radius = 1);

  // This fn generates more detailed visualization of the trajectories

  /**
   * Generates detailed visualization of a trajectory. Time and memory expensive, should only be used for a small number
   * of trajectories, i.e. solution(s).
   * @param info_type   One of info_geometry_t::LINE or info_geometry_t::FULL_LINE. The latter generates more visually
   * accurate trajectories at the cost of time-memory
   * @param traj        Trajectory to generate the visualizations for
   * @param body_name   Body to generate the trajectory visualization for
   * @param state_space State space of the plant for which body_name is part of
   * @param color       Color to use for the visualized trajectory.
   */
  void add_detailed_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name,
                              space_t* state_space, std::string color = "0xFF0000");

  /**
   * Creates an animation for the trajectory using the state space. If the trajectory is empty, only puts the system at
   * start_state.
   * @param traj        Trajectory to animate
   * @param state_space The space_t to use
   * @param default_state Where to put the system if the trajectory is empty.
   */
  void add_animation(const trajectory_t& traj, space_t* state_space, space_point_t default_state = nullptr);

  void update_vis_infos();

  void snapshot_state(double timestamp);

  void output_html(std::string filename);

  void add_tree_log(std::string log_name, space_t* state_space);

  template <typename Position, typename Quaternion, typename Dimension>
  void set_floor_plane(Position position, Quaternion quat, Dimension dimension, std::string color)
  {
    using namespace prx::utilities;
    const std::string x{ convert_to<std::string>(position[0]) };
    const std::string y{ convert_to<std::string>(position[1]) };
    const std::string z{ convert_to<std::string>(position[2]) };
    const std::string str_pos{ x + ", " + y + ", " + z };

    const std::string qx{ convert_to<std::string>(quat[0]) };
    const std::string qy{ convert_to<std::string>(quat[1]) };
    const std::string qz{ convert_to<std::string>(quat[2]) };
    const std::string qw{ convert_to<std::string>(quat[3]) };
    const std::string str_quat{ qx + ", " + qy + ", " + qz + ", " + qw };

    const std::string dim_x{ convert_to<std::string>(dimension[0]) };
    const std::string dim_y{ convert_to<std::string>(dimension[1]) };
    const std::string dim{ dim_x + ", " + dim_y };

    // clang-format off
    _floor = "var geometry = new THREE.PlaneBufferGeometry("+ dim +");"
            "var material = new THREE.MeshPhongMaterial({ color : " + color + "});"
            "var background = new THREE.Mesh(geometry, material);"
            "background.receiveShadow = true;"
            "background.position.set(" + str_pos + ");"
            "background.quaternion = new THREE.Quaternion(" + str_quat + ");"
            "scene.add(background);";
    // clang-format on
  }

protected:
  std::string get_opacity_from_color(const std::string color)
  {
    using namespace prx::utilities;
    // Check if olor is 0xRRGGBB
    if (color.size() != 10)
      return "1";

    // Color is 0xAARRGGBB
    using namespace prx::utilities;
    const std::string alpha_str{ color.substr(2, 2) };
    const double alpha{ convert_to<double>(color[2] + color[3]) };
    constexpr double max_opacity{ 256.0 };
    const std::string opacity{ std::to_string(alpha / max_opacity) };
    return opacity;
  }

  struct vis_info_t
  {
    vector_t position;
    quaternion_t rotation;
    std::weak_ptr<transform_t> transform;
    geometry_type_t geom_type;
    std::vector<double> geom_params;
    std::string body_name;
    std::string color;
    void update_info();
  };

  struct info_geom_params_t
  {
    info_geometry_t first;
    std::vector<vector_t> second;
    std::string third;
    double fourth;
  };

  // clang-format off
  const std::string html_header  // no-lint
      {                          // no-lint
        "<!DOCTYPE html>"
        "<html>"
        "<head>"// no-lint
          "<link rel=\"stylesheet\" href=\"" + js_path + "style.css\">" 
          "<meta charset=utf-8>"
          "<title>ML4KP Visualization</title>"
          "<style> body { margin: 0; } canvas { width: 100%; height: 100%; display: block; }</style>"
        "</head>"
        "<body>"
          "<button id=\"shot\">Screenshot</button>"
          "<button id=\"bt_play\">Play</button>"
          "<button id=\"bt_down_img_seq\">Download Seq</button>"
          "<div class=\"slidecontainer\">"
            "<input type=\"range\" min=\"0\" max=\"1000\" value=\"500\" class=\"slider\" id=\"time_slider\">"
          "</div>"
          "<script src=\"" + js_path + "three.js\"></script>"
          "<script src=\"" + js_path + "FileSaver.js\"></script>"
          "<script src=\"" + js_path + "map_controls.js\"></script>"
          "<script src=\"" + js_path + "prx.js\"></script>"
          "<script src=\"" + js_path + "jszip.min.js\"></script>"
          "<script>"
      };
  // clang-format on
  const std::string html_footer = "animate();</script></body></html>";

  std::string _floor;
  // const std::string html_footer = "</script></body></html>";

  void update_plants();

  std::vector<info_geom_params_t> info_geoms;
  std::vector<system_ptr_t> plants;
  std::vector<std::shared_ptr<vis_info_t>> state_infos;
  std::vector<std::vector<double>> plant_animation_params;
  std::vector<std::shared_ptr<vis_info_t>> obstacle_infos;
  std::string tree_log_file;
  int state_space_dim;
};
}  // namespace prx
