#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/external/PQP/PQP.h"
#include "prx/utilities/geometry/geometry-imp.hpp"

#include <memory>

namespace prx
{
enum class geometry_type_t
{
  // initialized with lengths on each axis
  BOX = 0,
  // initialized with radius
  SPHERE = 1,
  // initialized with lengths along each axis
  ELLIPSOID = 2,
  // initialized with radius of caps and length of cylinder
  CAPSULE = 3,
  // initialed with radius of cone opening and height of cone
  CONE = 4,
  // initialized with radius of cylinder and height
  CYLINDER = 5
};

class geometry_t
{
public:
  geometry_t(geometry_type_t new_geom_type);
  geometry_t(const prx::param_loader& param_loader)
  {
    init(param_loader);
  }
  ~geometry_t();

  static inline geometry_type_t geometry_type(const std::string geom)
  {
    const std::map<std::string, geometry_type_t> available_types{
      { "BOX", geometry_type_t::BOX },              // no-lint
      { "SPHERE", geometry_type_t::SPHERE },        // no-lint
      { "ELLIPSOID", geometry_type_t::ELLIPSOID },  // no-lint
      { "CAPSULE", geometry_type_t::CAPSULE },      // no-lint
      { "CONE", geometry_type_t::CONE },            // no-lint
      { "CYLINDER", geometry_type_t::CYLINDER }     // no-lint
    };
    std::string upper_geom{ prx::to_upper(geom) };

    return available_types.at(upper_geom);
  }
  static inline std::string geometry_type(const geometry_type_t& geom)
  {
    const std::map<geometry_type_t, std::string> available_types{
      { geometry_type_t::BOX, "BOX" },              // no-lint
      { geometry_type_t::SPHERE, "SPHERE" },        // no-lint
      { geometry_type_t::ELLIPSOID, "ELLIPSOID" },  // no-lint
      { geometry_type_t::CAPSULE, "CAPSULE" },      // no-lint
      { geometry_type_t::CONE, "CONE" },            // no-lint
      { geometry_type_t::CYLINDER, "CYLINDER" }     // no-lint
    };

    return available_types.at(geom);
  }

  std::weak_ptr<PQP_Model> get_collision_geometry();

  std::shared_ptr<PQP_Model> collision_geometry()
  {
    return _collision_geometry;
  }

  geometry_type_t get_geometry_type();

  std::vector<double> get_geometry_params();

  void initialize_geometry(const std::vector<double>& geom_params);

  void generate_collision_geometry();

  void set_visualization_color(std::string c)
  {
    vis_color = c;
  }

  std::string get_visualization_color()
  {
    return vis_color;
  }

  virtual prx::param_loader init()
  {
    prx::param_loader pl{};
    pl["color"].set(vis_color);
    pl["type"].set(geometry_type(geom_type));
    pl["parameters"].set(params);
    return pl;
  }

  virtual void init(const prx::param_loader& param_loader)
  {
    if (param_loader.exists("color"))
    {
      vis_color = param_loader["color"].as<std::string>();
    }
    if (param_loader.exists("type") and param_loader.exists("parameters"))
    {
      geom_type = geometry_type(param_loader["type"].as<std::string>());
      params = param_loader["parameters"].as<std::vector<double>>();
      generate_collision_geometry();
    }
  }

  template <typename PQPModelType>
  static std::shared_ptr<PQPModelType> create_collision_geometry(const geometry_type_t geom_type,
                                                                 const std::vector<double>& params)
  {
    using namespace prx::utilities;

    std::shared_ptr<PQPModelType> collision_geometry;
    // prx_assert(params_set, "Geometry params have not been provided, so a collision geometry cannot be
    // generated."); prx_assert(collision_geometry == nullptr, "Trying to recreate collision geometries when they
    // have already been created.");
    switch (geom_type)
    {
      case geometry_type_t::BOX:
        prx_assert(params.size() >= 3, "geometry_type_t::Box needs 3 parameters");
        collision_geometry =
            std::shared_ptr<PQPModelType>(create_box_trimesh<PQPModelType>(params[0], params[1], params[2]));
        break;
      case geometry_type_t::SPHERE:
        prx_assert(params.size() >= 1, "geometry_type_t::SPHERE needs 1 parameters");
        collision_geometry = std::shared_ptr<PQPModelType>(create_sphere_trimesh<PQPModelType>(params[0]));
        break;
      case geometry_type_t::ELLIPSOID:
        prx_assert(params.size() >= 3, "geometry_type_t::ELLIPSOID needs 3 parameters");
        collision_geometry =
            std::shared_ptr<PQPModelType>(create_ellipsoid_trimesh<PQPModelType>(params[0], params[1], params[2]));
        break;
      case geometry_type_t::CAPSULE:
        prx_assert(params.size() >= 2, "geometry_type_t::CAPSULE needs 2 parameters");
        collision_geometry = std::shared_ptr<PQPModelType>(create_capsule_trimesh<PQPModelType>(params[0], params[1]));
        break;
      case geometry_type_t::CONE:
        prx_assert(params.size() >= 2, "geometry_type_t::CONE needs 2 parameters");
        collision_geometry = std::shared_ptr<PQPModelType>(create_cone_trimesh<PQPModelType>(params[0], params[1]));
        break;
      case geometry_type_t::CYLINDER:
        prx_assert(params.size() >= 2, "geometry_type_t::CYLINDER needs 2 parameters");
        collision_geometry = std::shared_ptr<PQPModelType>(create_cylinder_trimesh<PQPModelType>(params[0], params[1]));
        break;
    };
    return collision_geometry;
  }

private:
  std::shared_ptr<PQP_Model> _collision_geometry;

  std::string vis_color;

  geometry_type_t geom_type;
  std::vector<double> params;
  bool params_set;
};
}  // namespace prx