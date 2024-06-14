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
  ~geometry_t();

  std::weak_ptr<PQP_Model> get_collision_geometry();

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
  std::shared_ptr<PQP_Model> collision_geometry;

  std::string vis_color;

  geometry_type_t geom_type;
  std::vector<double> params;
  bool params_set;
};
}  // namespace prx