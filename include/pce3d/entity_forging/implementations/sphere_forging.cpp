#ifndef sphere_forging_cpp
#define sphere_forging_cpp

#include "../sphere_forging.hpp"


namespace pce3d {
namespace forge {


Entity forgeSphereEntity(const double radius, const glm::dvec3 location, const std::vector<int> color) {
  Entity sphere_entity = control.CreateEntity();
  control.AddComponent(sphere_entity, pce::RigidObject{
    .radius=radius, .vertices={std::make_pair(1, location)},
    .mass = 4.0 * PI * pow(radius, 2.0)
  });
  control.AddComponent(sphere_entity, pce::Position{.actual_center_of_mass=location});
  control.AddComponent(sphere_entity, pce::LocalRotation{.versor = {1.0, 0, 0, 0}});
  control.AddComponent(sphere_entity, pce::Surface{.color=color, .collision_elasticity_index=0.7});
  control.AddComponent(sphere_entity, pce::FaceShade{});
  control.AddComponent(sphere_entity, pce::Force{
    .of_gravity = 0.0
    /* other fields filled by map and physics systems */
  });
  return sphere_entity;
}

}
}





#endif /* sphere_forging_cpp */
