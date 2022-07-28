#ifndef sphere_forging_cpp
#define sphere_forging_cpp

#include "../sphere_forging.hpp"


namespace pce3d {
namespace forge {

Entity forgeSphereEntity(const double radius, const glm::dvec3 location, const std::vector<int> color) {
  Entity sphere_entity = control.CreateEntity();
  control.AddComponent(sphere_entity, pce::RigidObject{.radius=radius, .vertices={std::make_pair(1, location)}});
  control.AddComponent(sphere_entity, pce::Position{.actual_center_of_mass=location});
  control.AddComponent(sphere_entity, pce::LocalRotation{.versor = {1.0, 0, 0, 0}});
  control.AddComponent(sphere_entity, pce::Surface{.color=color});
  control.AddComponent(sphere_entity, pce::FaceShade{});
  return sphere_entity;
}

}
}





#endif /* sphere_forging_cpp */
