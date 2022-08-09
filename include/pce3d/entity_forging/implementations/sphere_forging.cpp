#ifndef sphere_forging_cpp
#define sphere_forging_cpp

#include "../sphere_forging.hpp"


namespace pce3d {
namespace forge {


Entity forgeSphereEntity(const double radius, const glm::dvec3 location, const std::vector<int> color,
                         const glm::dvec3& velocity, const double gravitational_force) {
  Entity new_entity = pce3d::forge::forgeBaseEntity(location);
  control.AddComponent(new_entity, pce::RigidObject{
    .radius=radius, .vertices={std::make_pair(1, location)},
    .mass = 4.0 * PI * pow(radius, 2.0)
  });
  control.AddComponent(new_entity, pce::Surface{.color=color, .collision_elasticity_index=0.7});
  control.AddComponent(new_entity, pce::Force{ .of_gravity = gravitational_force });
  control.AddComponent(new_entity, pce::Motion{
    .speed = 0.0,
    .direction = glm::dvec3(0, 0, 0),
    .velocity = velocity,
    .rotational_speed = 0.0,
    .rotational_axis = glm::dvec3(0, 0, 0),
    .duration = 0.1,
    .previous_resting_position = location,
    .stationary_counter = 0
  });

  return new_entity;
}

}
}





#endif /* sphere_forging_cpp */
