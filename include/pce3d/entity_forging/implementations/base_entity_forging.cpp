#ifndef base_entity_forging_cpp
#define base_entity_forging_cpp

#include "../base_entity_forging.hpp"


namespace pce3d {
namespace forge {
  
  uint32_t forgeBaseEntity(
      const glm::dvec3 center_location
  )
  {
    uint32_t new_entity = control.CreateEntity();
    control.AddComponent(new_entity, pce::Position{.actual_center_of_mass = center_location});
    control.AddComponent(new_entity, pce::Radar{
      .closest_vertex_id = 1,
      .closest_vertex_distance = 100000.0,
      .farthest_vertex_id = 1,
      .farthest_vertex_distance = 0.0
    });
    control.AddComponent(new_entity, pce::FaceShade{});
    control.AddComponent(new_entity, pce::Render{ .is_registered = false });
    control.AddComponent(new_entity, pce::OrderOfRenderRegistration{});

    return new_entity;
  }

}
}


#endif /* base_entity_forging_cpp */
