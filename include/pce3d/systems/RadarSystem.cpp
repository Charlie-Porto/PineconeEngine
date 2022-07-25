#ifndef RadarSystem_cpp
#define RadarSystem_cpp

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

#include <pcecs/ecs/System.cpp>

#include "functions/radarFunctions.hpp"

extern ControlPanel control;

namespace pce3d {
class RadarSystem : public ISystem {
public:

  void UpdateEntities() {
    for (auto const& entity : entities) {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);

      const glm::dvec3 view_sphere_intersection_point = glm::normalize(position.center_of_mass_relative_to_camera);
      position.center_of_mass_radar_pixel = radar::convertPointOnViewSphereToPixel(view_sphere_intersection_point);
      position.distance_from_camera = sqrt(glm::dot(position.center_of_mass_relative_to_camera,
                                                    position.center_of_mass_relative_to_camera));


      for (auto const& [id, vertex] : rigid_object.camera_transformed_vertices) {
        const glm::dvec3 screen_plane_intersection_point = glm::normalize(vertex);
        rigid_object.vertex_pixels[id] = radar::convertPointOnViewSphereToPixel(screen_plane_intersection_point);
        rigid_object.vertex_distance_map[id] = sqrt(glm::dot(vertex, vertex));
      }
    }
  }

private:
};
}
#endif /* RadarSystem_cpp */
