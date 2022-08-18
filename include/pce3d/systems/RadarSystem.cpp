#ifndef RadarSystem_cpp
#define RadarSystem_cpp

/*----------------------------------------------------------------|
--------------------- System Description -------------------------|
determines where on the screen an entity will be rendered,
based on that entity's location in 3space relative to the camera.
-----------------------------------------------------------------*/

#include <iostream>
#include <algorithm>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

#include <pcecs/ecs/System.cpp>
#include "functions/radarFunctions.hpp"
#include "DevRenderSystem.cpp"

extern ControlPanel control;
extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
class RadarSystem : public ISystem {
public:

  void UpdateEntities() {
    std::cout << "---" << '\n';
    for (auto const& entity : entities) {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);
      auto& radar = control.GetComponent<pce::Radar>(entity);

      const glm::dvec3 view_sphere_intersection_point = glm::normalize(position.center_of_mass_relative_to_camera);
      position.center_of_mass_radar_pixel = radar::convertPointOnViewSphereToPixel(view_sphere_intersection_point, true, false);
      position.distance_from_camera = sqrt(glm::dot(position.center_of_mass_relative_to_camera,
                                                    position.center_of_mass_relative_to_camera));

      radar.closest_vertex_distance = 100000;
      radar.farthest_vertex_distance = 0.0;
      for (auto const& [id, vertex] : rigid_object.camera_transformed_vertices) {
        // dev_render_system.AddPointToPointColorMap(rigid_object.camera_transformed_vertices.at(id), {100, 20, 220, 255});
        const glm::dvec3 screen_plane_intersection_point = glm::normalize(vertex);
        rigid_object.vertex_pixels[id] = radar::convertPointOnViewSphereToPixel(screen_plane_intersection_point, false, false);
        rigid_object.vertex_distance_map[id] = sqrt(glm::dot(vertex, vertex));

        if (rigid_object.vertex_distance_map.at(id) < radar.closest_vertex_distance) {
          radar.closest_vertex_id = id;
          radar.closest_vertex_distance = rigid_object.vertex_distance_map.at(id);
        }
        if (rigid_object.vertex_distance_map.at(id) > radar.farthest_vertex_distance) {
          radar.farthest_vertex_id = id;
          radar.farthest_vertex_distance = rigid_object.vertex_distance_map.at(id);
        }
      }
      auto const point = rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id);

      // std::cout << "point " << ", "
      //           <<  point.x  << ", "
      //           <<  point.y  << ", "
      //           <<  point.z  << "\n";
                
      dev_render_system.AddPointToPointColorMap(rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id), {255, 0, 0, 255});
      // dev_render_system.AddPointToPointColorMap(rigid_object.camera_transformed_vertices.at(radar.farthest_vertex_id), {0, 255, 0, 255});
    }
  }

private:
};
}
#endif /* RadarSystem_cpp */
