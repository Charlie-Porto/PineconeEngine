#ifndef RadarSystem_cpp
#define RadarSystem_cpp

#include <iostream>
#include <algorithm>
#include <vector>
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
      position.center_of_mass_radar_pixel = radar::convertPointOnViewSphereToPixel(view_sphere_intersection_point, true, false);
      position.distance_from_camera = sqrt(glm::dot(position.center_of_mass_relative_to_camera,
                                                    position.center_of_mass_relative_to_camera));


      // std::vector<uint32_t> pixel_provided{}
      // std::vector<uint32_t> no_pixel_provided{};
      for (auto const& [id, vertex] : rigid_object.camera_transformed_vertices) {
        const glm::dvec3 screen_plane_intersection_point = glm::normalize(vertex);

        // if (screen_plane_intersection_point.z < -0.2) {
          // /* need to interpolate */
          // no_pixel_provided.push_back(id);
          rigid_object.vertex_pixels[id] = radar::convertPointOnViewSphereToPixel(screen_plane_intersection_point, false, false);
          rigid_object.vertex_distance_map[id] = sqrt(glm::dot(vertex, vertex));
        // } else {
          // rigid_object.vertex_pixels[id] = radar::convertPointOnViewSphereToPixel(screen_plane_intersection_point, false, false);
          // rigid_object.vertex_distance_map[id] = sqrt(glm::dot(vertex, vertex));
          // pixel_provided.push_back(id);
        // }
      }
      /* loop through vertices that need interpolation */
      // if (pixel_provided.size() > 0) {
      //   for (auto const& id : no_pixel_provided) {
      //     std::cout << "POINT NEEDS INTERPOLATION" << '\n';

      //     /* get connected vertex */
      //     uint32_t connected_vertex = 0;
      //     for (auto const& vertex : rigid_object.vertex_vertex_map.at(id)) {
      //       if (std::count(pixel_provided.begin(), pixel_provided.end(), vertex)) {
      //         connected_vertex = vertex;
      //         break;
      //       }
      //     }

      //     if (connected_vertex == 0) {
      //       std::cout << "ERROR: connected vertex not found; will fail to access map" << '\n';
      //       break;
      //     } else {
      //       /* interpolate */
      //       std::cout << "connected vertex: " << connected_vertex << '\n';
      //       glm::dvec3 interpolated_view_sphere_point = radar::interpolateViewSphereIntersectionPoint(
      //         rigid_object.vertices.at(id), 0.0, rigid_object.vertices.at(connected_vertex));
      //       rigid_object.vertex_pixels[id] = radar::convertPointOnViewSphereToPixel(interpolated_view_sphere_point, true, false);
      //     }
      //   }

      // }
    }
  }

private:
};
}
#endif /* RadarSystem_cpp */
