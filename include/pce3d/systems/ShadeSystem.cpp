#ifndef ShadeSystem_cpp
#define ShadeSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for calculating amount of light reaching object faces (thus the color)
-----------------------------------------------------------------*/

#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <pcecs/ecs/System.cpp>
#include <ezprint.cpp>
#include "functions/shadeFunctions.hpp"
#include "../utilities/functions/raster_functions.hpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../maths/functions/quaternion_functions.hpp"
  
extern ControlPanel control;

namespace pce3d {


class ShadeSystem : public ISystem {
public:

  void UpdateEntities(const glm::dquat& camera_versor) {
    ROTATED_LIGHT_FLOW_DIRECTION_ = pce::rotateVector3byQuaternion(LIGHT_FLOW_DIRECTION_, camera_versor);

    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto& face_shade = control.GetComponent<pce::FaceShade>(entity); 
      
      if (rigid_object.radius == 0) {
        for (auto& [face, vertices] : rigid_object.face_vertex_map) {
          glm::dvec3 vertex_a = rigid_object.vertices.at(vertices[1]) - rigid_object.vertices.at(vertices[0]);
          glm::dvec3 vertex_b = rigid_object.vertices.at(vertices[2]) - rigid_object.vertices.at(vertices[0]);
          const glm::dvec3 normal_vect = glm::cross(vertex_a, vertex_b);
          face_shade.face_shade_map[face] = shade::calculateFaceBrightness(LIGHT_FLOW_DIRECTION_, normal_vect);
        }
      }

      else {
        if (rigid_object.vertex_distance_map.at(1) < 15.0) { continue; }
        /* do pixel color calculation */
        using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;

        const glm::vec2 ncenter_point = position.center_of_mass_radar_pixel * 10.0;

        PixelMap outline_pixels = pce::raster::getCircleOutlinePixelPairs(ncenter_point.x,
                                                                          ncenter_point.y,
                                                                          rigid_object.radius * 500.0 / rigid_object.vertex_distance_map.at(1));

        shade::calculateFaceBrightnessForSpherePixels(ROTATED_LIGHT_FLOW_DIRECTION_,
                                                      position.center_of_mass_relative_to_camera,
                                                      rigid_object.radius,
                                                      outline_pixels,
                                                      face_shade.pixel_shade_map);
      }
    }
  }

private:
  const glm::dvec3 LIGHT_FLOW_DIRECTION_ = glm::dvec3(0.4, -1.0, 0.3);
  glm::dvec3 ROTATED_LIGHT_FLOW_DIRECTION_;

};
}
#endif /* ShadeSystem_cpp */
