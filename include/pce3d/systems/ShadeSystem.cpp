#ifndef ShadeSystem_cpp
#define ShadeSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for calculating amount of light reaching object faces 
(and, ergo, the shaded color of each face)
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include "../maths/functions/quaternion_functions.hpp"
#include <unordered_map>
#include <glm/geometric.hpp>
#include <pcecs/ecs/System.cpp>
#include "functions/shadeFunctions.hpp"
#include "../utilities/functions/raster_functions.hpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../maths/functions/rounding_functions.hpp"
  
extern ControlPanel control;
extern pce3d::DevRenderSystem dev_render_system;
const double ZONE_GRANULARITY = 1.0;

namespace pce3d {
class ShadeSystem : public ISystem {
public:
  
  void setOrdinaryZoomIndex(double ORDINARY_ZOOM_INDEX) {
    ORDINARY_ZOOM_INDEX_ = ORDINARY_ZOOM_INDEX;
  }


  void DoPreLoopSetup()
  {
    for (auto const& entity : entities)
    {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      if (rigid_object.radius != 0)
      {
        auto& face_shade = control.GetComponent<pce::FaceShade>(entity); 

        pce3d::shade::doSphereLightScan(
          rigid_object.vertices.at(1),
          rigid_object.radius,
          face_shade.surface_zone_brightness_map,
          LIGHT_FLOW_DIRECTION_,
          ZONE_GRANULARITY
        );
      }
    }
  }

  
  void UpdateEntities(const glm::dquat& camera_versor, const glm::dvec3& camera_transformation) {
    ROTATED_LIGHT_FLOW_DIRECTION_ = pce::rotateVector3byQuaternion(LIGHT_FLOW_DIRECTION_, camera_versor);

    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto& face_shade = control.GetComponent<pce::FaceShade>(entity); 
      
      /* update non-spheres */
      if (rigid_object.radius == 0) {
        for (auto& [face, vertices] : rigid_object.face_vertex_map) {

          glm::dvec3 vertex_a = rigid_object.vertices.at(vertices[1]) - rigid_object.vertices.at(vertices[0]);
          glm::dvec3 vertex_b = rigid_object.vertices.at(vertices[2]) - rigid_object.vertices.at(vertices[0]);
          const glm::dvec3 normal_vect = glm::cross(vertex_a, vertex_b);
          face_shade.face_shade_map[face] = shade::calculateFaceBrightness(LIGHT_FLOW_DIRECTION_, normal_vect);
        }
      }

      /* update spheres */
      else {
        /* if close to sphere, do shortcut alg to avoid exp complexity */
        if (rigid_object.vertex_distance_map.at(1) < 20.0) { 
          
          std::unordered_map<glm::dvec2, glm::dvec2> center_pixel 
                                     = {{position.center_of_mass_radar_pixel * ORDINARY_ZOOM_INDEX_,
                                         position.center_of_mass_radar_pixel * ORDINARY_ZOOM_INDEX_}};
          shade::calculateFaceBrightnessForSpherePixels(ROTATED_LIGHT_FLOW_DIRECTION_,
                                                        position.center_of_mass_relative_to_camera,
                                                        rigid_object.radius,
                                                        center_pixel,
                                                        face_shade.pixel_shade_map);
          continue;
        } 
        // /* do pixel color calculation */
        using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;
        const glm::vec2 ncenter_point = position.center_of_mass_radar_pixel * ORDINARY_ZOOM_INDEX_;
        PixelMap outline_pixels = pce::raster::getCircleOutlinePixelPairs(ncenter_point.x,
                                                                          ncenter_point.y,
                                                                          rigid_object.radius * 800.0 / rigid_object.vertex_distance_map.at(1));

        if (rigid_object.vertex_distance_map.at(1) > 20.0)
        { 
          shade::calculateFaceBrightnessForSpherePixels(ROTATED_LIGHT_FLOW_DIRECTION_,
                                                      position.center_of_mass_relative_to_camera,
                                                      rigid_object.radius,
                                                      outline_pixels,
                                                      face_shade.pixel_shade_map);
          continue;
        }

        // face_shade.pixel_shade_map.clear();
        // face_shade.camera_transformed_surface_zone_brightness_map.clear();


        // // for (auto const& [zone, brightness] : face_shade.surface_zone_brightness_map)
        // // {
        // //   const glm::dvec3 rzone = zone - camera_transformation;
        // //   glm::dvec3 tzone = pce::rotateVector3byQuaternion(rzone, camera_versor);
        // //   // std::cout << "tzone: "
        // //             // << tzone.x << ", "
        // //             // << tzone.y << ", "
        // //             // << tzone.z << '\n';
        // //   tzone = pce3d::round::roundVec3ComponentsToNearestInterval(
        // //     ZONE_GRANULARITY, tzone
        // //   );
        // //   // tzone = glm::dvec3(
        // //     // int(tzone.x),
        // //     // int(tzone.y),
        // //     // int(tzone.z)
        // //   // );


        // //   // std::cout << "tzone ROUNDED: "
        // //             // << tzone.x << ", "
        // //             // << tzone.y << ", "
        // //             // << tzone.z << '\n';

        // //   face_shade.camera_transformed_surface_zone_brightness_map[tzone] = brightness;

        // //   // dev_render_system.AddPointToPointColorMap(tzone, {255, 0, 200, 255}, 1.0);
        // // }

        // shade::mapSpherePixelsToBrightnessZones(
        //   face_shade.pixel_shade_map,
        //   outline_pixels,
        //   face_shade.surface_zone_brightness_map,
        //   ROTATED_LIGHT_FLOW_DIRECTION_,
        //   rigid_object.vertices.at(1),
        //   rigid_object.radius,
        //   ZONE_GRANULARITY
        // );


      }
    }
  }

private:
  const glm::dvec3 LIGHT_FLOW_DIRECTION_ = glm::dvec3(0.2, -1.0, 0.5);
  glm::dvec3 ROTATED_LIGHT_FLOW_DIRECTION_;
  double ORDINARY_ZOOM_INDEX_;

};
}
#endif /* ShadeSystem_cpp */
