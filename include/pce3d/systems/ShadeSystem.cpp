#ifndef ShadeSystem_cpp
#define ShadeSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for calculating amount of light reaching object faces 
(and, ergo, the shaded color of each face)
-----------------------------------------------------------------*/

#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <pcecs/ecs/System.cpp>
#include "functions/shadeFunctions.hpp"
#include "../utilities/functions/raster_functions.hpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../maths/functions/quaternion_functions.hpp"
  
extern ControlPanel control;
extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
class ShadeSystem : public ISystem {
public:
  
  void setOrdinaryZoomIndex(double ORDINARY_ZOOM_INDEX) {
    ORDINARY_ZOOM_INDEX_ = ORDINARY_ZOOM_INDEX;
  }

  
  void UpdateEntities(const glm::dquat& camera_versor) {

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
          
          /* extra stuff */
          // glm::dvec3 r_vertex_a = glm::normalize(rigid_object.camera_transformed_vertices.at(vertices[1]) - rigid_object.camera_transformed_vertices.at(vertices[0]));
          // glm::dvec3 r_vertex_b = glm::normalize(rigid_object.camera_transformed_vertices.at(vertices[2]) - rigid_object.camera_transformed_vertices.at(vertices[0]));
          // const glm::dvec3 r_normal_vect = glm::cross(r_vertex_a, r_vertex_b);

          // const glm::dvec3 avg_vertex = (rigid_object.camera_transformed_vertices.at(vertices[0]) 
          //                              + rigid_object.camera_transformed_vertices.at(vertices[1]) 
          //                              + rigid_object.camera_transformed_vertices.at(vertices[2])) / 3.0;

          // const glm::dvec3 face_normal_vect_point = avg_vertex + r_normal_vect * 5.0;

          // std::cout << "r_normal_vect: " 
          //           << r_normal_vect.x << ", "
          //           << r_normal_vect.y << ", "
          //           << r_normal_vect.z << '\n';

          // std::vector<int> mcolor = {255, 255, 255, 255};

          // switch (face) 
          // {
          //   case 1:
          //     mcolor = {0, 0, 255, 255};
          //     break;
          //   case 2:
          //     mcolor = {0, 0, 255, 255};
          //     break;
          //   case 3:
          //     mcolor = {0, 255, 0, 255};
          //     break;
          //   case 4:
          //     mcolor = {255, 0, 0, 255};
          //     break;
          //   case 5:
          //     mcolor = {200, 30, 200, 255};
          //     break;
          //   default:
          //     break;
          // }

          // dev_render_system.AddPointToPointColorMap(avg_vertex, mcolor, 1.0);
          // dev_render_system.AddPointToPointColorMap(face_normal_vect_point, mcolor, 1.0);
        }
      }

      /* update spheres */
      else {
        /* if close to sphere, do shortcut alg to avoid exp complexity */
        if (rigid_object.vertex_distance_map.at(1) < 25.0) { 
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
        /* do pixel color calculation */
        using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;
        const glm::vec2 ncenter_point = position.center_of_mass_radar_pixel * ORDINARY_ZOOM_INDEX_;
        PixelMap outline_pixels = pce::raster::getCircleOutlinePixelPairs(ncenter_point.x,
                                                                          ncenter_point.y,
                                                                          rigid_object.radius * 800.0 / rigid_object.vertex_distance_map.at(1));
        shade::calculateFaceBrightnessForSpherePixels(ROTATED_LIGHT_FLOW_DIRECTION_,
                                                      position.center_of_mass_relative_to_camera,
                                                      rigid_object.radius,
                                                      outline_pixels,
                                                      face_shade.pixel_shade_map);
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
