#ifndef RenderSystem_cpp
#define RenderSystem_cpp

#include <iostream>
#include <pcecs/ecs/System.cpp>
#include "OrderForRenderSystem.cpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../maths/objects/Quadrilateral.hpp"


extern ControlPanel control;

namespace pce3d {
class RenderSystem : public ISystem {
public:

  void UpdateEntities() {
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto const& surface = control.GetComponent<pce::Surface>(entity);
      pce::quickdraw::drawCircle(position.center_of_mass_radar_pixel, 10.0/(position.distance_from_camera/3.0), surface.color, 10.0);

      for (auto const& [id, pixel] : rigid_object.vertex_pixels) {
        pce::quickdraw::drawCircle(pixel, 10.0/(position.distance_from_camera/3.0), surface.color, 10.0);
      }

      
      /* draw filled faces */
      uint32_t face = 1;
      glm::dvec2 vertex_a = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[0]);
      glm::dvec2 vertex_b = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[1]);
      glm::dvec2 vertex_c = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[2]);
      glm::dvec2 vertex_d = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[3]);

      
      auto quad = pce3d::maths::Quadrilateral{vertex_a, vertex_b, vertex_c, vertex_d};
      pce::quickdraw::drawFilledQuadrilateral(quad, surface.color, 10.0);

      /* draw all lines */
      for (auto const& edge : rigid_object.edges) {
        pce::quickdraw::drawLine(
          rigid_object.vertex_pixels.at(edge.first),
          rigid_object.vertex_pixels.at(edge.second),
          {255, 255, 255, 255},
          10.0
        );
      }


    }
  }

private:
};
}
#endif /* RenderSystem_cpp */
