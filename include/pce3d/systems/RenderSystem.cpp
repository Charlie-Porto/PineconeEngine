#ifndef RenderSystem_cpp
#define RenderSystem_cpp

#include <iostream>
#include <pcecs/ecs/System.cpp>
#include "OrderForRenderSystem.cpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../maths/objects/Quadrilateral.hpp"
#include "functions/renderFunctions.hpp"


extern ControlPanel control;

namespace pce3d {
class RenderSystem : public ISystem {
public:

  void UpdateEntities(std::vector<std::pair<uint32_t, double>> order_of_render) {

    for (auto const& entity_pair : order_of_render) {
      auto const entity = entity_pair.first;
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto const& surface = control.GetComponent<pce::Surface>(entity);
      auto const& shade = control.GetComponent<pce::FaceShade>(entity);

      if (   abs(position.center_of_mass_radar_pixel.y) >= 1000 
          || abs(position.center_of_mass_radar_pixel.x) >= 1000) {
        continue;
      }

      std::vector<std::pair<uint32_t, double>> faces_in_render_order = render::orderFacesByCameraProximity(
          rigid_object.face_vertex_map, rigid_object.vertex_distance_map);
      

      for (size_t i = 0; i < faces_in_render_order.size(); ++i) {
        uint32_t face = faces_in_render_order[i].first;
        glm::dvec2 vertex_a = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[0]);
        glm::dvec2 vertex_b = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[1]);
        glm::dvec2 vertex_c = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[2]);
        glm::dvec2 vertex_d = rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[3]);

        std::vector<int> face_color = surface.color;
        face_color[0] = int(double(face_color[0]) * shade.face_shade_map.at(face));
        face_color[1] = int(double(face_color[1]) * shade.face_shade_map.at(face));
        face_color[2] = int(double(face_color[2]) * shade.face_shade_map.at(face));

        auto quad = pce3d::maths::Quadrilateral{vertex_a, vertex_b, vertex_c, vertex_d};
        pce::quickdraw::drawFilledQuadrilateral(quad, face_color, 10.0);
      }


    }
  }

private:
};
}
#endif /* RenderSystem_cpp */
