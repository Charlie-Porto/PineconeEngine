#ifndef RenderSystem_cpp
#define RenderSystem_cpp

#include <iostream>
#include <pcecs/ecs/System.cpp>
#include "OrderForRenderSystem.cpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../utilities/functions/render_functions.hpp"
#include "../maths/objects/Quadrilateral.hpp"
#include "../maths/objects/Triangle.hpp"
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


      /* check if item is on screen */
      if (position.center_of_mass_radar_pixel == glm::dvec2(0, 0)) {
        continue;
      }

      /* check if entity is a sphere */ 
      if (rigid_object.radius != 0) {
        // pce::quickdraw::drawFilledCircleClean(position.center_of_mass_radar_pixel, rigid_object.radius * 550.0 / rigid_object.vertex_distance_map.at(1), surface.color);
        pce::render::renderFilledCircleShaded(shade.pixel_shade_map, surface.color);
      }

      
      if (rigid_object.radius == 0) {
        std::vector<std::pair<uint32_t, double>> faces_in_render_order = render::orderFacesByCameraProximity(
            rigid_object.face_vertex_map, rigid_object.vertex_distance_map);
        
        for (size_t i = 0; i < faces_in_render_order.size(); ++i) {
          uint32_t face = faces_in_render_order[i].first;

          /* get face color */
          std::vector<int> face_color = surface.color;
          face_color[0] = int(double(face_color[0]) * shade.face_shade_map.at(face));
          face_color[1] = int(double(face_color[1]) * shade.face_shade_map.at(face));
          face_color[2] = int(double(face_color[2]) * shade.face_shade_map.at(face));
          
          std::vector<glm::dvec2> render_vertices{};
          size_t face_vertex_count = rigid_object.face_vertex_map.at(face).size();
          /* collect vertices */
          for (size_t j = 0; j < face_vertex_count; ++j) {
            render_vertices.push_back(rigid_object.vertex_pixels.at(rigid_object.face_vertex_map.at(face)[j]));
          }

          if (face_vertex_count == 4) {
            auto quad = pce3d::maths::Quadrilateral{render_vertices[0], render_vertices[1], 
                                                    render_vertices[2],render_vertices[3]};
            pce::quickdraw::drawFilledQuadrilateral(quad, face_color);
          }
          if (face_vertex_count == 3) {
            auto tri = pce3d::maths::Triangle{render_vertices[0], render_vertices[1], render_vertices[2]};
            pce::quickdraw::drawFilledTriangle(tri, face_color);
          }

        }
      }
    }
  }

private:
};
}
#endif /* RenderSystem_cpp */
