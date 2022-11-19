#ifndef RenderSystem_cpp
#define RenderSystem_cpp

/*----------------------------------------------------------------|
--------------------- System Description -------------------------|
system that handles the rendering of on-screen entities
-----------------------------------------------------------------*/

#include <iostream>
#include <pcecs/ecs/System.cpp>
#include "OrderForRenderSystem.cpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../utilities/functions/render_functions.hpp"
#include "../maths/objects/Quadrilateral.hpp"
#include "../maths/objects/Triangle.hpp"
#include "functions/renderFunctions.hpp"
#include "objects/orderTag.hpp"

extern ControlPanel control;
extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
class RenderSystem : public ISystem {
public:

  void setOrdinaryZoomIndex(double ORDINARY_ZOOM_INDEX) {
    ORDINARY_ZOOM_INDEX_ = ORDINARY_ZOOM_INDEX;
  }


  void UpdateEntities(const std::vector<uint32_t> order_of_render){
    /* render objects in order of furthest from camera to closest */
    // std::cout << "updating render system" << '\n';

    for (auto const& entity : order_of_render) {
      // std::cout << "entity: " << entity << '\n';
      auto const& radar = control.GetComponent<pce::Radar>(entity);
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto const& surface = control.GetComponent<pce::Surface>(entity);
      auto const& shade = control.GetComponent<pce::FaceShade>(entity);

      /* check if item is on screen */
      if (position.center_of_mass_radar_pixel == glm::dvec2(0, 0)) {
        // std::cout << "skipping render; off screen per radar system" << '\n';
        continue;
      }

      /* handle rendering of sphere entities */ 
      if (rigid_object.radius != 0 && rigid_object.vertices.size() == 1) {
        if (rigid_object.vertex_distance_map.at(1) < 20.0) {
          const std::vector<int> ncolor 
            = {int(surface.color[0] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel 
                                                               * ORDINARY_ZOOM_INDEX_)),
               int(surface.color[1] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel 
                                                               * ORDINARY_ZOOM_INDEX_)),
               int(surface.color[2] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel
                                                               * ORDINARY_ZOOM_INDEX_)),
               255};
          pce::quickdraw::drawFilledCircleClean(position.center_of_mass_radar_pixel, rigid_object.radius * 800.0 / rigid_object.vertex_distance_map.at(1), ncolor);
        } else {
          pce::render::renderFilledCircleShaded(shade.pixel_shade_map, surface.color);
        }

        // std::cout << "rendering sphere" << '\n';
      }
      
      // Render Cylinder
      if (rigid_object.radius != 0 && rigid_object.vertices.size() > 1) {
        std::cout << "rendering CYLINDER" << '\n';
        for (auto const& [id, vpair] : rigid_object.edges)
        {
          const glm::dvec3 a = rigid_object.camera_transformed_vertices.at(vpair.first);
          const glm::dvec3 b = rigid_object.camera_transformed_vertices.at(vpair.second);
          std::cout << "a: " << a.x << ", " << a.y << ", " << a.z << '\n';
          // std::cout << "a length: " << a.y - b.y << '\n';
          std::cout << "b: " << b.x << ", " << b.y << ", " << b.z << '\n';
          dev_render_system.AddPointToPointColorMap(a, {55, 250, 25, 255}, 4.0);
          dev_render_system.AddPointToPointColorMap(b, {55, 25, 250, 255}, 4.0);
          pce::render::renderUnOrdinaryZoomedLine(rigid_object.vertex_pixels.at(vpair.first), rigid_object.vertex_pixels.at(vpair.second), surface.color);
        }
      }

      /* handle rendering of non-sphere entities */ 
      else if (rigid_object.radius == 0 && !surface.is_transparent) {
        // std::vector<std::pair<uint32_t, double>> faces_in_render_order = render::orderFacesByCameraProximity(
            // rigid_object.face_vertex_map, rigid_object.vertex_distance_map);

        // dev_render_system.AddPointToPointColorMap(rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id), {0, 155, 55, 255}, 4.0);
        // std::cout << "calling faces order function" << '\n';
        std::vector<uint32_t> faces_in_render_order{};
        if (rigid_object.face_count == 6 || rigid_object.face_count == 1)
        {
          // std::cout << "getting rect faces ordered for render " << '\n';
          faces_in_render_order = render::getFacesOrderedForRender(radar.closest_vertex_id,
                                                                   rigid_object.vertex_face_corner_map,
                                                                   rigid_object.camera_rotated_face_corner_map);
        }
        else if (rigid_object.face_count == 4 || rigid_object.face_count == 5)
        {
          // std::cout << "getting pyramid faces ordered for render " << '\n';
          faces_in_render_order = render::getPyramidFacesOrderedForRender(
                                      radar.closest_vertex_id,
                                      rigid_object.base_face_id,
                                      rigid_object.camera_transformed_vertices,
                                      rigid_object.face_vertex_map,
                                      rigid_object.vertex_distance_map,
                                      rigid_object.vertex_face_corner_map,
                                      rigid_object.camera_rotated_face_corner_map);
        }
        
        for (size_t i = 0; i < faces_in_render_order.size(); ++i) {
          // std::cout << "face: " << faces_in_render_order[i] << '\n';
          // uint32_t face = faces_in_render_order[i].first;
          uint32_t face = faces_in_render_order[i];


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

          /* render each side of the non-sphere entity */
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
      else if (rigid_object.radius == 0 && surface.is_transparent) 
      {
        pce::render::renderTransparentObject(rigid_object, surface.color);
      }
    }
  }
private:
  double ORDINARY_ZOOM_INDEX_;


};
}
#endif /* RenderSystem_cpp */
