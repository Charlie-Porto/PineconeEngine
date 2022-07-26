#ifndef sheet_forging_cpp
#define sheet_forging_cpp

#include "../sheet_forging.hpp"

namespace pce3d {
namespace forge {

Entity forgeSheetEntity(const double w, const double l, const glm::dvec3& center, 
                        const glm::dquat& local_rotation, const std::vector<int>& color) {
  VertexMap vertices{};
  vertices[1] = glm::dvec3(center + glm::dvec3(w, 0, -l));
  vertices[2] = glm::dvec3(center + glm::dvec3(-w, 0, -l));
  vertices[3] = glm::dvec3(center + glm::dvec3(-w, 0, l));
  vertices[4] = glm::dvec3(center + glm::dvec3(w, 0, l));

  FaceVertexMap face_map{};
  face_map[1] = {1, 2, 3, 4};

  EdgeMap edge_map = { 
    std::make_pair(1, 2),
    std::make_pair(2, 3),
    std::make_pair(3, 4),
    std::make_pair(4, 1)
  };

  /* to handle cases of rotation later */

  Entity new_entity = control.CreateEntity();
  control.AddComponent(new_entity, pce::Position{.actual_center_of_mass = center});
  control.AddComponent(new_entity, pce::Surface{.color = color});
  control.AddComponent(new_entity, pce::FaceShade{});
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .vertices = vertices,
    .edges = edge_map,
    .face_vertex_map = face_map
  });

  return new_entity;
}

}}

#endif /* sheet_forging_cpp */