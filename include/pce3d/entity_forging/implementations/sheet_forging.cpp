#ifndef sheet_forging_cpp
#define sheet_forging_cpp

#include "../sheet_forging.hpp"

namespace pce3d {
namespace forge {

Entity forgeRectSheetEntity(const double w, const double l, const glm::dvec3& center, 
                            const double angle, const glm::dvec3& axis_of_rotation,
                            const std::vector<int>& color, const bool is_transparent) {
  VertexMap vertices{};
  vertices[1] = glm::dvec3(center + glm::dvec3(w/2.0, 0, -l/2.0));
  vertices[2] = glm::dvec3(center + glm::dvec3(-w/2.0, 0, -l/2.0));
  vertices[3] = glm::dvec3(center + glm::dvec3(-w/2.0, 0, l/2.0));
  vertices[4] = glm::dvec3(center + glm::dvec3(w/2.0, 0, l/2.0));
  FaceVertexMap face_map{
    {1, {1, 2, 3, 4}}
  };
  EdgeMap edge_map = { 
    {1, std::make_pair(1, 2)},
    {2, std::make_pair(2, 3)},
    {3, std::make_pair(3, 4)},
    {4, std::make_pair(4, 1)}
  };

  FaceEdgeMap face_edge_map{};
  createFaceEdgeMap(face_map, edge_map, face_edge_map);

  FaceCornerMap face_corner_map{};
  FaceVertexCornerMap face_vertex_corner_map{};
  VertexFaceCornerMap vertex_face_corner_map{};

  pce3d::forge::createFaceVertexCornerMaps(vertices, face_map, 
                                           face_corner_map, 
                                           face_vertex_corner_map, vertex_face_corner_map,
                                           center);


  pce3d::forge::rotateVertices(vertices, angle, axis_of_rotation, center);

  Entity new_entity = pce3d::forge::forgeBaseEntity(center);
  control.AddComponent(new_entity, pce::Surface{.color=color, .collision_elasticity_index=0.9, .is_transparent = is_transparent});
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .mass = w * l,
    .is_deadbod = true,
    .is_restingbod = false,
    .vertices = vertices,
    .edges = edge_map,
    .face_count = 1,
    .face_vertex_map = face_map,
    .face_edge_map = face_edge_map,
    .face_corner_map = face_corner_map,
    .face_vertex_corner_map = face_vertex_corner_map,
    .vertex_face_corner_map = vertex_face_corner_map
  });

  return new_entity;
}



Entity forgeTriangleSheetEntity(const std::vector<glm::dvec3>& triangle_points,
                               const std::vector<int>& color, const bool is_transparent) {
  const glm::dvec3 point_avg = (triangle_points[0] + triangle_points[1]  + triangle_points[2]) / 3.0;
  VertexMap vertices = {
    {1, triangle_points[0]},
    {2, triangle_points[1]},
    {3, triangle_points[2]},
  };
  FaceVertexMap face_map = {
    {1, {1, 2, 3}}
  };
  EdgeMap edge_map = {
    {1, std::make_pair(1, 2)},
    {2, std::make_pair(2, 3)},
    {3, std::make_pair(3, 1)},
  };

  FaceEdgeMap face_edge_map{};
  createFaceEdgeMap(face_map, edge_map, face_edge_map);

  FaceCornerMap face_corner_map{};
  FaceVertexCornerMap face_vertex_corner_map{};
  VertexFaceCornerMap vertex_face_corner_map{};

  pce3d::forge::createFaceVertexCornerMaps(vertices, face_map, 
                                           face_corner_map, 
                                           face_vertex_corner_map, vertex_face_corner_map,
                                           point_avg, true);

  Entity new_entity = pce3d::forge::forgeBaseEntity(point_avg);
  control.AddComponent(new_entity, pce::Surface{.color = color, .collision_elasticity_index=0.9});
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .mass = 100.0,
    .is_deadbod = true,
    .is_restingbod = false,
    .vertices = vertices,
    .edges = edge_map,
    .face_count = 1,
    .face_vertex_map = face_map,
    .face_edge_map = face_edge_map,
    .face_corner_map = face_corner_map,
    .face_vertex_corner_map = face_vertex_corner_map,
    .vertex_face_corner_map = vertex_face_corner_map
  });
  control.AddComponent(new_entity, pce::Render{ .is_registered = false });
  
  return new_entity;
}







}}

#endif /* sheet_forging_cpp */
