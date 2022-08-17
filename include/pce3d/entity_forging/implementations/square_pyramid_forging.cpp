#ifndef square_pyramid_forging_cpp
#define square_pyramid_forging_cpp

#include "../square_pyramid_forging.hpp"

namespace pce3d {
namespace forge {


Entity forgeSquarePyramidEntity(const double h, const double base_side_length, 
                                const glm::dvec3& center, const glm::dquat& local_rotation,
                                const std::vector<int>& color) {
  VertexMap vertices = {
    {1, glm::dvec3(center.x + base_side_length/2.0, center.y - h/2.0, center.z + base_side_length/2.0)},
    {2, glm::dvec3(center.x - base_side_length/2.0, center.y - h/2.0, center.z + base_side_length/2.0)},
    {3, glm::dvec3(center.x - base_side_length/2.0, center.y - h/2.0, center.z - base_side_length/2.0)},
    {4, glm::dvec3(center.x + base_side_length/2.0, center.y - h/2.0, center.z - base_side_length/2.0)},
    {5, glm::dvec3(center.x, center.y + h/2.0, center.z)}
  };

  EdgeMap edge_map = {
    std::make_pair(1, 2),
    std::make_pair(2, 3),
    std::make_pair(3, 4),
    std::make_pair(4, 1),
    std::make_pair(1, 5),
    std::make_pair(2, 5),
    std::make_pair(3, 5),
    std::make_pair(4, 5)
  };

  FaceVertexMap face_vertex_map = {
    {1, {1, 2, 3, 4}},
    {2, {4, 5, 1}},
    {3, {1, 5, 2}},
    {4, {3, 5, 2}},
    {5, {4, 5, 3}}
  };

  VertexVertexMap vvmap = {
    {1, {2, 5, 4}},
    {2, {1, 5, 3}},
    {3, {2, 5, 4}},
    {4, {1, 5, 3}},
    {5, {1, 2, 3, 4}}
  };

  FaceCornerMap face_corner_map{};
  FaceVertexCornerMap face_vertex_corner_map{};
  VertexFaceCornerMap vertex_face_corner_map{};

  pce3d::forge::createFaceVertexCornerMaps(vertices, face_vertex_map, 
                                           face_corner_map, 
                                           face_vertex_corner_map, vertex_face_corner_map,
                                           center, true);


  Entity new_entity = pce3d::forge::forgeBaseEntity(center);
  control.AddComponent(new_entity, pce::Surface{.color = color, .collision_elasticity_index = 0.9});
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .mass = 10.0,
    .is_deadbod = true,
    .is_restingbod = true,
    .vertices = vertices,
    .base_face_id = 1,
    .face_count = 5,
    .vertex_vertex_map = vvmap,
    .edges = edge_map,
    .face_corner_map = face_corner_map,
    .face_vertex_corner_map = face_vertex_corner_map,
    .vertex_face_corner_map = vertex_face_corner_map,
    .index_face_map = {},
    .face_index_map = {},
    .entity_face_collision_map = {},
    .entity_time_collision_map = {},
    .face_vertex_map = face_vertex_map
  });

  return new_entity;

}


}}

#endif /* square_pyramid_forging_cpp */
