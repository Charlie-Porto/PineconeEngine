#ifndef rectangular_prism_forging_cpp
#define rectangular_prism_forging_cpp

#include "../rectangular_prism_forging.hpp"
#include <ezprint.cpp>
#include <vezprint.cpp>

namespace pce3d {
namespace forge {


Entity forgeRectPrismEntity(const double w, const double h, const double l,
                            const glm::dvec3& center, const double angle, const glm::dvec3& axis,
                            const std::vector<int>& color) {

  /* create new entity's major attributes */
  VertexMap e_vertex_map = calculateRectPrismOriginalVertexLocations(w, h, l, center);
  FaceVertexMap e_face_vertex_map = assignVerticesToFaces();
  EdgeMap e_edge_map = assignEdgesToVertices();
  VertexVertexMap vvmap = assignVerticestoVertices();
  FaceCornerMap face_corner_map{};
  FaceVertexCornerMap face_vertex_corner_map{};
  VertexFaceCornerMap vertex_face_corner_map{};

  pce3d::forge::rotateVertices(e_vertex_map, angle, axis, center);
  pce3d::forge::createFaceVertexCornerMaps(e_vertex_map, e_face_vertex_map, 
                                           face_corner_map, 
                                           face_vertex_corner_map, vertex_face_corner_map,
                                           center);

  /* adjust vertex points based on rotation and center point */
  for (auto& [id, vec3] : e_vertex_map) {
    vec3 += center;
  }
  
  /* create the new entity */
  Entity new_entity = pce3d::forge::forgeBaseEntity(center);
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .mass = w * h * l,
    .is_deadbod = true,
    .is_restingbod = false,
    .vertices = e_vertex_map,
    .vertex_vertex_map = vvmap,
    .base_face_id = 1,
    .face_count = 6,
    .edges = e_edge_map,
    .face_vertex_map = e_face_vertex_map,
    .face_corner_map = face_corner_map,
    .face_vertex_corner_map = face_vertex_corner_map,
    .vertex_face_corner_map = vertex_face_corner_map,
    .index_face_map = {},
    .face_index_map = {},
    .entity_face_collision_map = {}
  });
  control.AddComponent(new_entity, pce::Surface{.color=color, .collision_elasticity_index=0.9});

  for (auto const& [vertex, face_corners] : vertex_face_corner_map) {
    std::cout << "vertex: " << vertex << '\n';
  }

  return new_entity;
}



VertexMap calculateRectPrismOriginalVertexLocations(const double w, const double h, const double l, 
                                                    const glm::dvec3& center) {
  const double half_w = w / 2.0; 
  const double half_h = h / 2.0; 
  const double half_l = l / 2.0;

  const std::vector<double> h_halfs = {half_h, -half_h};
  const std::vector<double> w_halfs = {half_w, -half_w};
  const std::vector<double> l_halfs = {half_l, -half_l};



  VertexMap vertex_map = {};
  uint32_t vertex_id = 1;
  for (auto const& h_half : h_halfs) {
    for (auto const& w_half : w_halfs) {
      for (auto const& l_half : l_halfs) {
        vertex_map[vertex_id] = glm::dvec3(w_half, h_half, l_half);
        ++vertex_id;
      }
    }
  }

  return vertex_map;
}



FaceVertexMap assignVerticesToFaces() {
  /* assignment done by hand (for now) */
  FaceVertexMap face_vertex_map = {
    {1, {1, 3, 7, 5}},
    {2, {1, 2, 4, 3}},
    {3, {6, 8, 4, 2}},
    {4, {7, 8, 6, 5}},
    {5, {7, 3, 4, 8}},
    {6, {1, 2, 6, 5}}
  };
  return face_vertex_map;
}



EdgeMap assignEdgesToVertices() {
  EdgeMap edge_map = {
    {1, 3}, {1, 2}, {1, 5}, 
    {4, 2}, {4, 3}, {4, 8},
    {7, 5}, {7, 3}, {7, 8},
    {6, 5}, {6, 8}, {6, 2}
  };

  return edge_map;
}



VertexVertexMap assignVerticestoVertices() {
  VertexVertexMap vvmap = {
    {1, {4, 2, 5}},
    {2, {1, 3, 6}},
    {3, {1, 4, 7}},
    {4, {2, 8, 7}},
    {5, {1, 7, 6}},
    {6, {2, 5, 8}},
    {7, {5, 3, 8}},
    {8, {7, 4, 6}}
  };
  return vvmap;
}


}
}






#endif /* rectangular_prism_forging_cpp */
