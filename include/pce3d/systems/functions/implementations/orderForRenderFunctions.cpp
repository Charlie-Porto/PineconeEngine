#ifndef orderForRenderFunctions_cpp
#define orderForRenderFunctions_cpp

#include "../orderForRenderFunctions.hpp"

namespace pce3d {
namespace render_order {

void insertEntityIntoRenderOrderVectorLinear(const std::pair<uint32_t, double>& entity, 
                                             std::vector<std::pair<uint32_t, double>>& order_vector) {
  size_t i = 0;
  while (i < order_vector.size()+1) {
    if (i == order_vector.size()) {
      order_vector.push_back(entity);
      break;
    }
    if (entity.second > order_vector[i].second) {
      order_vector.insert(order_vector.begin()+i, entity);
      break;
    }
    ++i;
  }
}




void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
                                       std::vector<std::pair<uint32_t, double>>& order_vector) {

  size_t left = 0;
  size_t right = order_vector.size();
  size_t index = 0;

  while (left < right) {
    size_t mid = (left + right) / 2;
    bool condition_index_1 = false;
    bool condition_index_2 = false;

    /* check index #1 */
    if (mid <= 0) { condition_index_1 = true; }
    else if (order_vector[mid+1].second < entity.second) { condition_index_1 = true; }

    /* check index #2 */
    if (mid >= order_vector.size()) { condition_index_2 = true; }
    if (order_vector[mid].second > entity.second) { condition_index_2 = true; }

    /* check indices and adjust left or right if needed */
    if (condition_index_1 && condition_index_2) { index = mid; break; }
    else if (!condition_index_1) { left = mid; }
    else if (!condition_index_2) { right = mid; }
  }
  if (order_vector.size() > 0) { order_vector.insert(order_vector.begin() + index+1, entity); } 
  else { order_vector.push_back(entity); }
}


std::pair<bool, size_t> tryInsertEntityIntoRenderOrderMap(const orderTag& entity_tag, std::vector<orderTag>& order_list) {
  for (size_t i = 0; i != order_list.size()+1; ++i) {
    if (i == order_list.size()) { order_list.push_back(entity_tag); }

    /* check if entity closest vertex is farther than both comparison entity vertices */
    if (entity_tag.closest_vertex_distance > order_list[i].closest_vertex_distance
     && entity_tag.farthest_vertex_distance > order_list[i].farthest_vertex_distance) {
      order_list.insert(order_list.begin() + i, entity_tag);
      return std::make_pair(true, i);
    }
    /* check if entity closest vertex is closer than both comparison entity vertices */
    else if (entity_tag.closest_vertex_distance < order_list[i].closest_vertex_distance
         &&  entity_tag.farthest_vertex_distance < order_list[i].farthest_vertex_distance) {
      continue;
    }
    /* handle case in which entity closest vertex distance is inbetween the others */
    else {
      return std::make_pair(false, i);
    }
  }
  return std::make_pair(false, 0);
}


void insertEntityBetweenVerticesIntoRenderOrderMapAtIndex(const orderTag& entity_tag, size_t i, 
                                                          const glm::dvec3 closest_vertex_rotated_pos,
                                                          std::vector<orderTag>& order_list) {
  /* NOTE: this function will require a call to the ControlPanel */
  const uint32_t mentity = order_list[i].entity;
  auto const& mrigid_object = control.GetComponent<pce::RigidObject>(mentity);
  /* 1. get 3 points of face closest to the mvertex */ 
  /*   A. get closest vertex to mvertex */
  /*   B. get vertices connected to vertex */
  /*   C. get the unit vectors that bisect adjacent pairs of edges */
  /*   D. calc distance between mvertex the vertex + each unit vector */
  /*   E. return the 3 points that make of the plane of the side */
  const std::vector<uint32_t> closest_face_points 
      = pce3d::maths::calculateClosestPolyhedronFaceToPoint(mrigid_object.camera_transformed_vertices,
                                                            mrigid_object.edges,
                                                            mrigid_object.face_vertex_map,
                                                            closest_vertex_rotated_pos);
  assert(closest_face_points.size() == 3);
  /* 2. get point on this face that is closest to the vertex*/
  /*   A. relatively simple calc */
  const glm::dvec3 face_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
                                    mrigid_object.vertices.at(closest_face_points[0]),
                                    mrigid_object.vertices.at(closest_face_points[1]),
                                    mrigid_object.vertices.at(closest_face_points[2]),
                                    closest_vertex_rotated_pos);
  
  const double face_point_distance = sqrt(glm::dot(face_point, face_point));
  if (entity_tag.closest_vertex_distance < face_point_distance) {
    if (i == order_list.size()-1) { order_list.push_back(entity_tag); }
    else {
      order_list.insert(order_list.begin() + i+1, entity_tag);
    }
  } else {
    order_list.insert(order_list.begin() + i, entity_tag);
  }
}









}
}




#endif /* orderForRenderFunctions_cpp */
