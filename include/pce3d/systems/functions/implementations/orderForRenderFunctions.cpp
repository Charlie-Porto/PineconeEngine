#ifndef orderForRenderFunctions_cpp
#define orderForRenderFunctions_cpp

#include "../orderForRenderFunctions.hpp"
#include <ezprint.cpp>

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
                                                          std::vector<orderTag>& order_list) {
  // std::cout << "placing in specific index within order list" << '\n';
  /* NOTE: this function will require a call to the ControlPanel */
  /* 1. get face closest to the mvertex */ 
  /*   A. get closest vertex to mvertex */
  /*   B. get vertices connected to vertex */
  /*   C. get the unit vectors that bisect adjacent pairs of edges */
  /*   D. calc distance between mvertex the vertex + each unit vector */
  /*   E. return the face that corresponds to the selected point (contains the 3 points) */

  /* 2. get point on this face that is closest to the vertex*/
  /*   A. relatively simple calc */
}









}
}




#endif /* orderForRenderFunctions_cpp */
