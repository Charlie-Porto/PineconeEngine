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

  dev_render_system.AddPointToPointColorMap(face_point, {255, 0, 0, 255});

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



void insertEntityIntoOrderMap(const orderTag& entity_tag,
                              std::vector<orderTag>& order_list, size_t start_position) {
  // std::cout << "inserting entity into order map: " << entity_tag.entity << '\n';
  if (order_list.empty()) { order_list.push_back(entity_tag); }
  else {
    for (size_t i = start_position; i != order_list.size(); ++i) {
      if (entity_tag.closest_vertex_distance > order_list[i].farthest_vertex_distance) {
        order_list.insert(order_list.begin() + i, entity_tag);
        break;
      }
      else if (entity_tag.closest_vertex_distance < order_list[i].closest_vertex_distance
       && entity_tag.farthest_vertex_distance < order_list[i].closest_vertex_distance) {
        if (i == order_list.size()-1) {
          order_list.push_back(entity_tag); 
          break;
        }
        else {
          continue;
        }
      }
      else if (entity_tag.closest_vertex_distance < order_list[i].closest_vertex_distance
       && entity_tag.farthest_vertex_distance > order_list[i].closest_vertex_distance) {
        const pce3d::orderTag temptap = order_list[i];
        order_list[i] = entity_tag;
        insertEntityIntoOrderMapBesideIndex(temptap, i, order_list);
        break;
      }
      else {
      /* pick up here */
        /* this code below is temporary */
        // order_list.insert(order_list.begin() + i, entity_tag);
        // std::cout << "entity closest vertex distance" << entity_tag.closest_vertex_distance << '\n';
        // std::cout << "mentity closest vertex distance" << order_list[i].closest_vertex_distance << '\n';
        // std::cout << "mentity farthest vertex distance" << order_list[i].farthest_vertex_distance << '\n';
        insertEntityIntoOrderMapBesideIndex(entity_tag, i, order_list);
        break;
      }
    }
  }
}


void insertEntityIntoOrderMapBesideIndex(const orderTag& entity_tag, 
                                         size_t i,
                                         std::vector<orderTag>& order_list) {
  // std::cout << "inserting direct" << '\n';
  size_t order_list_size = order_list.size();
  // std::cout << "entity: " << entity_tag.entity << '\n';
  // std::cout << "mentity: " << order_list[i].entity << '\n';
  const orderTag mentity_tag = order_list[i];
  auto const mrigid_object = control.GetComponent<pce::RigidObject>(mentity_tag.entity);
  auto const mradar = control.GetComponent<pce::Radar>(mentity_tag.entity);
  uint32_t closest_face = 2;
  double closest_face_corner_distance = 10000;
  double vertex_distance = sqrt(glm::dot(
    mrigid_object.camera_transformed_vertices.at(mradar.farthest_vertex_id) - entity_tag.closest_vertex_location,
    mrigid_object.camera_transformed_vertices.at(mradar.farthest_vertex_id) - entity_tag.closest_vertex_location));
  
  // for (auto const& [face, vertex_list] : mrigid_object.face_vertex_map) {
  //   std::cout << "face: " << face << " | " << "vertex count: " << vertex_list.size() << '\n';
  // } 

  // dev_render_system.AddPointToPointColorMap(mrigid_object.camera_transformed_vertices.at(mradar.farthest_vertex_id), {255, 0, 0, 255});
  // std::cout << "---" << '\n';
  for (auto const& [vertex, corner_map] : mrigid_object.vertex_face_corner_map) {
    // std::cout << "vertex: " << vertex << " | " << "farthest_vertex_id: " << mradar.farthest_vertex_id << '\n';
    if (vertex == mradar.farthest_vertex_id) {
      // dev_render_system.AddPointToPointColorMap(mrigid_object.camera_transformed_vertices.at(vertex), {255, 0, 0, 255});
      // dev_render_system.AddPointToPointColorMap(mrigid_object.camera_transformed_vertices.at(mradar.farthest_vertex_id), {255, 0, 0, 255});
      for (auto const& [face, corner] : corner_map) {

        // std::cout << "face: " << face << '\n';
        const double distance = sqrt(glm::dot(
          mrigid_object.camera_rotated_face_corner_map.at(corner) - entity_tag.closest_vertex_location,
          mrigid_object.camera_rotated_face_corner_map.at(corner) - entity_tag.closest_vertex_location));
          
        dev_render_system.AddPointToPointColorMap(mrigid_object.camera_rotated_face_corner_map.at(corner), {0, 255, 0, 255});
        // auto const A = mrigid_object.camera_rotated_face_corner_map.at(corner);
        // std::cout << "A: " << A.x << ", " << A.y << ", " << A.z << "\n";
        if (distance < closest_face_corner_distance) {
          closest_face_corner_distance = distance;
          closest_face = face;
        }
      }
    }
  }
  // std::cout << "closest_vertex_location: " << closest_vertex_location.x << ", " << closest_vertex_location.y << ", " << closest_vertex_location.z << "\n";
  // std::cout << "closest face: " << closest_face << '\n';
  const glm::dvec3 face_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
    mrigid_object.camera_transformed_vertices.at(mrigid_object.face_vertex_map.at(closest_face)[0]),
    mrigid_object.camera_transformed_vertices.at(mrigid_object.face_vertex_map.at(closest_face)[1]),
    mrigid_object.camera_transformed_vertices.at(mrigid_object.face_vertex_map.at(closest_face)[2]),
    entity_tag.closest_vertex_location);
  
  dev_render_system.AddPointToPointColorMap(face_point, {255, 0, 0, 255});
  dev_render_system.AddPointToPointColorMap(entity_tag.closest_vertex_location, {0, 0, 200, 255});

  const double face_point_magnitude = sqrt(glm::dot(face_point, face_point));
  // std::cout << "vertex point: " << closest_vertex_location.x << ", "
  //                               << closest_vertex_location.y << ", "
  //                               << closest_vertex_location.z << "\n";
  // std::cout << "face point: " << face_point.x << ", "
  //                             << face_point.y << ", "
  //                             << face_point.z << "\n";
  // std::cout << "face point distance: " << face_point_magnitude << '\n';
  // std::cout << "closest vertex distance: " << entity_tag.closest_vertex_distance << '\n';

  if (face_point_magnitude <= entity_tag.closest_vertex_distance) {
    // std::cout << "inserting entity: " << entity_tag.entity << '\n';
    order_list.insert(order_list.begin() + i, entity_tag);
  }
  else {
    if (i >= order_list_size - 1) { order_list.push_back(entity_tag); }
    else { 
      // std::cout << "sending back at index: " << i+1 << '\n';
      // std::cout << "entity: " << entity_tag.entity << '\n';
      insertEntityIntoOrderMap(entity_tag, order_list, i + 1);
    }
  }
}



}
}




#endif /* orderForRenderFunctions_cpp */
