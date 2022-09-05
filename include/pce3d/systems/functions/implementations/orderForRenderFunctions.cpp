#ifndef orderForRenderFunctions_cpp
#define orderForRenderFunctions_cpp

#include "../orderForRenderFunctions.hpp"

namespace pce3d {
namespace render_order {

uint32_t getCloserOfTwoOverlappingEntitiesToOrigin(const orderTag& a_entity_tag, 
                                                   const orderTag& b_entity_tag)
{

  uint32_t big_entity = b_entity_tag.entity;
  uint32_t small_entity = a_entity_tag.entity;
  orderTag small_tag = a_entity_tag;
  orderTag big_tag = b_entity_tag;
  
  bool swap = false;
  auto const& a_radar = control.GetComponent<pce::Radar>(small_entity);
  auto const& b_radar = control.GetComponent<pce::Radar>(big_entity);
  auto& a_rigid_object = control.GetComponent<pce::RigidObject>(small_entity);
  auto& b_rigid_object = control.GetComponent<pce::RigidObject>(big_entity);

  if (a_rigid_object.radius > 0 && b_rigid_object.radius > 0)
  {
    return a_entity_tag.closest_vertex_distance <= b_entity_tag.closest_vertex_distance
      ? b_entity_tag.entity : a_entity_tag.entity;
  }

  /* NOTE: this swap capability is critical */
  // if (b_entity_tag.closest_vertex_distance - b_entity_tag.farthest_vertex_distance == 0
  //  || a_entity_tag.closest_vertex_distance - a_entity_tag.farthest_vertex_distance == 0)
  // {
    // swap = true;
  // }
  // if (abs(b_entity_tag.closest_vertex_distance - b_entity_tag.farthest_vertex_distance) 
    // < abs(a_entity_tag.closest_vertex_distance - a_entity_tag.farthest_vertex_distance))

  if (big_tag.closest_vertex_distance < small_tag.closest_vertex_distance)
  {
    swap = true;
    // std::cout << "1swap" << '\n';
  }
  if (big_tag.farthest_vertex_distance < small_tag.farthest_vertex_distance)
  {
    swap = true;
    // std::cout << "2swap" << '\n';
  }
  if (pce3d::maths::calculateDistanceBetweenVectors(a_rigid_object.vertices.at(a_radar.closest_vertex_id), a_rigid_object.vertices.at(a_radar.farthest_vertex_id))
   >  pce3d::maths::calculateDistanceBetweenVectors(b_rigid_object.vertices.at(b_radar.closest_vertex_id), b_rigid_object.vertices.at(b_radar.farthest_vertex_id)))
  {
    swap = true;
    // std::cout << "3swap" << '\n';
  }

  if (swap) 
  {
    big_entity = a_entity_tag.entity;
    small_entity = b_entity_tag.entity;
    small_tag = b_entity_tag;
    big_tag = a_entity_tag;
  }

  auto& big_rigid_object = control.GetComponent<pce::RigidObject>(big_entity);
  auto& big_radar = control.GetComponent<pce::Radar>(big_entity);
  auto& sm_rigid_object = control.GetComponent<pce::RigidObject>(small_entity);
  auto& sm_radar = control.GetComponent<pce::Radar>(small_entity);

  uint32_t big_closest_face = 1;
  double big_closest_face_corner_distance = 10000;
  
  for (auto const& [face, corner] : big_rigid_object.vertex_face_corner_map.at(big_radar.closest_vertex_id)) 
  {
    const double distance = pce3d::maths::calculateDistanceBetweenVectors(
      big_rigid_object.camera_rotated_face_corner_map.at(corner), small_tag.closest_vertex_location);

    if (distance < big_closest_face_corner_distance) 
    {
      big_closest_face_corner_distance = distance;
      big_closest_face = face;
    }
  }

  const glm::dvec3 big_entity_face_plane_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
    big_rigid_object.camera_transformed_vertices.at(big_rigid_object.face_vertex_map.at(big_closest_face)[0]),
    big_rigid_object.camera_transformed_vertices.at(big_rigid_object.face_vertex_map.at(big_closest_face)[1]),
    big_rigid_object.camera_transformed_vertices.at(big_rigid_object.face_vertex_map.at(big_closest_face)[2]),
    // small_tag.closest_vertex_location
    sm_rigid_object.camera_transformed_vertices.at(sm_radar.farthest_vertex_id)
  );
  
  const double face_point_magnitude = sqrt(glm::dot(big_entity_face_plane_point, big_entity_face_plane_point));
  // dev_render_system.AddPointToPointColorMap(a_rigid_object.camera_transformed_vertices.at(a_radar.farthest_vertex_id), {129, 160, 200, 255}, 7.0);
  // dev_render_system.AddPointToPointColorMap(b_rigid_object.camera_transformed_vertices.at(b_radar.farthest_vertex_id), {129, 160, 200, 255}, 7.0);

  dev_render_system.AddPointToPointColorMap(big_tag.closest_vertex_location, {200, 100, 200, 255}, 7.0);
  dev_render_system.AddPointToPointColorMap(small_tag.closest_vertex_location, {20, 100, 200, 255}, 7.0);
  dev_render_system.AddPointToPointColorMap(big_entity_face_plane_point, {200, 50, 100, 255}, 7.0);
  // dev_render_system.AddPointToPointColorMap(small_tag.closest_vertex_location, {100, 200, 10, 255}, 4.0);
  // dev_render_system.AddPointToPointColorMap(
    // big_rigid_object.camera_rotated_face_corner_map.at(
      // big_rigid_object.face_vertex_corner_map.at(
        // big_closest_face).at(big_radar.closest_vertex_id)), {0, 255, 29, 255}, 7.0);

  const double tolerance = .001;
  if (face_point_magnitude - small_tag.closest_vertex_distance > tolerance)
  {
    return face_point_magnitude < small_tag.farthest_vertex_distance
      ? big_entity : small_entity;
  }
  else
  {
    return big_tag.closest_vertex_distance <= small_tag.closest_vertex_distance
      ? big_entity : small_entity;
  }
}

uint32_t getCloserOfTwoEntitiesToOrigin(const orderTag& a_entity_tag, const orderTag& b_entity_tag)
{
  if (a_entity_tag.closest_vertex_distance <= b_entity_tag.closest_vertex_distance
   && a_entity_tag.farthest_vertex_distance <= b_entity_tag.closest_vertex_distance)
  {
    return a_entity_tag.entity;
  }
  if (a_entity_tag.closest_vertex_distance >= b_entity_tag.farthest_vertex_distance
   && a_entity_tag.farthest_vertex_distance >= b_entity_tag.farthest_vertex_distance)
  {
    return b_entity_tag.entity;
  }
  if (a_entity_tag.closest_vertex_distance == a_entity_tag.farthest_vertex_distance
   && b_entity_tag.closest_vertex_distance == b_entity_tag.farthest_vertex_distance)
  {
    return a_entity_tag.closest_vertex_distance <= b_entity_tag.closest_vertex_distance
      ? b_entity_tag.entity : a_entity_tag.entity;
  }
  else
  {
    // std::cout << "getting closer of two overlapping" << '\n';
    // std::cout << "entity a: " << a_entity_tag.entity << " | " << "entity b: " << b_entity_tag.entity << '\n';
    return getCloserOfTwoOverlappingEntitiesToOrigin(a_entity_tag, b_entity_tag);
  }
}

void insertEntityIntoOrderMapBinary(const orderTag& entity_tag,
                                    std::vector<orderTag>& order_list)
{
  size_t left = 0;
  size_t right = order_list.size();
  size_t index = 0;
  size_t previous_left_right_difference = 0;

  while (left < right) 
  {
    // std::cout << "right: " << right << '\n';
    // std::cout << "left: " << left << '\n';
    size_t mid = (left + right) / 2; 
    if (right - left == previous_left_right_difference)
    {
      // index = 1;
      // std::cout << "breaking due to incoming endless loop" << '\n';
      // break;
      --right;
    }
    previous_left_right_difference = right - left;

    uint32_t closer_left_neighbor_or_entity = getCloserOfTwoEntitiesToOrigin(entity_tag, order_list[mid]);
    uint32_t closer_right_neighbor_or_entity = entity_tag.entity; 

    if (order_list.size() > 1 && mid < order_list.size()-1) 
    { 
      closer_right_neighbor_or_entity = getCloserOfTwoEntitiesToOrigin(entity_tag, order_list[mid+1]);
    }

    // std::cout << "closer left: " <<  closer_left_neighbor_or_entity<< '\n';
    // std::cout << "closer right: " <<  closer_right_neighbor_or_entity<< '\n';

    const bool this_entity_closer_than_left  
        = (closer_left_neighbor_or_entity == entity_tag.entity) ? true : false;
    const bool this_entity_closer_than_right  
        = (closer_right_neighbor_or_entity == entity_tag.entity) ? true : false;

    if (this_entity_closer_than_left && !this_entity_closer_than_right)
    {
      // std::cout << "spot found: " << '\n';
      index = mid+1;
      break;
    }
    if (this_entity_closer_than_left && this_entity_closer_than_right
       && mid == order_list.size()-1)
    {
      // std::cout << "spot found: " << '\n';
      index = mid+1;
      break;
    }
    else if (!this_entity_closer_than_left)
    {
      right = mid;
      // std::cout << "new right: " << right << '\n';
      index = mid;
    }
    else if (this_entity_closer_than_right)
    {
      left = mid; 
      // std::cout << "new left: " << left << '\n';
      index = mid;
      index = mid;
    }
  }
  // std::cout << "inserting entity:  " << entity_tag.entity << "at index: "<< index <<'\n';
  if (order_list.size() < 2 && index != 0) { order_list.push_back(entity_tag); }
  else if (order_list.size() > 2 && index >= order_list.size()) 
  {
    order_list.push_back(entity_tag);
  }
  else 
  {
    order_list.insert(order_list.begin() + index, entity_tag);
  }
}


void insertEntityIntoOrderMapStartingAtEnd(const orderTag& entity_tag,
                                           std::vector<orderTag>& order_list)
{
  size_t index = order_list.size()-1;
  for (size_t i = order_list.size(); i != -1; --i)
  {
    if (order_list[i].closest_vertex_distance == order_list[i].farthest_vertex_distance
     && entity_tag.closest_vertex_distance == entity_tag.farthest_vertex_distance)
    {
      if (order_list[i].closest_vertex_distance < entity_tag.closest_vertex_distance)
      {
        continue;
      }
      else
      {
        index = i;
        break;
      }
    }
    uint32_t closer_entity = getCloserOfTwoEntitiesToOrigin(entity_tag, order_list[i]);
    if (closer_entity == entity_tag.entity)
    {
      index = i;
      break;
    }
  }
  if (index == order_list.size()-1)
  {
    order_list.push_back(entity_tag);
  }
  else
  {
    order_list.insert(order_list.begin() + index, entity_tag);
  }
}
                                

}
}




#endif /* orderForRenderFunctions_cpp */
