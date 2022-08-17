#ifndef renderFunctions_cpp
#define renderFunctions_cpp

#include <vector>
#include <utility>
#include "../renderFunctions.hpp"


namespace pce3d {
namespace render {


std::vector<uint32_t> getFacesOrderedForRender(const uint32_t closest_vertex_id,
                                               const VertexFaceCornerMap& vertex_face_corner_map,
                                               const FaceCornerMap& face_corner_map)
{
  std::vector<std::pair<uint32_t, double>> corner_distance_order{};
  /* returns faces connected to the closest vertex, in order. */
  // std::cout << "beginning loop" << '\n';
  for (auto const& [face_id, corner_id] : vertex_face_corner_map.at(closest_vertex_id))
  {
    // std::cout << "calculating corner distance" << '\n';
    const double corner_distance = pce3d::maths::calculateDistanceBetweenVectors(face_corner_map.at(corner_id), glm::dvec3(0, 0, 0));
    // std::cout << "corner_distance: " << corner_distance << '\n';
    // dev_render_system.AddPointToPointColorMap(face_corner_map.at(corner_id), {255, 30, 100, 255});
    // std::cout << "corner distance: "<< corner_distance << '\n';
    if (corner_distance_order.empty()) 
    { 
      // std::cout << "pushing back; empty" << '\n';
      corner_distance_order.push_back(std::make_pair(face_id, corner_distance)); 
    }
    else
    {
      // std::cout << "checking for insertion place" << '\n';
      // std::cout << "current vector size: "<< corner_distance_order.size() << '\n';
      for (size_t i = 0; i != corner_distance_order.size(); ++i)
      {
        // std::cout << "current vector size: "<< corner_distance_order.size() << '\n';
        // std::cout << "current vector position distance: "<< corner_distance_order[i].second << '\n';
        if (corner_distance > corner_distance_order[i].second)
        {
          // std::cout << "attempting to insert at: "<< i << '\n';
          corner_distance_order.insert(corner_distance_order.begin() + i, std::make_pair(face_id, corner_distance));
          break;
        }
        else if (i == corner_distance_order.size()-1) { corner_distance_order.push_back(std::make_pair(face_id, corner_distance)); break; }
      }
    }
  }
  std::vector<uint32_t> faces_in_order{};
  for (auto const& [face_id, corner_distance] : corner_distance_order)
  {
    faces_in_order.push_back(face_id);
  }
  return faces_in_order; 
}

std::vector<std::pair<uint32_t, double>> orderFacesByCameraProximity(
    const FaceVertexMap& face_vertex_map,
    const VertexDistanceMap& vertex_distance_map) {
  std::vector<std::pair<uint32_t, double>> faces_furthest_to_closest{};
  std::vector<double> averages{};
  for (auto const& [face, vertices] : face_vertex_map) {
    
    double average_vertex = 0.0;
    double count = 0;
    for (auto const& vertex : vertices) {
      average_vertex += vertex_distance_map.at(vertex);
      ++count;
    }
    average_vertex = average_vertex/count;

    std::vector<double> faces_vertex_distances;
    for (int i = 0; i < count; ++i) {
      faces_vertex_distances.push_back(vertex_distance_map.at(vertices[i]));
    }
   
    const double closest_vertex = *std::min_element(faces_vertex_distances.begin(), faces_vertex_distances.end());

    if (faces_furthest_to_closest.size() == 0) { 
      faces_furthest_to_closest.push_back(std::make_pair(face, closest_vertex)); 
      averages.push_back(average_vertex);
      continue;
    }
    else {
      size_t i = 0;
      while (i < faces_furthest_to_closest.size()+1) {
        if (i == faces_furthest_to_closest.size()) {
          faces_furthest_to_closest.push_back(std::make_pair(face, closest_vertex));
          averages.push_back(average_vertex);
          break;
        }
        if (closest_vertex == faces_furthest_to_closest[i].second) {
          if (average_vertex >= averages[i]) {
            faces_furthest_to_closest.insert(faces_furthest_to_closest.begin()+i, std::make_pair(face, closest_vertex));
            averages.insert(averages.begin()+i, average_vertex);
            break;
          }
        }
        if (closest_vertex > faces_furthest_to_closest[i].second) {
            faces_furthest_to_closest.insert(faces_furthest_to_closest.begin()+i, std::make_pair(face, closest_vertex));
            averages.insert(averages.begin()+i, average_vertex);
            break;
        }
        ++i;
      }
    }
  }
  return faces_furthest_to_closest;
}



std::vector<uint32_t> getPyramidFacesOrderedForRender(const uint32_t closest_vertex_id, 
                                                      const uint32_t base_face_id,
                                                      const VertexMap vertices,
                                                      const FaceVertexMap& face_vertex_map,
                                                      const VertexDistanceMap& vertex_distance_map,
                                                      const VertexFaceCornerMap& vertex_face_corner_map,
                                                      const FaceCornerMap& face_corner_map)
{
  FaceOrderRenderHeadNode* head_node = new FaceOrderRenderHeadNode(
                                              {vertices.at(face_vertex_map.at(base_face_id)[0]),
                                               vertices.at(face_vertex_map.at(base_face_id)[1]),
                                               vertices.at(face_vertex_map.at(base_face_id)[2])},
                                               base_face_id);

  for (auto const& [face, vertices] : face_vertex_map)
  {
    if (face == base_face_id) { continue; }
    
    const uint32_t face_closest_vertex_id = pce3d::maths::calculateClosestVertexOfFaceToOrigin(
                                              face_vertex_map.at(face), vertex_distance_map);
    // std::cout << "face: " << face << '\n';
    // std::cout << " getting face corner " << '\n';
    const glm::dvec3 face_corner = face_corner_map.at(vertex_face_corner_map.at(face_closest_vertex_id).at(face));
    // std::cout << " face corner created " << '\n';
    // const glm::dvec3 face_corner = face_corner_map.at(face_closest_corner_id);
    FaceOrderRenderNode* node = new FaceOrderRenderNode(face_corner, face);

    head_node->InsertWithPlaneComparison(node); 
  }
  std::vector<uint32_t> ordered_faces = head_node->GetListAtHeadNode();

  // for (auto const& face : ordered_faces) 
  // {
  //   std::cout << face << '\n';
  // }

  // std::cout << "deleting head node"  << '\n';
  delete head_node;
  // std::cout << "head node deleted"  << '\n';

  return ordered_faces;
}                                                    






}
}




#endif /* renderFunctions_cpp */
