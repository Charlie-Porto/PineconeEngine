#ifndef renderFunctions_cpp
#define renderFunctions_cpp

#include <vector>
#include <utility>
#include "../renderFunctions.hpp"


namespace pce3d {
namespace render {


std::vector<std::pair<uint32_t, double>> orderFacesByCameraProximity(
    const FaceVertexMap& face_vertex_map,
    const VertexDistanceMap& vertex_distance_map) {
  std::vector<std::pair<uint32_t, double>> faces_furthest_to_closest{};
  std::vector<double> averages{};
  double closest_face = 6;
  for (auto const& [face, vertices] : face_vertex_map) {
    
    const double average_vertex = (vertex_distance_map.at(vertices[0])
                                + vertex_distance_map.at(vertices[1])
                                + vertex_distance_map.at(vertices[2])
                                + vertex_distance_map.at(vertices[3]))
                                / 4.0;

    const double closest_vertex = *std::min_element(vertices.begin(), vertices.end());
    if (faces_furthest_to_closest.size() == 0) { 
      faces_furthest_to_closest.push_back(std::make_pair(face, closest_vertex)); 
      averages.push_back(average_vertex);
      std::cout << "first added: " << face << '\n';
      continue;
    }
    else {
      size_t i = 0;
      while (i < faces_furthest_to_closest.size()+1) {
        if (i == faces_furthest_to_closest.size()) {
          faces_furthest_to_closest.push_back(std::make_pair(face, closest_vertex));
          averages.push_back(average_vertex);
          closest_face = face;
          break;
        }
        if (closest_vertex == faces_furthest_to_closest[i].second) {
          if (average_vertex > averages[i]) {
            faces_furthest_to_closest.insert(faces_furthest_to_closest.begin()+i, std::make_pair(face, closest_vertex));
            averages.insert(averages.begin()+i, average_vertex);
            closest_face = face;
            break;
          }
        }
        if (closest_vertex > faces_furthest_to_closest[i].second) {
          if (average_vertex > averages[i]) {
            faces_furthest_to_closest.insert(faces_furthest_to_closest.begin()+i, std::make_pair(face, closest_vertex));
            averages.insert(averages.begin()+i, average_vertex);
            closest_face = face;
            break;
          }
        }
        ++i;
      }
    }
  }
  return faces_furthest_to_closest;
}

}
}




#endif /* renderFunctions_cpp */
