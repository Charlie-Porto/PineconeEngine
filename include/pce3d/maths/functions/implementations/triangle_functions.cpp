#ifndef triangle_functions_cpp
#define triangle_functions_cpp

#include "../triangle_functions.hpp"

namespace pce {
namespace maths {

std::vector<uint32_t> sortVerticesByDistance(const VertexMap& vertices, 
                                             const std::vector<uint32_t>& face) {
  double side_a_magnitude = sqrt(glm::dot(vertices.at(face[0]) - vertices.at(face[1]), 
                                          vertices.at(face[0]) - vertices.at(face[1])));
  double side_b_magnitude = sqrt(glm::dot(vertices.at(face[1]) - vertices.at(face[2]), 
                                          vertices.at(face[1]) - vertices.at(face[2])));
  double side_c_magnitude = sqrt(glm::dot(vertices.at(face[0]) - vertices.at(face[2]), 
                                          vertices.at(face[0]) - vertices.at(face[2])));

  std::vector<uint32_t> order{};

  double maximum = std::max({side_a_magnitude, side_b_magnitude, side_c_magnitude});

  if (maximum == side_a_magnitude) {
    order = {face[0], face[2], face[1]};
  }

  if (maximum == side_b_magnitude) {
    order = {face[1], face[0], face[2]};
  }

  if (maximum == side_c_magnitude) {
    order = {face[0], face[1], face[2]};
  }

  return order;
}
  


glm::dvec3 determineCrawlDirection(const VertexMap& vertices, 
                                   const std::vector<uint32_t>& sorted_face) {
  const glm::dvec3 longside_midpoint = (vertices.at(sorted_face[0]) - vertices.at(sorted_face[2])) / 2.0;
  return glm::normalize(vertices.at(sorted_face[1]) - longside_midpoint);
}
  


  
}
}


#endif /* triangle_functions_cpp */
