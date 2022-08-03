#ifndef spaceMapFunctions_cpp
#define spaceMapFunctions_cpp

#include "../spaceMapFunctions.hpp"

namespace pce3d {
namespace space_map {


glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const glm::ivec3& mdim, const double mir) {
  glm::dvec3 p = point / mir;
  p = p + ((glm::dvec3(mdim.x, mdim.y, mdim.z) / mir) / 2.0);
  return glm::ivec3(p.x, p.y, p.z);
}


glm::dvec3 findPointOfIndex(const glm::ivec3& index, const glm::ivec3& mdim, const double mir) {
  glm::ivec3 p = index - ((mdim / int(mir)) / 2);
  p = p * int(mir);
  return glm::dvec3(p.x, p.y, p.z);
}


std::vector<glm::ivec3> findIndicesGivenVertices(const VertexMap& vertices, const glm::dvec3& mdim, const double mir) {
  std::vector<glm::ivec3> indices{};
  for (auto const& [id, vertex] : vertices) {
    indices.push_back(findIndexOfPoint(vertex, mdim, mir));
  }
  return indices;
}


// std::vector<glm::ivec3> findIndicesOfFaceMidpoints(const std::vector<uint32_t>& face,
//                                                    const VertexMap& vertices, const glm::dvec3& mdim, 
//                                                    const double mir) {
  
// }


std::vector<glm::ivec3> findFaceIndices(const std::vector<uint32_t>& face,
                                        const VertexMap& vertices, const glm::dvec3& mdim,
                                        const double mir) {
  std::vector<glm::ivec3> face_indices{};
  switch (face.size()) {
    case 3:
      break;
    case 4:
      break;
    default:
      break;
  }
  return face_indices;
}


// std::vector<glm::ivec3> findTriangleFaceIndices(const std::vector<uint32_t>& face,
//                                                 const VertexMap& vertices, const glm::dvec3& mdim,
//                                                 const double mir) {
//   std::vector<glm::ivec3> tri_face_indices{};
//   const std::vector<uint32_t> sorted_face = pce::maths::sortVerticesByDistance(vertices, face);
//   const glm::dvec3 face_crawl_direction = pce::maths::determineCrawlDirection(vertices, sorted_face);
//   const glm::dvec3 line_crawl_direction = glm::normalize(vertices.at(sorted_face[0]) 
//                                                        - vertices.at(sorted_face[2]));
  
//   glm::dvec3 current_line_point = vertices.at(sorted_face[2]);
//   /* PICK UP HERE */
// }

std::vector<glm::dvec3> orderVerticesByDistanceFromFirst(const std::vector<glm::dvec3>& vertices) {
  std::cout << "v0" << vertices[0].x << ", " << vertices[0].y << ", " << vertices[0].z << '\n';
  std::cout << "v1" << vertices[1].x << ", " << vertices[1].y << ", " << vertices[1].z << '\n';
  std::cout << "v2" << vertices[2].x << ", " << vertices[2].y << ", " << vertices[2].z << '\n';
  std::cout << "v3" << vertices[3].x << ", " << vertices[3].y << ", " << vertices[3].z << '\n';
  std::vector<glm::dvec3> ordered_vertices{vertices[0]};
  std::unordered_map<glm::dvec3, double> distance_map{};
  distance_map[ordered_vertices[0]] = 0.0;

  for (size_t i = 0; i < vertices.size(); ++i) { 
    if (i == 0) { continue; }
    const double distance = sqrt(glm::dot(vertices[i] - vertices[0], vertices[i] - vertices[0]));
    distance_map[vertices[i]] = distance;
    bool has_been_added = false; 

    size_t smaller_size = std::min(vertices.size(), ordered_vertices.size());

    for (size_t j = 0; j < smaller_size; ++j) {
      if (distance < distance_map.at(ordered_vertices[j])) {
        std::cout << "j: " << j << '\n';
        if (i == vertices.size()-1) {
          std::cout << "i equals vertices.size() - 1" << '\n';
          auto d = ordered_vertices.insert(ordered_vertices.begin()+j, vertices.begin()+i, vertices.end());
          has_been_added = true;
          continue;
        } 
        else {
          ordered_vertices.insert(ordered_vertices.begin()+j, vertices.begin()+i, vertices.begin()+i);
          has_been_added = true;
          continue;
        }
      }
      if (has_been_added) { break; }
    }
    if (!has_been_added) { ordered_vertices.push_back(vertices[i]); std::cout << "pushing back" << '\n';}

    std::cout << "ordered_vertices:" << '\n';
    for (auto const& vertex : ordered_vertices) {
       std::cout << "vertex: " << vertex.x << ", " << vertex.y << ", " << vertex.z << '\n';
    }
  }
    std::cout << "distances:" << '\n';
  for (auto const& vertex : ordered_vertices) {
    std::cout << distance_map.at(vertex) << '\n';
  }
  // return ordered_vertices;
  return vertices;
}




std::vector<glm::ivec3> findRectFaceIndices(const std::vector<uint32_t>& face,
                                            const VertexMap& unordered_vertices, const glm::dvec3& mdim,
                                            const double mir) {
  std::vector<glm::dvec3> vertices = orderVerticesByDistanceFromFirst({
    unordered_vertices.at(face[0]), unordered_vertices.at(face[1]), unordered_vertices.at(face[2]), unordered_vertices.at(face[3]) });
  std::vector<glm::ivec3> indices{};
  // std::cout << "v0" << vertices[0].x << ", " << vertices[0].y << ", " << vertices[0].z << '\n';
  // std::cout << "v1" << vertices[1].x << ", " << vertices[1].y << ", " << vertices[1].z << '\n';
  // std::cout << "v2" << vertices[2].x << ", " << vertices[2].y << ", " << vertices[2].z << '\n';
  // std::cout << "v3" << vertices[3].x << ", " << vertices[3].y << ", " << vertices[3].z << '\n';

  const glm::dvec3 i_crawl_direction = glm::normalize(vertices[3] - vertices[0]); 
  const glm::dvec3 j_crawl_direction = glm::normalize(vertices[1] - vertices[0]);
  // const glm::dvec3 i_crawl_direction = glm::normalize(vertices[0] - vertices[1]); 
  // const glm::dvec3 j_crawl_direction = glm::normalize(vertices[0] - vertices[2]);

  const double i_distance = sqrt(glm::dot(vertices[0] - vertices[3],
                                          vertices[0] - vertices[3]));
  const double j_distance = sqrt(glm::dot(vertices[0] - vertices[1],
                                          vertices[0] - vertices[1]));
  std::cout << "j_distance: "  << j_distance << '\n';
  std::cout << "i_distance: "  << i_distance << '\n';

  glm::dvec3 i_position = vertices[0];
  double i_dist_traveled = 0.0;
  while (i_dist_traveled <= i_distance) {
    double j_dist_traveled = 0.0;
    glm::dvec3 j_position = i_position; 
    while (j_dist_traveled <= j_distance) {
    // while (j_dist_traveled <= 1) {
      indices.push_back(findIndexOfPoint(j_position, mdim, mir));
      j_position += j_crawl_direction; 
      j_dist_traveled += 1.0;
    }
    // i_position += i_crawl_direction;  
    // i_dist_traveled += 1.0;
    i_position = (i_position + i_crawl_direction);
    i_dist_traveled = (i_dist_traveled + 1.0);
  }

  return indices;
  
}



}
}


#endif /* spaceMapFunctions_cpp */
