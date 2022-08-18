#ifndef spaceMapFunctions_cpp
#define spaceMapFunctions_cpp

#include "../spaceMapFunctions.hpp"

namespace pce3d {
namespace space_map {


glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const glm::ivec3& mdim, const double mir) {
  glm::dvec3 p = point / mir;
  // std::cout << "~~~~~~~~~p: " << p.x << ", " << p.y << ", " << p.z << '\n';
  p = p + ((glm::dvec3(mdim.x, mdim.y, mdim.z) / mir) / 2.0);
  return glm::ivec3(p.x, p.y, p.z);
}


glm::dvec3 findPointOfIndex(const glm::ivec3& index, const glm::ivec3& mdim, const double mir) {
  glm::ivec3 p = index - ((mdim / int(mir)) / 2);
  // std::cout << "~~~~~~~~~p: " << p.x << ", " << p.y << ", " << p.z << '\n';
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


std::vector<glm::dvec3> orderVerticesByDistanceFromFirst(const std::vector<glm::dvec3>& vertices) {
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
          ordered_vertices.insert(ordered_vertices.begin()+j, vertices.begin()+i, vertices.begin()+i);
          has_been_added = true;
          continue;
      }
      if (has_been_added) { break; }
    }
    if (!has_been_added) { ordered_vertices.push_back(vertices[i]); std::cout << "pushing back" << '\n';}

    // std::cout << "ordered_vertices:" << '\n';
    // for (auto const& vertex : ordered_vertices) {
      // std::cout << "vertex: " << vertex.x << ", " << vertex.y << ", " << vertex.z << '\n';
    // }
  }
    // std::cout << "distances:" << '\n';
  // for (auto const& vertex : ordered_vertices) {
    // std::cout << distance_map.at(vertex) << '\n';
  // }
  // return ordered_vertices;
  return vertices;
}




std::vector<glm::ivec3> findRectFaceIndices(const std::vector<uint32_t>& face,
                                            const VertexMap& unordered_vertices, const glm::dvec3& mdim,
                                            const double mir) {
  std::vector<glm::dvec3> vertices = orderVerticesByDistanceFromFirst({
    unordered_vertices.at(face[0]), unordered_vertices.at(face[1]), unordered_vertices.at(face[2]), unordered_vertices.at(face[3]) });
  std::vector<glm::ivec3> indices{};

  const glm::dvec3 i_crawl_direction = glm::normalize(vertices[3] - vertices[0]); 
  const glm::dvec3 j_crawl_direction = glm::normalize(vertices[1] - vertices[0]);

  const double i_distance = sqrt(glm::dot(vertices[0] - vertices[3],
                                          vertices[0] - vertices[3]));
  const double j_distance = sqrt(glm::dot(vertices[0] - vertices[1],
                                          vertices[0] - vertices[1]));

  glm::dvec3 i_position = vertices[0];
  double i_dist_traveled = 0.0;
  while (i_dist_traveled <= i_distance) {
    double j_dist_traveled = 0.0;
    glm::dvec3 j_position = i_position; 
    while (j_dist_traveled <= j_distance) {
      indices.push_back(findIndexOfPoint(j_position, mdim, mir));
      j_position += j_crawl_direction; 
      j_dist_traveled += 1.0;
    }
    i_position = (i_position + i_crawl_direction);
    i_dist_traveled = (i_dist_traveled + 1.0);
  }

  return indices;
  
}



std::vector<glm::ivec3> findTriangleFaceIndices(const std::vector<uint32_t>& face,
                                                const VertexMap& vertices, const glm::dvec3& mdim,
                                                const double mir)
{
  std::vector<glm::ivec3> indices{};

  const double crawl_distance = 1.0;
  const glm::dvec3 crawl_start_vertex = vertices.at(face[0]);
  const glm::dvec3 sweep_start_vertex = vertices.at(face[1]);
  const glm::dvec3 sweep_end_vertex = vertices.at(face[2]);
  const glm::dvec3 sweep_direction = glm::normalize(sweep_end_vertex - sweep_start_vertex);

  glm::dvec3 current_sweep_point = sweep_start_vertex;
  double current_sweep_crawl_distance = 0.0;
  const double total_sweep_crawl_distance = pce3d::maths::calculateDistanceBetweenVectors(
                                                sweep_start_vertex, sweep_end_vertex);
  std::cout << "total_sweep_crawl_distance: " << total_sweep_crawl_distance << '\n';

  while (current_sweep_crawl_distance <= total_sweep_crawl_distance)
  {
    glm::dvec3 crawl_direction = glm::normalize(current_sweep_point - crawl_start_vertex);
    glm::dvec3 current_point = crawl_start_vertex;
    double current_crawl_distance = 0.0;
    const double total_crawl_distance = pce3d::maths::calculateDistanceBetweenVectors(
                                            current_sweep_point, crawl_start_vertex);
    
    while (current_crawl_distance <= total_crawl_distance)
    {
      indices.push_back(findIndexOfPoint(current_point, mdim, mir));
      current_point += crawl_direction * crawl_distance;
      current_crawl_distance += crawl_distance;
    }

    current_sweep_point += sweep_direction * crawl_distance;
    current_sweep_crawl_distance += crawl_distance;
  }

  return indices;
}




}
}


#endif /* spaceMapFunctions_cpp */
