#ifndef forge_functions_cpp
#define forge_functions_cpp

#include "../forge_functions.hpp"


namespace pce3d {
namespace forge {


void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity) {
  for (auto& [id, vertex] : vertices) {
    vertex = vertex - center_of_gravity;
    vertex = pce::rotateVector3byAngleAxis(vertex, angle, axis);
    vertex = vertex + center_of_gravity;
  }
}

void createFaceVertexCornerMaps(const VertexMap& vertices, const FaceVertexMap& faces,
                                FaceCornerMap& face_corner_map,
                                FaceVertexCornerMap& face_vertex_corner_map,
                                VertexFaceCornerMap& vertex_face_corner_map,
                                const glm::dvec3& center_point) {
  uint32_t current_id = 1;
  for (auto const& [face, vertex_list] : faces) {
    for (size_t i = 0; i != vertex_list.size(); ++i) {
      const size_t index_neighbor_a = (i == 0) ? vertex_list.size()-1 : i - 1;
      const size_t index_neighbor_b = (i == vertex_list.size()-1) ? 0 : i + 1;

      const glm::dvec3 mvertex = vertices.at(vertex_list[i]);
      const glm::dvec3 connected_vertex_a = vertices.at(vertex_list[index_neighbor_a]);
      const glm::dvec3 connected_vertex_b = vertices.at(vertex_list[index_neighbor_b]);

      const glm::dvec3 face_corner_direction = glm::normalize((connected_vertex_a-mvertex + connected_vertex_b-mvertex));
      const glm::dvec3 face_corner_point = vertices.at(vertex_list[i]) + face_corner_direction + center_point;
      auto const A = face_corner_point;
      
      
      face_corner_map[current_id] = face_corner_point;
      face_vertex_corner_map[face][vertex_list[i]] = current_id;
      vertex_face_corner_map[vertex_list[i]][face] = current_id;
      std::cout << "current id: " <<current_id << '\n';
      ++current_id;
    }
  }
}

}
}


#endif /* forge_functions_cpp */
