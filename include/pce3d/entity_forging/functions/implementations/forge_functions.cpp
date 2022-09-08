#ifndef forge_functions_cpp
#define forge_functions_cpp

#include "../forge_functions.hpp"


namespace pce3d {
namespace forge {


void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity, bool vertices_normalized_to_object_center) {
  if (!vertices_normalized_to_object_center)
  {
    for (auto& [id, vertex] : vertices) {
      vertex = vertex - center_of_gravity;
      vertex = pce::rotateVector3byAngleAxis(vertex, angle, axis);
      vertex = vertex + center_of_gravity;
    }
  }
  else
  {
    for (auto& [id, vertex] : vertices) {
      vertex = pce::rotateVector3byAngleAxis(vertex, angle, axis);
    }
  }
}



void createFaceVertexCornerMaps(const VertexMap& vertices, const FaceVertexMap& faces,
                                FaceCornerMap& face_corner_map,
                                FaceVertexCornerMap& face_vertex_corner_map,
                                VertexFaceCornerMap& vertex_face_corner_map,
                                const glm::dvec3& center_point,
                                bool vertices_normalized_to_object_center) {
  uint32_t current_id = 1;
  const glm::dvec3 psuedo_center_point 
      = vertices_normalized_to_object_center ?  glm::dvec3(0, 0, 0) : center_point;
  for (auto const& [face, vertex_list] : faces) {
    for (size_t i = 0; i != vertex_list.size(); ++i) {
      const size_t index_neighbor_a = (i == 0) ? vertex_list.size()-1 : i - 1;
      const size_t index_neighbor_b = (i == vertex_list.size()-1) ? 0 : i + 1;

      const glm::dvec3 mvertex = vertices.at(vertex_list[i]);
      const glm::dvec3 connected_vertex_a = vertices.at(vertex_list[index_neighbor_a]); const glm::dvec3 connected_vertex_b = vertices.at(vertex_list[index_neighbor_b]);
      const glm::dvec3 face_corner_direction = glm::normalize((connected_vertex_a-mvertex + connected_vertex_b-mvertex));
      const glm::dvec3 face_corner_point = vertices.at(vertex_list[i]) + face_corner_direction + psuedo_center_point;
      
      
      face_corner_map[current_id] = face_corner_point;
      face_vertex_corner_map[face][vertex_list[i]] = current_id;
      vertex_face_corner_map[vertex_list[i]][face] = current_id;
      // std::cout << "current id: " <<current_id << '\n';
      ++current_id;
    }
  }
}



void createFaceEdgeMap(
    const FaceVertexMap& face_vertex_map
  , const EdgeMap& edge_map
  , FaceEdgeMap& face_edge_map
)
{
  for (auto const& [face_id, face_vertex_ids] : face_vertex_map)
  {
    size_t counter_face_vertices = face_vertex_ids.size();
    size_t i = 0;
    for (auto const& [edge_id, edge_vertex_ids] : edge_map)
    {
      if (std::find(face_vertex_ids.begin(), face_vertex_ids.end(), edge_vertex_ids.first) != face_vertex_ids.end()
       && std::find(face_vertex_ids.begin(), face_vertex_ids.end(), edge_vertex_ids.second) != face_vertex_ids.end())
      {
        face_edge_map[face_id].push_back(edge_id);
        ++i;
        if (i == counter_face_vertices) { break; }
      }
    }
  }
}


void createVertexEdgeMap(
    const VertexVertexMap& vertex_vertex_map
  , const EdgeMap& edge_map
  , FaceEdgeMap& vertex_edge_map
)
{
  for (auto const& [vertex_id, vertex_list] : vertex_vertex_map)
  {
    for (auto const& v : vertex_list)
    {
      for (auto const& [edge_id, vpair] : edge_map)
      {
        if ((vpair.first == v && vpair.second == vertex_id)
         || (vpair.first == vertex_id && vpair.second == v))
        {
          vertex_edge_map[vertex_id].push_back(edge_id);
        }
      }
    }
  }
}



}
}


#endif /* forge_functions_cpp */
