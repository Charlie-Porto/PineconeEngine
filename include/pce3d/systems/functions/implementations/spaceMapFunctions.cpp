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



std::unordered_map<uint32_t, glm::ivec3> findIndicesGivenVerticesLabeled(const VertexMap& vertices, const glm::dvec3& mdim, const double mir)
{
  std::unordered_map<uint32_t, glm::ivec3> indices{};
  for (auto const& [id, vertex] : vertices) {
    indices[id] = findIndexOfPoint(vertex, mdim, mir);
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
      // assert(distance_map.find(ordered_vertices[j]) != distance_map.end());
      if (distance_map.find(ordered_vertices[j]) != distance_map.end())
      {
        if (distance < distance_map.at(ordered_vertices[j])) {
          ordered_vertices.insert(ordered_vertices.begin()+j, vertices.begin()+i, vertices.begin()+i);
          has_been_added = true;
          continue;
        }
      }
      if (has_been_added) { break; }
    }
    if (!has_been_added) { ordered_vertices.push_back(vertices[i]);}

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
    unordered_vertices.at(face[0]), 
    unordered_vertices.at(face[1]), 
    unordered_vertices.at(face[2]), 
    unordered_vertices.at(face[3]) 
  });
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
  // std::cout << "total_sweep_crawl_distance: " << total_sweep_crawl_distance << '\n';

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

std::vector<glm::ivec3> findFaceIndicesGeneral(
    const uint32_t entity
  , const uint32_t face
  , pce::RigidObject& rigid_object
  , const glm::ivec3& mdim
  , const double mir
)
{
  std::vector<glm::ivec3> face_indices{};
  assert(rigid_object.face_vertex_map.find(face) != rigid_object.face_vertex_map.end());
  const size_t size = rigid_object.face_vertex_map.at(face).size();
  switch (size) 
  {
    case 1:
      face_indices = {space_map::findIndexOfPoint(rigid_object.vertices.at(1), mdim, mir)};
      // std::cout << "ONLY ONE FACE INDEX: "
                // << face_indices[0].x << ", "
                // << face_indices[0].y << ", "
                // << face_indices[0].z << '\n';
                 
      break;
    case 4: 
      face_indices = space_map::findRectFaceIndices(rigid_object.face_vertex_map.at(face),
                                                    rigid_object.vertices,
                                                    mdim,
                                                    mir);
      break;
    case 3:
      face_indices = space_map::findTriangleFaceIndices(rigid_object.face_vertex_map.at(face),
                                                        rigid_object.vertices,
                                                        mdim,
                                                        mir);
      break;
    default:
      break;
  }

  return face_indices;
}



std::unordered_map<uint32_t, glm::ivec3> updateBodVertexMap(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& bod_vertex_map_ 
  , const glm::ivec3 mdim
  , const double mir
)
{
  const std::unordered_map<uint32_t, glm::ivec3> labeled_vertex_indices 
    = space_map::findIndicesGivenVerticesLabeled(rigid_object.vertices, mdim, mir);
  for (auto const& [id, index] : labeled_vertex_indices) 
  {
    bod_vertex_map_[index] = std::unordered_map<uint32_t, uint32_t>{{entity, id}};
  }

  return labeled_vertex_indices;
}



std::unordered_map<uint32_t, std::vector<glm::ivec3>> updateBodEdgeMap(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& bod_edge_map_ 
  , const glm::ivec3 mdim
  , const double mir
)
{
  std::unordered_map<uint32_t, std::vector<glm::ivec3>> edge_indices_map{};

  for (auto const& [edge, vpair] : rigid_object.edges)
  {
    const uint32_t& start_vertex_id = vpair.first;
    const uint32_t& end_vertex_id = vpair.second;

    const double distance = pce3d::maths::calculateDistanceBetweenVectors(
      rigid_object.vertices.at(start_vertex_id), rigid_object.vertices.at(end_vertex_id)
    );

    const glm::dvec3 crawl_direction 
      = glm::normalize((rigid_object.vertices.at(start_vertex_id) - rigid_object.vertices.at(end_vertex_id)));
    double i = 0.0; 

    while (i < distance)
    {
      const glm::dvec3 current_point = rigid_object.vertices.at(start_vertex_id) - crawl_direction * i;
      const glm::ivec3 current_point_index = findIndexOfPoint(current_point, mdim, mir);
      edge_indices_map[edge].push_back(current_point_index);

      bod_edge_map_[current_point_index][entity] = edge;

      i += pce3d::Core3D::COLLISION_METER_INDEX_RATIO;
    }

  }

  return edge_indices_map;
}



void updateLiveBodIndicesAndCheckForLiveBodCollision(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , const glm::ivec3 mdim
  , const double mir
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& livebod_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_vertex_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_edge_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_index_face_map
  , std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& potential_collision_entity_map
  , std::unordered_map<uint32_t, glm::ivec3>& potential_collision_index_map
  , uint32_t& next_id
)
{
  assert(!rigid_object.face_vertex_map.empty());
  for (auto const& [face, vertex_ids] : rigid_object.face_vertex_map)
  {
    std::vector<glm::ivec3> face_indices = space_map::findFaceIndicesGeneral(
      entity,
      face, 
      rigid_object,
      mdim, mir
    );
    assert(!face_indices.empty());
    for (auto const& index : face_indices)
    {
      // std::cout << "index: " << index.x << ", " << index.y << ", " << index.z << '\n';

      rigid_object.index_face_map[index] = face;
      rigid_object.face_index_map[face] = index;

      size_t points = 0;
      /* check if this index has been occupied yet */
      if (livebod_map.find(index) == livebod_map.end()) 
      {
        livebod_map[index] = {entity};

        if (livebod_vertex_map.find(index) != livebod_vertex_map.end())
        {
          if (livebod_vertex_map.at(index).find(entity) == livebod_vertex_map.at(index).end())
          {
            ++points;
          }
        }
        else if (livebod_edge_map.find(index) != livebod_edge_map.end())
        {
          if (livebod_edge_map.at(index).find(entity) == livebod_edge_map.at(index).end())
          {
            ++points;
          }
        }
        if (points == 2)
        {
          livebod_index_face_map[index][entity] = face;
        }
      } 
      /* else if index is occupied, check if occupied by this entity or another */
      else 
      {
        assert(livebod_map.find(index) != livebod_map.end()); 
      }
      if (!std::count(livebod_map.at(index).begin(), livebod_map.at(index).end(), entity))
      {
        const uint32_t& other_entity = livebod_map.at(index)[0];
        auto& other_rigid_object = control.GetComponent<pce::RigidObject>(other_entity);

        // std::cout << "LOGGING LIVEBOD COLLISION BETWEEN ENTITIES:" << entity << ", " << other_entity << '\n';
        rigid_object.entity_index_collision_map[other_entity] = {index};
        other_rigid_object.entity_index_collision_map[entity] = {index};

        other_rigid_object.is_restingbod = false;
        assert(livebod_map.at(index).size() >= 1);
        livebod_map.at(index).push_back(entity); 
        assert(livebod_map.at(index).size() >= 2);
        potential_collision_entity_map[next_id] = std::make_pair(entity, other_entity);
        potential_collision_index_map[next_id] = index;
        ++next_id;

        bool original_entity_done = false;
        bool other_entity_done = false;

        /* check if entity index point is a vertex and if so log it as such */
        if (livebod_vertex_map.find(index) != livebod_vertex_map.end())
        {
          if (livebod_vertex_map.at(index).find(entity) != livebod_vertex_map.at(index).end())
          {
            rigid_object.entity_vertex_collision_map[other_entity] 
              = livebod_vertex_map.at(index).at(entity);
            original_entity_done = true;
          }
        }
        if (!original_entity_done)
        {
          rigid_object.entity_face_collision_map[other_entity] = face;
        }

        /* check if other_entity index point is a vertex and if so log it as such */
        if (livebod_vertex_map.find(index) != livebod_vertex_map.end())
        {
          if (livebod_vertex_map.at(index).find(other_entity) != livebod_vertex_map.at(index).end())
          {
            other_rigid_object.entity_vertex_collision_map[entity] 
              = livebod_vertex_map.at(index).at(other_entity);
            other_entity_done = true;
          }
        }
        if (!other_entity_done)
        {
          other_rigid_object.entity_face_collision_map[entity] = face;
        }

      }
    }
  }
}



void doPreLoopMapUpdate(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& bod_map_
  , const glm::ivec3 mdim
  , const double mir
)
{

  for (auto const& [face, vertex_ids] : rigid_object.face_vertex_map) {
        
    std::vector<glm::ivec3> face_indices = pce3d::space_map::findFaceIndicesGeneral(
      entity,
      face,
      rigid_object,
      mdim,
      mir
    );
    for (auto const& index : face_indices) {
      rigid_object.index_face_map[index] = face;
      rigid_object.face_index_map[face] = index;
      bod_map_[index].push_back(entity);
    }
  }
}


void checkForCollisionWithNonLiveBods(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<uint32_t, glm::ivec3>& vertex_indices
  , const glm::ivec3 mdim
  , const double mir
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& deadbod_map
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& restingbod_map
  , std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& potential_collision_entity_map
  , std::unordered_map<uint32_t, glm::ivec3>& potential_collision_index_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_vertex_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_edge_map
  , uint32_t& next_id
)
{
  for (auto const& [id, index] : vertex_indices)
  {
    if (restingbod_map.find(index) != restingbod_map.end()) 
    {
      uint32_t other_entity = restingbod_map.at(index)[0];
      potential_collision_entity_map[next_id] = std::make_pair(entity, other_entity);
      potential_collision_index_map[next_id] = index;
      ++next_id;
      auto& other_rigid_object = control.GetComponent<pce::RigidObject>(other_entity);
      other_rigid_object.is_restingbod = false;
      continue;
    }
    else if (deadbod_map.find(index) != deadbod_map.end()) 
    {
      // std::cout << "SPACEMAP: DETECTED DEADBOD MAP COLLISION" << '\n';
      uint32_t deadbod_entity = deadbod_map.at(index)[0];
      potential_collision_entity_map[next_id] = std::make_pair(entity, deadbod_entity);
      potential_collision_index_map[next_id] = index;
      ++next_id;
      auto& deadbod_rigid_object = control.GetComponent<pce::RigidObject>(deadbod_entity);
      deadbod_rigid_object.entity_face_collision_map[entity] = deadbod_rigid_object.index_face_map.at(index);

      if (livebod_vertex_map.find(index) != livebod_vertex_map.end()
      && livebod_vertex_map.at(index).find(entity) != livebod_vertex_map.at(index).end())
      {
        rigid_object.entity_vertex_collision_map[deadbod_entity] = livebod_vertex_map.at(index).at(entity);
      }
      else if (livebod_edge_map.find(index) != livebod_edge_map.end()
      && livebod_edge_map.at(index).find(entity) != livebod_edge_map.at(index).end())
      {
        rigid_object.entity_edge_collision_map[deadbod_entity] = livebod_edge_map.at(index).at(entity);
      }
      else
      {
        rigid_object.entity_face_collision_map[deadbod_entity] = rigid_object.index_face_map.at(index);
      }
    }
  }
}


}
}


#endif /* spaceMapFunctions_cpp */
