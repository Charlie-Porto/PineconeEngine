#ifndef massMapFunctions_cpp
#define massMapFunctions_cpp

#include "../massMapFunctions.hpp"


namespace pce3d {
namespace massmap {

const double zone_length = pce3d::Core3D::MASS_ZONE_SIDE_LENGTH_METERS;


std::vector<glm::dvec3> findRectFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
)
{
  std::vector<glm::dvec3> zones{};

  std::vector<glm::dvec3> ordered_vertices = pce3d::space_map::orderVerticesByDistanceFromFirst({
    vertices.at(faces.at(face_id)[0]), 
    vertices.at(faces.at(face_id)[1]), 
    vertices.at(faces.at(face_id)[2]), 
    vertices.at(faces.at(face_id)[3]) 
  });

  const glm::dvec3 i_crawl_direction = glm::normalize(ordered_vertices[3] - ordered_vertices[0]); 
  const glm::dvec3 j_crawl_direction = glm::normalize(ordered_vertices[1] - ordered_vertices[0]);
  const double i_distance = sqrt(glm::dot(ordered_vertices[0] - ordered_vertices[3],
                                          ordered_vertices[0] - ordered_vertices[3]));
  const double j_distance = sqrt(glm::dot(ordered_vertices[0] - ordered_vertices[1],
                                          ordered_vertices[0] - ordered_vertices[1]));
  
  glm::dvec3 i_position = ordered_vertices[0];
  double i_dist_traveled = 0.0;
  while (i_dist_traveled <= i_distance) 
  {
    double j_dist_traveled = 0.0;
    glm::dvec3 j_position = i_position; 
    while (j_dist_traveled <= j_distance) 
    {
      const glm::dvec3 rounded_position = pce3d::round::roundVec3ComponentsToNearestInterval(
        zone_length, j_position);

      zones.push_back(rounded_position);
      j_position += j_crawl_direction * zone_length; 
      j_dist_traveled += zone_length;
    }
    i_position += i_crawl_direction * zone_length;
    i_dist_traveled = (i_dist_traveled + zone_length);
  }

  return zones;
}



std::vector<glm::dvec3> findTriFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
)
{
  std::vector<glm::dvec3> zones{};

  const glm::dvec3 crawl_start_vertex = vertices.at(faces.at(face_id)[0]);
  const glm::dvec3 sweep_start_vertex = vertices.at(faces.at(face_id)[1]);
  const glm::dvec3 sweep_end_vertex = vertices.at(faces.at(face_id)[2]);

  const glm::dvec3 sweep_direction = glm::normalize(sweep_end_vertex - sweep_start_vertex);

  glm::dvec3 current_sweep_point = sweep_start_vertex;
  double current_sweep_crawl_distance = 0.0;
  const double total_sweep_crawl_distance = pce3d::maths::calculateDistanceBetweenVectors(
                                                sweep_start_vertex, sweep_end_vertex);

  while (current_sweep_crawl_distance <= total_sweep_crawl_distance)
  {
    glm::dvec3 crawl_direction = glm::normalize(current_sweep_point - crawl_start_vertex);
    glm::dvec3 current_point = crawl_start_vertex;
    double current_crawl_distance = 0.0;
    const double total_crawl_distance = pce3d::maths::calculateDistanceBetweenVectors(
                                            current_sweep_point, crawl_start_vertex);
    
    while (current_crawl_distance <= total_crawl_distance)
    {
      const glm::dvec3 rounded_position = pce3d::round::roundVec3ComponentsToNearestInterval(
        zone_length, current_point);
      zones.push_back(rounded_position);
      current_point += crawl_direction * zone_length;
      current_crawl_distance += zone_length;
    }

    current_sweep_point += sweep_direction * zone_length;
    current_sweep_crawl_distance += zone_length;
  }

  return zones;
}



std::vector<glm::dvec3> findFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
)
{
  std::vector<glm::dvec3> zones{};
  assert(faces.at(face_id).size() == 3 || faces.at(face_id).size() == 4);
  switch (faces.at(face_id).size())
  {
    case 3:
      zones = findTriFaceMassZones(face_id, faces, vertices);
      break;
    case 4:
      zones = findRectFaceMassZones(face_id, faces, vertices);
      break;
    default:
      break;
  }

  return zones;
}



std::vector<glm::dvec3> findAllSurfaceMassZones(
    const FaceVertexMap& faces
  , const VertexMap& vertices
)
{
  std::vector<glm::dvec3> surface_zones{};
  for (auto const& [face_id, vertex_list] : faces)
  {
    const std::vector<glm::dvec3> zones = findFaceMassZones(
      face_id, faces, vertices);
    
    surface_zones.insert(surface_zones.end(), zones.begin(), zones.end());
  }

  return surface_zones;
}



void findAllMassZonesGivenSurfaceZones(
    const glm::dvec3& center_of_mass
  , const std::vector<glm::dvec3>& face_mass_zones
  , std::unordered_map<uint32_t, glm::dvec3>& mass_zone_map 
  , std::unordered_map<glm::dvec3, uint32_t>& coordinate_zone_map 
  , std::unordered_map<uint32_t, double>& mass_zone_distance_map 
)
{
  uint32_t id_counter = 1;
  for (auto const& zone : face_mass_zones)
  {
    /* first, map face point and get crawl distance */
    const glm::dvec3 rounded_position = pce3d::round::roundVec3ComponentsToNearestInterval(
      zone_length, zone);
    mass_zone_map[id_counter] = rounded_position;
    coordinate_zone_map[rounded_position] = id_counter;

    const glm::dvec3 crawl_direction = glm::normalize(center_of_mass - zone);
    const double total_crawl_distance = pce3d::maths::calculateDistanceBetweenVectors(zone, center_of_mass); 
    mass_zone_distance_map[id_counter] = total_crawl_distance;
    
    double current_crawl_distance = zone_length;
    ++id_counter;

    /* then, crawl inward until reach the center of mass or an already-logged point */
    while (current_crawl_distance <= total_crawl_distance)
    {
      glm::dvec3 current_crawl_point = zone + (crawl_direction * current_crawl_distance);
      /* if coordinate already mapped, stop mapping */
      if (coordinate_zone_map.find(current_crawl_point) != coordinate_zone_map.end())
      {
        break;
      }
       
      const glm::dvec3 rounded_position = pce3d::round::roundVec3ComponentsToNearestInterval(
        zone_length, current_crawl_point);

      mass_zone_map[id_counter] = rounded_position;
      coordinate_zone_map[rounded_position] = id_counter;
      
      ++id_counter;
      current_crawl_distance += zone_length;
    }
  }
}



}
}

#endif /* massMapFunctions_cpp */
