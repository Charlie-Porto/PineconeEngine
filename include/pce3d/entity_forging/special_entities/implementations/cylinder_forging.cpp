#ifndef cylinder_forging_cpp
#define cylinder_forging_cpp

#include "../cylinder_forging.hpp"

namespace pce3d {
namespace forge {

// const double PI = 3.14159265;
const extern double PI;

uint32_t forgeCylinder(
    const double h
  , const double r
  , const glm::dvec3& center
  , const std::vector<int>& color
  , const int sides
  , const double angle
  , const glm::dvec3 axis
)
{
  // 1: set up cylinder points 
  const glm::dvec3 center_point = center; 
  const glm::dvec3 upper_face_center_point = glm::dvec3(0, h/2.0, 0);
  const glm::dvec3 lower_face_center_point = glm::dvec3(0, -h/2.0, 0);

  std::vector<glm::dvec3> face_points{};
  std::vector<std::pair<uint32_t, uint32_t>> sidelines{};
  std::unordered_map<uint32_t, glm::dvec3> vertices;
  std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> edges;
  
  // calculate 'vertices'
  const double theta_incr = 360.0/static_cast<double>(sides); 
  uint32_t v_id = 1;
  uint32_t e_id = 1;
  for (double i = 0; i < static_cast<double>(sides); ++i)
  {
    const double theta = theta_incr * i;
    const double y = h/2.0;
    const double ny = -y;
    const double z = r * sin(theta / 180.0 * PI);
    const double x = r * cos(theta / 180.0 * PI);
    
    // calculate base face point
    glm::dvec3 fp = glm::dvec3(x, y, z);
    glm::dvec3 nfp = glm::dvec3(x, ny, z);
    // rotate
    fp = pce::rotateVector3byAngleAxis(fp, angle, axis);
    nfp = pce::rotateVector3byAngleAxis(nfp, angle, axis);
    // transform
    fp = center + fp;
    nfp = center + nfp;
    // add to vector
    face_points.push_back(fp);
    face_points.push_back(nfp);
    const uint32_t fp_id = v_id; 
    ++v_id;
    const uint32_t nfp_id = v_id; 
    ++v_id;
    vertices[fp_id] = fp;
    vertices[nfp_id] = nfp;
    sidelines.push_back(std::make_pair(fp_id, nfp_id));
    edges[e_id] = std::make_pair(fp_id, nfp_id);
    ++e_id;
  }


  // 2: create entity
  uint32_t entity = forgeBaseEntity(center, color, 1.0);
  control.AddComponent(entity, pce::RigidObject{
    .radius = r,
    .mass = 1000,
    .is_deadbod = true,
    .is_restingbod = false,
    .vertices = vertices,
    .edges = edges,
    .face_vertex_map = {{1, {1}}},
    .base_face_id = 1,
    .face_count = 1,
    .index_face_map = {},
    .face_index_map = {},
    .entity_face_collision_map = {},
    .entity_vertex_collision_map = {},
    .entity_index_collision_map = {}
  });
  control.AddComponent(entity, pce::Force{ .of_gravity = 0.0 });
  control.AddComponent(entity, pce::Motion{
    .speed = 0.0,
    .direction = glm::dvec3(0, 1, 0),
    .velocity = glm::dvec3(0, 1, 0),
    .rotational_speed = 0.0,
    .rotational_axis = glm::dvec3(0, 0, 0),
    .duration = 0.1,
    .previous_resting_position = center,
    .stationary_counter = 0
  });
  return entity;
}

}
}


#endif /* cylinder_forging_cpp */
