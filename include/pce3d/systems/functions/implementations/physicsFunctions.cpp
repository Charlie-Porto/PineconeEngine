#ifndef physicsFunctions_cpp
#define physicsFunctions_cpp

#include "../physicsFunctions.hpp"


namespace pce3d {
namespace physics {

glm::dvec3 calculateParticlePositionGivenTime(
    const glm::dvec3& initial_position, const glm::dvec3& initial_velocity, 
    double time_change, double gravitational_force_applied,
    double& duration) 
{
  duration += time_change;
  glm::dvec3 path_traveled = initial_velocity * duration;   
  path_traveled.y += (0.5 * pow(duration, 2.0) * GRAVITY * gravitational_force_applied);
  return glm::dvec3(initial_position + path_traveled);
}



bool determineIfParticlesAreColliding(
    const glm::dvec3& a_position, const double a_radius,
    const glm::dvec3& b_position, const double b_radius)
{
  const double total_radius = a_radius + b_radius;
  const glm::dvec3 diff_vect = a_position - b_position;
  const double centers_distance = sqrt(glm::dot(diff_vect, diff_vect));
  return total_radius > centers_distance ? true : false;
}



std::pair<glm::dvec3, glm::dvec3> calculateVelocityVectorsAfterTwoParticleCollision(
    const glm::dvec3& a_center, const double a_radius, const glm::dvec3& a_velocity_vect, const double a_mass,
    const glm::dvec3& b_center, const double b_radius, const glm::dvec3& b_velocity_vect, const double b_mass) 
{
  
  const glm::dvec3 a_hitpoint_direction = glm::normalize(b_center - a_center);
  const glm::dvec3 b_hitpoint_direction = glm::normalize(a_center - b_center);
  // std::cout << "a_velocity_vect: "<< a_velocity_vect.x << ", " << a_velocity_vect.y << ", " << a_velocity_vect.z << '\n';
  // std::cout << "b_velocity_vect: "<< b_velocity_vect.x << ", " << b_velocity_vect.y << ", " << b_velocity_vect.z << '\n';

  const double a_velocity_magnitude = sqrt(glm::dot(a_velocity_vect, a_velocity_vect));
  const double b_velocity_magnitude = sqrt(glm::dot(b_velocity_vect, b_velocity_vect));

  const glm::dvec3 a_magnitude_in_a_direction = a_hitpoint_direction * a_velocity_magnitude * a_mass;
  const glm::dvec3 b_magnitude_in_b_direction = b_hitpoint_direction * b_velocity_magnitude * b_mass;

  return std::make_pair(a_magnitude_in_a_direction, b_magnitude_in_b_direction);
}



void updateBothEntityInfoAfterTwoParticleCollision(
    const glm::dvec3& a_center, const double a_radius, pce::Motion& a_motion, const double a_mass,
    const glm::dvec3& b_center, const double b_radius, pce::Motion& b_motion, const double b_mass)
{
  std::pair<glm::dvec3, glm::dvec3> new_velocity_vectors 
    = physics::calculateVelocityVectorsAfterTwoParticleCollision(
        a_center, a_radius, a_motion.direction * a_motion.speed, a_mass,
        b_center, b_radius, b_motion.direction * b_motion.speed, b_mass);
  
  std::cout << "former_a_velocity: "<< a_motion.velocity.x << ", " << a_motion.velocity.y << ", " << a_motion.velocity.z << '\n';
  a_motion.velocity = new_velocity_vectors.second / a_mass;
  // a_motion.velocity += (new_velocity_vectors.second) / a_mass;
  std::cout << "new_a_velocity: "<< a_motion.velocity.x << ", " << a_motion.velocity.y << ", " << a_motion.velocity.z << '\n';
  a_motion.direction = glm::normalize(a_motion.velocity);
  // if (isnan(a_motion.direction.x) || isnan(a_motion.direction.y) || isnan(a_motion.direction.y)) {
    // a_motion.direction = a_motion.velocity;
  // }
  std::cout << "former_b_velocity: "<< b_motion.velocity.x << ", " << b_motion.velocity.y << ", " << b_motion.velocity.z << '\n';
  // b_motion.velocity += (new_velocity_vectors.first + new_velocity_vectors.second);
  b_motion.velocity = new_velocity_vectors.first / b_mass;
  // b_motion.velocity += (new_velocity_vectors.second) / b_mass;
  std::cout << "new_b_velocity: "<< b_motion.velocity.x << ", " << b_motion.velocity.y << ", " << b_motion.velocity.z << '\n';
  // b_motion.direction = glm::normalize(b_motion.velocity);
  // if (isnan(b_motion.direction.x) || isnan(b_motion.direction.y) || isnan(b_motion.direction.y)) {
    // b_motion.direction = b_motion.velocity;
  // }

  a_motion.previous_resting_position = a_center;
  b_motion.previous_resting_position = b_center;
  a_motion.duration = 0.0;
  b_motion.duration = 0.0;
}



bool determineIfParticleIsCollidingWithFace(
    const glm::dvec3& p_center, const double p_radius, 
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices) 
{

  pce3d::maths::PlaneCartesianForm face_plane = pce3d::maths::calculatePlaneGiven3Points(face_vertices[0], 
                                                                                         face_vertices[1], 
                                                                                         face_vertices[2]);
  const double distance = pce3d::maths::calculateDistanceBetweenPointAndPlane(face_plane, p_center);
  // std::cout << "distance" << distance << '\n';
  // std::cout << "radius" << p_radius << '\n';
  
  return (distance < p_radius) ? true : false;
}



glm::dvec3 calculateVelocityVectorAfterLiveParticleDeadFaceCollision(
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices, double elasticity) 
{
  /* pick up here */ 
  glm::dvec3 new_velocity_vector = p_velocity_vect;
  // std::cout << "original velocity: "
  //           << new_velocity_vector.x << ", " 
  //           << new_velocity_vector.y << ", " 
  //           << new_velocity_vector.z << '\n';

  glm::dvec3 normal_vec = glm::normalize(glm::cross(face_vertices[0] - face_vertices[1], 
                                                    face_vertices[2] - face_vertices[1]));

  const double collision_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    p_velocity_vect, normal_vec
  );

  elasticity = std::min(1.0, elasticity * (90.0 / collision_angle));

  // std::cout << "normal_vec: "
            // << normal_vec.x << ", " 
            // << normal_vec.y << ", " 
            // << normal_vec.z << '\n';
  const glm::dvec3 reverse_velocity_vect = -p_velocity_vect;
  new_velocity_vector = pce::rotateVector3byAngleAxis(reverse_velocity_vect, 180.0, normal_vec) * elasticity;
  // std::cout << "new velocity: "
  //           << new_velocity_vector.x << ", " 
  //           << new_velocity_vector.y << ", " 
  //           << new_velocity_vector.z << '\n';
  return new_velocity_vector;
}



glm::dvec3 calculateStartPositionAfterLiveParticleDeadFaceCollision(
    const glm::dvec3 A
  , const glm::dvec3 B
  , const glm::dvec3 C
  , const glm::dvec3 plane_side_direction
  , const glm::dvec3 particle_position
  , const double particle_radius)
{
  const glm::dvec3 normal_vector = pce3d::maths::calculateNormalVectorInDirectionOfPoint(A, B, C, plane_side_direction);
  const glm::dvec3 plane_point = pce3d::maths::calculateClosestPointInPlaneToPoint(A, B, C, particle_position);
  return plane_point + particle_radius * normal_vector;
}



void updateLiveParticleInfoAfterDeadFaceCollision(
    const glm::dvec3& p_center, const double p_radius, 
    const double mass, pce::Motion& motion,
    const std::vector<glm::dvec3>& face_vertices, double elasticity) 
{
  const glm::dvec3 nvelocity = physics::calculateVelocityVectorAfterLiveParticleDeadFaceCollision( 
      motion.direction * motion.speed, mass, face_vertices, elasticity);
  
  motion.velocity = nvelocity;
  // std::cout << "nvelocity: "
  //           << nvelocity.x << ", " 
  //           << nvelocity.y << ", " 
  //           << nvelocity.z << '\n';
  motion.direction = glm::normalize(nvelocity);
  motion.previous_resting_position 
      = calculateStartPositionAfterLiveParticleDeadFaceCollision(face_vertices[0],
                                                                 face_vertices[1],
                                                                 face_vertices[2],
                                                                 p_center + nvelocity,
                                                                 p_center,
                                                                 p_radius);

  motion.duration = 0.0;
  motion.speed = sqrt(glm::dot(motion.velocity, motion.velocity));
}



std::pair<std::pair<bool, glm::dvec3>, std::pair<bool, glm::dvec3>> 
calculateLiveBodHitPointsAndIfVertex(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::RigidObject& b_rigid_object
  , pce::Position& a_position
  , pce::Position& b_position
)
{
  glm::dvec3 a_hit_point = a_position.actual_center_of_mass;
  glm::dvec3 b_hit_point = b_position.actual_center_of_mass;
  std::pair<bool, glm::dvec3> a_pair = std::make_pair(false, a_position.actual_center_of_mass);
  std::pair<bool, glm::dvec3> b_pair = std::make_pair(false, b_position.actual_center_of_mass);
  
  assert(a_rigid_object.entity_index_collision_map.find(entity_b) != a_rigid_object.entity_index_collision_map.end());

  glm::dvec3 collision_point = pce3d::space_map::findPointOfIndex(
    a_rigid_object.entity_index_collision_map.at(entity_b)[0],
    pce3d::Core3D::SPACE_MAP_DIMENSIONS, 
    pce3d::Core3D::COLLISION_METER_INDEX_RATIO);
  
  /* find entity a hitpoint */
  if (a_rigid_object.entity_vertex_collision_map.find(entity_b) 
  != a_rigid_object.entity_vertex_collision_map.end())
  {
    std::cout << "entity A: vertex collision" << '\n';
    const uint32_t a_hitpoint_id = a_rigid_object.entity_vertex_collision_map.at(entity_b);
    // std::cout << "a_hitpoint_id: " << a_hitpoint_id << '\n';
    a_hit_point = a_rigid_object.vertices.at(a_hitpoint_id);
    if (a_rigid_object.radius != 0)
    {
      std::cout << "entity A is a particle" << '\n';
      const glm::dvec3 hit_point_direction = glm::normalize(collision_point - a_position.actual_center_of_mass);
      // a_hit_point += a_rigid_object.radius * hit_point_direction;
      a_hit_point += a_rigid_object.radius * 0.9 * hit_point_direction;
    }
    a_pair = std::make_pair(true, a_hit_point);
  }
  else
  {
    std::cout << "entity A: face collision" << '\n';
    assert(a_rigid_object.entity_face_collision_map.find(entity_b) != a_rigid_object.entity_face_collision_map.end());
    const uint32_t a_hit_face_id = a_rigid_object.entity_face_collision_map.at(entity_b);

    a_hit_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(a_hit_face_id)[0]),
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(a_hit_face_id)[1]),
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(a_hit_face_id)[2]),
      collision_point);
    a_pair = std::make_pair(false, a_hit_point);
  }

  /* find entity b hitpoint */
  if (b_rigid_object.entity_vertex_collision_map.find(entity_a) 
  != b_rigid_object.entity_vertex_collision_map.end())
  {
    std::cout << "entity B: vertex collision" << '\n';
    const uint32_t b_hitpoint_id = b_rigid_object.entity_vertex_collision_map.at(entity_a);
    b_hit_point = b_rigid_object.vertices.at(b_hitpoint_id);
    if (b_rigid_object.radius != 0)
    {
      std::cout << "entity B is a particle" << '\n';
      const glm::dvec3 hit_point_direction = glm::normalize(collision_point - b_position.actual_center_of_mass);
      b_hit_point += b_rigid_object.radius * hit_point_direction;
    }
    b_pair = std::make_pair(true, b_hit_point);
  }
  else
  {
    std::cout << "entity B: face collision" << '\n';
    assert(b_rigid_object.entity_face_collision_map.find(entity_a) != b_rigid_object.entity_face_collision_map.end());
    const uint32_t b_hit_face_id = b_rigid_object.entity_face_collision_map.at(entity_a);

    b_hit_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_hit_face_id)[0]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_hit_face_id)[1]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_hit_face_id)[2]),
      collision_point);
    b_pair = std::make_pair(false, b_hit_point);
  }


  return std::make_pair(a_pair, b_pair);
}



glm::dvec3 calculateFaceNormalVector(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
)
{
  const glm::dvec3 face_normal_line = pce3d::maths::calculateNormalVectorInDirectionOfPoint(
    rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[0]),
    rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[1]),
    rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[2]),
    position.actual_center_of_mass); 
  
  return face_normal_line;
}



std::pair<double, double> calculateLinearAndRotationalMomentumAllocationsAtPoint(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
  , const pce::Motion& motion
)
{
const glm::dvec3 center_to_point_vector = position.actual_center_of_mass - point;
  glm::dvec3 face_normal_line = center_to_point_vector;
  if (rigid_object.radius == 0 && face != 0)
  {
    // std::cout << "doing special face normal line calc" << '\n';
    face_normal_line = pce3d::maths::calculateNormalVectorInDirectionOfPoint(
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[0]),
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[1]),
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[2]),
      position.actual_center_of_mass);
  }

  const double angle = pce3d::maths::calculateAngleDegreesBetweenVectors(face_normal_line, center_to_point_vector);
  // std::cout << "point angle relative to center: " << angle << '\n';

  /* A: calculate linear component */
  double linear_allocation_percentage = (90.0 - abs(angle)) / 90.0;

  double rotational_allocation_percentage = angle / 90.0;

  if (isnan(linear_allocation_percentage)) {linear_allocation_percentage = 0.5; }
  if (isnan(rotational_allocation_percentage)) {rotational_allocation_percentage = 0.5; }

  return std::make_pair(linear_allocation_percentage, rotational_allocation_percentage);
}



glm::dvec3 calculateMomentumVectorAtSurfacePoint(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
  , pce::Motion& motion
)
{
  const glm::dvec3 center_to_point_vector = position.actual_center_of_mass - point;

  assert(!isnan(motion.velocity.x) && !isnan(motion.velocity.y) && !isnan(motion.velocity.z));
  if (isnan(motion.direction.x) || isnan(motion.direction.y) || isnan(motion.direction.z))
  {
    motion.direction = glm::normalize(motion.velocity);
  }

  glm::dvec3 face_normal_line = -glm::normalize(motion.velocity);

  if (rigid_object.radius == 0 && face != 0)
  {
    // std::cout << "doing special face normal line calc" << '\n';
    face_normal_line = pce3d::maths::calculateNormalVectorInDirectionOfPoint(
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[0]),
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[1]),
      rigid_object.vertices.at(rigid_object.face_vertex_map.at(face)[2]),
      position.actual_center_of_mass);
  }

  double angle = pce3d::maths::calculateAngleDegreesBetweenVectors(face_normal_line, center_to_point_vector);
  if (isnan(angle)) { angle = 0.0; }
  std::cout << "point angle relative to center: " << angle << '\n';

  /* A: calculate linear component */
  const double linear_allocation_percentage = (90.0 - abs(angle)) / 90.0;
  std::cout << "linear allocation: " << linear_allocation_percentage << '\n';



  std::cout << "speed: " << motion.speed << '\n';
  if (isnan(motion.direction.x)) { motion.direction.x = 0; }
  if (isnan(motion.direction.y)) { motion.direction.y = 0; }
  if (isnan(motion.direction.z)) { motion.direction.z = 0; }
  std::cout << "motion.direction: "
            << motion.direction.x << ", " 
            << motion.direction.y << ", " 
            << motion.direction.z << '\n';
  const glm::dvec3 linear_momentum_component = motion.direction * motion.speed * linear_allocation_percentage * rigid_object.mass;
  std::cout << "linear_momentum_component: "
            << linear_momentum_component.x << ", " 
            << linear_momentum_component.y << ", " 
            << linear_momentum_component.z << '\n';


  /* B: calculate rotational component */
  const glm::dvec3 incrementally_rotated_point = pce::rotateVector3byAngleAxis(
    center_to_point_vector,
    -0.001, 
    motion.rotational_axis);

  const glm::dvec3 rotation_velocity_vector 
    = glm::normalize(center_to_point_vector + incrementally_rotated_point) * motion.rotational_speed;

  const double rotational_mass_allocation = angle / 90.0;
  // const double rotational_mass_allocation = (90.0 - abs(angle)) / 90.0;
  // std::cout << "rotational allocation: " << rotational_allocation_percentage << '\n';
  const glm::dvec3 rotational_momentum_component = rotation_velocity_vector * rotational_mass_allocation  * rigid_object.mass;
  /* C: combine */
  std::cout << "rotational_momentum_component: "
            << rotational_momentum_component.x << ", " 
            << rotational_momentum_component.y << ", " 
            << rotational_momentum_component.z << '\n';

  glm::dvec3 total_momentum = motion.rotational_speed == 0 || rotational_mass_allocation == 0
    ? linear_momentum_component : linear_momentum_component + rotational_momentum_component;

  std::cout << "total_momentum: "
            << total_momentum.x << ", " 
            << total_momentum.y << ", " 
            << total_momentum.z << '\n';

  return (total_momentum);
}



std::pair<glm::dvec3, glm::dvec3> calculateMomentumVectorsAfterLiveBodCollision(
    const double l_mass   
  , const double s_mass
  , const glm::dvec3 l_momentum
  , const glm::dvec3 s_momentum
  , const pce::Surface& l_surface
  , const pce::Surface& s_surface
  , const glm::dvec3& l_surface_point
  , const glm::dvec3& s_surface_point
  , const glm::dvec3& l_s_hitpoint_wire
)
{
  const double l_magnitude_in_h_direction = glm::dot(l_momentum, l_s_hitpoint_wire)
                                          / glm::dot(l_s_hitpoint_wire, l_s_hitpoint_wire);
  const glm::dvec3 l_component_in_h_direction = l_s_hitpoint_wire * l_magnitude_in_h_direction;
  std::cout << "l_component_in_h_direction: "
            << l_component_in_h_direction.x << ", " 
            << l_component_in_h_direction.y << ", " 
            << l_component_in_h_direction.z << '\n';

  const double s_magnitude_in_h_direction = glm::dot(s_momentum, -l_s_hitpoint_wire)
                                          / glm::dot(-l_s_hitpoint_wire, -l_s_hitpoint_wire);
  const glm::dvec3 s_component_in_h_direction = -l_s_hitpoint_wire * s_magnitude_in_h_direction;

  std::cout << "s_component_in_h_direction: "
            << s_component_in_h_direction.x << ", " 
            << s_component_in_h_direction.y << ", " 
            << s_component_in_h_direction.z << '\n';

  // std::cout << "a_mag_in_h_dir: " << a_magnitude_in_h_direction << '\n';
  // std::cout << "b_mag_in_h_dir: " << b_magnitude_in_h_direction << '\n';

  double l_impact_directness = abs(l_magnitude_in_h_direction) / sqrt(glm::dot(l_momentum, l_momentum));
  double s_impact_directness = abs(s_magnitude_in_h_direction) / sqrt(glm::dot(s_momentum, s_momentum));
  
  if (isnan(l_impact_directness)) { l_impact_directness = 0.1; }
  if (isnan(s_impact_directness)) { s_impact_directness = 0.1; }

  std::cout << "l_impact_directness: " << l_impact_directness << '\n';
  std::cout << "s_impact_directness: " << s_impact_directness << '\n';

  /* assume large particle stops moving */ 
  const double total_collision_elasticity 
    = l_surface.collision_elasticity_index * s_surface.collision_elasticity_index;
  
  std::cout << "collision elasticity: " << total_collision_elasticity << '\n';

  const glm::dvec3 force = (l_component_in_h_direction - s_component_in_h_direction) * total_collision_elasticity;
  std::cout << "force: " << force.x << ", " << force.y << ", " << force.z << " newtons" << '\n';
  
  std::cout << "l_mass: " << l_mass << '\n';
  std::cout << "s_mass: " << s_mass << '\n';
    
  const glm::dvec3 l_new_momentum = l_momentum 
                                  // - l_component_in_h_direction
                                  // + (force / (l_mass / (l_mass + s_mass)));
                                  // + (force / (s_mass / l_mass));
                                  // - (force / (l_mass / s_mass));
                                  - (force / ((l_mass * l_impact_directness) / (s_mass * s_impact_directness)));
                                  // - (force / (l_magnitude_in_h_direction / s_magnitude_in_h_direction));

  std::cout << "l_new_momentum: "
            << l_new_momentum.x << ", " 
            << l_new_momentum.y << ", " 
            << l_new_momentum.z << '\n';

  const glm::dvec3 s_new_momentum = s_momentum 
                                  // - s_component_in_h_direction
                                  // + force / s_mass;
                                  // + (-force / (s_mass / (l_mass + s_mass)));
                                  // + (-force / (l_mass / s_mass));
                                  - (-force / ((s_mass * s_impact_directness) / (l_mass * l_impact_directness)));
                                  // - (-force / (s_mass / l_mass));
                                  // + (force / (s_magnitude_in_h_direction / l_magnitude_in_h_direction));

  std::cout << "s_new_momentum: "
            << s_new_momentum.x << ", " 
            << s_new_momentum.y << ", " 
            << s_new_momentum.z << '\n';

  return std::make_pair(l_new_momentum, s_new_momentum);
}



void updateLinearMotionAfterCollision(
    const double linear_allocation_at_point
  , const double rotational_allocation_at_point
  , const glm::dvec3 new_total_momentum_at_point
  , pce::Motion& motion
  , const double mass
)
{
  // const double linear_alloc_percent = (linear_allocation_at_point / (linear_allocation_at_point + rotational_allocation_at_point));
  const double linear_alloc_percent = linear_allocation_at_point;
  std::cout << "linear alloc: " << linear_alloc_percent << '\n';
  motion.velocity = (new_total_momentum_at_point / (mass * linear_alloc_percent)) * linear_alloc_percent;
  std::cout << "new linear velocity: "
            << motion.velocity.x << ", " 
            << motion.velocity.y << ", " 
            << motion.velocity.z << '\n';
}



void updateRotationalMotionAfterCollision(
    const double rotational_allocation_at_point
  , const double linear_allocation_at_point
  , const glm::dvec3 surface_point
  , const glm::dvec3 center_of_mass
  , const glm::dvec3 new_total_momentum_at_point
  , pce::Motion& motion
  , const double mass
)
{
  const double rotational_alloc_percent = (rotational_allocation_at_point / (linear_allocation_at_point + rotational_allocation_at_point));
  std::cout << "rotational_alloc_at_point: " << rotational_allocation_at_point << '\n';
  std::cout << "rotational_alloc_percent: " << rotational_alloc_percent << '\n';
  // const glm::dvec3 nv = glm::normalize(new_total_momentum_at_point);
  // std::cout << "nv: "
            // << nv.x << ", "
            // << nv.x << ", "
            // << nv.z << '\n';
  const glm::dvec3 rotation_path_point = glm::normalize(new_total_momentum_at_point) * rotational_alloc_percent;
  const glm::dvec3 normalized_surface_point = surface_point - center_of_mass;
  const glm::dvec3 normalized_path_point = rotation_path_point - center_of_mass;

  const double angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    normalized_surface_point,
    normalized_path_point
  );

  const glm::dvec3 axis = glm::cross(
    normalized_surface_point,
    rotation_path_point
  );

  const double new_rotational_speed = sqrt(glm::dot(rotation_path_point, rotation_path_point)) 
                                    * pce::math::sign(angle);
  // const double new_rotational_speed = sqrt(glm::dot(new_total_momentum_at_point, new_total_momentum_at_point)) 
                                      // * -pce::math::sign(angle) * rotational_alloc_percent;
  if (!isnan(new_rotational_speed))
  {
    std::cout << "previous rotational speed: " << motion.rotational_speed << '\n';
    motion.rotational_speed += new_rotational_speed * rotational_alloc_percent;
    std::cout << "new rotational speed: " << motion.rotational_speed << '\n';
  }
  else
  {
    motion.rotational_speed *= -1.0; 
  }
  // std::cout << "rotational_speed: " << motion.rotational_speed  << '\n';
  if (!isnan(axis.x) && !isnan(axis.y) && !isnan(axis.z))
  {
    motion.rotational_axis += axis;
  }
  // std::cout << "motion.rotational_axis: "
            // << motion.rotational_axis.x << ", "
            // << motion.rotational_axis.y << ", "
            // << motion.rotational_axis.z << "\n";
}



void updateEntityDataFromLiveBodCollision(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::Position& a_position
  , pce::Surface& a_surface
  , pce::Motion& a_motion
  , pce::Force& a_force
  , pce::RigidObject& b_rigid_object
  , pce::Position& b_position
  , pce::Surface& b_surface
  , pce::Motion& b_motion
  , pce::Force& b_force
)
{
  const std::pair<std::pair<bool, glm::dvec3>, std::pair<bool, glm::dvec3>>
  entityAandB_hitpoints = calculateLiveBodHitPointsAndIfVertex(
    entity_a,
    entity_b,
    a_rigid_object,
    b_rigid_object,
    a_position,
    b_position
  );

  const glm::dvec3 a_hitpoint = entityAandB_hitpoints.first.second;
  const glm::dvec3 b_hitpoint = entityAandB_hitpoints.second.second;
  glm::dvec3 a_b_hitpoint_wire = glm::normalize(b_hitpoint - a_hitpoint);
  if (isnan(a_b_hitpoint_wire.x) || isnan(a_b_hitpoint_wire.y) || isnan(a_b_hitpoint_wire.z))
  {
    a_b_hitpoint_wire = glm::dvec3(0, 0, 0);
  }

  // std::cout << "a_b_hitpoint_wire: "
  //           << a_b_hitpoint_wire.x << ", " 
  //           << a_b_hitpoint_wire.y << ", " 
  //           << a_b_hitpoint_wire.z << '\n';
  // std::cout << "a_hitpoint: "
  //           << a_hitpoint.x << ", " 
  //           << a_hitpoint.y << ", " 
  //           << a_hitpoint.z << '\n';
  // std::cout << "b_hitpoint: "
  //           << b_hitpoint.x << ", " 
  //           << b_hitpoint.y << ", " 
  //           << b_hitpoint.z << '\n';
  
  uint32_t a_face = 1;
  uint32_t b_face = 1;
  
  if (a_rigid_object.radius == 0)
  {
    a_face = !entityAandB_hitpoints.first.first
      ? a_rigid_object.entity_face_collision_map.at(entity_b)
      : 0;
  }

  if (b_rigid_object.radius == 0)
  {
  b_face = !entityAandB_hitpoints.second.first
    ? b_rigid_object.entity_face_collision_map.at(entity_a)
    : 0;
  }

  const glm::dvec3 a_momentum_vector = calculateMomentumVectorAtSurfacePoint(
    a_hitpoint,
    a_face,
    a_rigid_object,
    a_position,
    a_motion
  );

  const glm::dvec3 b_momentum_vector = calculateMomentumVectorAtSurfacePoint(
    b_hitpoint,
    b_face,
    b_rigid_object,
    b_position,
    b_motion
  );

  
  std::pair<glm::dvec3, glm::dvec3> new_momentum_vectors 
    = std::make_pair(a_momentum_vector, b_momentum_vector);
  glm::dvec3 a_new_momentum_vector = a_momentum_vector;
  glm::dvec3 b_new_momentum_vector = b_momentum_vector;

  if (a_rigid_object.mass >= b_rigid_object.mass)
  {
    new_momentum_vectors = calculateMomentumVectorsAfterLiveBodCollision(
      a_rigid_object.mass,
      b_rigid_object.mass,
      a_momentum_vector,
      b_momentum_vector,
      a_surface,
      b_surface,
      a_hitpoint,
      b_hitpoint,
      a_b_hitpoint_wire
    );
    a_new_momentum_vector = new_momentum_vectors.first;
    b_new_momentum_vector = new_momentum_vectors.second;
  } 
  else
  {
    new_momentum_vectors = calculateMomentumVectorsAfterLiveBodCollision(
      b_rigid_object.mass,
      a_rigid_object.mass,
      b_momentum_vector,
      a_momentum_vector,
      b_surface,
      a_surface,
      b_hitpoint,
      a_hitpoint,
      -a_b_hitpoint_wire
    );
    b_new_momentum_vector = new_momentum_vectors.first;
    a_new_momentum_vector = new_momentum_vectors.second;
  }

  const std::pair<double, double> a_momentum_allocations 
    = calculateLinearAndRotationalMomentumAllocationsAtPoint(
      a_hitpoint,
      a_face,
      a_rigid_object,
      a_position,
      a_motion
    );

  const std::pair<double, double> b_momentum_allocations 
    = calculateLinearAndRotationalMomentumAllocationsAtPoint(
      b_hitpoint,
      b_face,
      b_rigid_object,
      b_position,
      b_motion
    );

  // std::cout << "a_momentum_allocations: " << a_momentum_allocations.first << ", " << a_momentum_allocations.second << '\n';
  // std::cout << "b_momentum_allocations: " << b_momentum_allocations.first << ", " << b_momentum_allocations.second << '\n';

  updateLinearMotionAfterCollision(
    a_momentum_allocations.first,
    a_momentum_allocations.second,
    a_new_momentum_vector,
    a_motion,
    a_rigid_object.mass
  );
  updateLinearMotionAfterCollision(
    b_momentum_allocations.first,
    b_momentum_allocations.second,
    b_new_momentum_vector,
    b_motion,
    b_rigid_object.mass
  );

  if (a_rigid_object.radius == 0)
  {
    updateRotationalMotionAfterCollision(
      a_momentum_allocations.second,
      a_momentum_allocations.first,
      a_hitpoint,
      a_position.actual_center_of_mass,
      a_new_momentum_vector,
      a_motion,
      a_rigid_object.mass
    );
  }
  if (b_rigid_object.radius == 0)
  {
    updateRotationalMotionAfterCollision(
      b_momentum_allocations.second,
      b_momentum_allocations.first,
      b_hitpoint,
      b_position.actual_center_of_mass,
      b_new_momentum_vector,
      b_motion,
      b_rigid_object.mass
    );

  }
  
  a_motion.duration = 0.1;
  a_motion.previous_resting_position = a_position.actual_center_of_mass;
  b_motion.previous_resting_position = b_position.actual_center_of_mass;
  b_motion.duration = 0.1;
  

}



void updateComplexLivebodInfoAfterDeadfaceCollision(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::Position& a_position
  , pce::Surface& a_surface
  , pce::Motion& a_motion
  , const std::vector<glm::dvec3>& deadbod_face_vertices
  , double total_surface_elasticity
)
{
  glm::dvec3 a_collision_point = a_position.actual_center_of_mass;
  uint32_t face = 0;
  if (a_rigid_object.entity_vertex_collision_map.find(entity_b) != a_rigid_object.entity_vertex_collision_map.end())
  {
    a_collision_point = a_rigid_object.vertices.at(a_rigid_object.entity_vertex_collision_map.at(entity_b));
  }
  // else if (a_rigid_object.entity_edge_collision_map.find(entity_b) != a_rigid_object.entity_edge_collision_map.end())
  // {
    // a_collision_point = a_rigid_object.entity_edge_collision_map.at(entity_b);
  // }
  else 
  {
    assert(a_rigid_object.entity_face_collision_map.find(entity_b) != a_rigid_object.entity_face_collision_map.end());
    uint32_t a_face_id = a_rigid_object.entity_face_collision_map.at(entity_b);
    face = a_face_id;
    glm::dvec3 sum = glm::dvec3(0, 0, 0);
    for (auto const& vertex_id : a_rigid_object.face_vertex_map.at(a_face_id))
    {
      sum += a_rigid_object.vertices.at(vertex_id);
    }
    a_collision_point = sum  * (1.0 / double(a_rigid_object.face_vertex_map.at(a_face_id).size())); 
  }

  std::cout << "a collision point: "
            << a_collision_point.x << ", "
            << a_collision_point.y << ", "
            << a_collision_point.z << '\n';

  std::pair<double, double> a_momentum_allocations 
    = calculateLinearAndRotationalMomentumAllocationsAtPoint(
        a_collision_point,
        face,
        a_rigid_object,
        a_position,
        a_motion
  );

  // const glm::dvec3 a_momentum_vector = calculateMomentumVectorAtSurfacePoint(
    // a_collision_point,
    // face,
    // a_rigid_object,
    // a_position,
    // a_motion
  // );

  const glm::dvec3 new_a_velocity = a_rigid_object.mass * calculateVelocityVectorAfterLiveParticleDeadFaceCollision( 
      a_motion.direction * a_motion.speed, a_rigid_object.mass, deadbod_face_vertices, total_surface_elasticity);
  

  updateLinearMotionAfterCollision(
    a_momentum_allocations.first,
    a_momentum_allocations.second,
    /*update the below eventually -- should be momentum, just looks better w/ velocity atm*/
    new_a_velocity,
    a_motion,
    a_rigid_object.mass
  );
  if (a_rigid_object.radius == 0)
  {
    updateRotationalMotionAfterCollision(
      a_momentum_allocations.second,
      a_momentum_allocations.first,
      a_collision_point,
      a_position.actual_center_of_mass,
    /*update the below eventually -- should be momentum, just looks better w/ velocity atm*/
      new_a_velocity,
      a_motion,
      a_rigid_object.mass
    );
  }

  a_motion.duration = 0.1;
  a_motion.previous_resting_position = a_position.actual_center_of_mass;

  std::cout << "new_a_velocity: "
            << a_motion.velocity.x << ", " 
            << a_motion.velocity.y << ", " 
            << a_motion.velocity.z << '\n';
  
}


void checkForParticleCollisionWithHardBoundary(
    pce::Position& position
  , pce::RigidObject& rigid_object
  , pce::Motion& motion
  , pce::Surface& surface
)
{
  const glm::dvec3 boundaries = pce3d::Core3D::HARD_BOUNDARIES - pce3d::Core3D::MAP_CENTER;

  glm::dvec3 face_point = glm::dvec3(0, 0, 0);
  bool collision_occured = false;
  
  std::vector<glm::dvec3> face_vertices{};
  /* check left - right sides */
  if ((position.actual_center_of_mass.x + rigid_object.radius) > boundaries.x)
  {
    position.actual_center_of_mass.x = boundaries.x - rigid_object.radius;

    face_point = glm::dvec3(boundaries.x,
                            position.actual_center_of_mass.y,
                            position.actual_center_of_mass.z);
    face_vertices.push_back(face_point);
    face_point.y += 1.0;
    face_vertices.push_back(face_point);
    face_point.z += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }
  if ((position.actual_center_of_mass.x - rigid_object.radius) < -boundaries.x)
  {
    position.actual_center_of_mass.x = -boundaries.x + rigid_object.radius;

    face_point = glm::dvec3(-boundaries.x,
                            position.actual_center_of_mass.y,
                            position.actual_center_of_mass.z);
    face_vertices.push_back(face_point);
    face_point.y += 1.0;
    face_vertices.push_back(face_point);
    face_point.z += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }

  /* check front - back sides */
  if ((position.actual_center_of_mass.z + rigid_object.radius) > boundaries.z)
  {
    position.actual_center_of_mass.z = boundaries.z - rigid_object.radius;

    face_point = glm::dvec3(position.actual_center_of_mass.x,
                            position.actual_center_of_mass.y,
                            boundaries.z);

    face_vertices.push_back(face_point);
    face_point.y += 1.0;
    face_vertices.push_back(face_point);
    face_point.x += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }
  if ((position.actual_center_of_mass.z - rigid_object.radius) < -boundaries.z)
  {
    position.actual_center_of_mass.z = -boundaries.z + rigid_object.radius;

    face_point = glm::dvec3(position.actual_center_of_mass.x,
                            position.actual_center_of_mass.y,
                            -boundaries.z);

    face_vertices.push_back(face_point);
    face_point.y += 1.0;
    face_vertices.push_back(face_point);
    face_point.x += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }

  /* check top - bottom sides*/
  if ((position.actual_center_of_mass.y - rigid_object.radius) < -boundaries.y)
  {
    position.actual_center_of_mass.y = -boundaries.y + rigid_object.radius;

    face_point = glm::dvec3(position.actual_center_of_mass.x,
                            -boundaries.y,
                            position.actual_center_of_mass.z);

    face_vertices.push_back(face_point);
    face_point.z += 1.0;
    face_vertices.push_back(face_point);
    face_point.x += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }
  if ((position.actual_center_of_mass.y + rigid_object.radius) > boundaries.y)
  {
    position.actual_center_of_mass.y = boundaries.y - rigid_object.radius;

    face_point = glm::dvec3(position.actual_center_of_mass.x,
                            boundaries.y,
                            position.actual_center_of_mass.z);

    face_vertices.push_back(face_point);
    face_point.z += 1.0;
    face_vertices.push_back(face_point);
    face_point.x += 1.0;
    face_vertices.push_back(face_point);
    collision_occured = true;
  }

  if (collision_occured)
  {
    rigid_object.vertices[1] = position.actual_center_of_mass;

    physics::updateLiveParticleInfoAfterDeadFaceCollision( 
      position.actual_center_of_mass, 
      rigid_object.radius,
      rigid_object.mass,
      motion,
      face_vertices,
      surface.collision_elasticity_index
    );
  }
}



}
}



#endif /* physicsFunctions_cpp */
