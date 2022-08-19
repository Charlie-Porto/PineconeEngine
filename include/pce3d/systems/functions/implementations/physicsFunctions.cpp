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
  std::cout << "a_velocity_vect: "<< a_velocity_vect.x << ", " << a_velocity_vect.y << ", " << a_velocity_vect.z << '\n';
  std::cout << "b_velocity_vect: "<< b_velocity_vect.x << ", " << b_velocity_vect.y << ", " << b_velocity_vect.z << '\n';

  const double a_velocity_magnitude = sqrt(glm::dot(a_velocity_vect, a_velocity_vect));
  const double b_velocity_magnitude = sqrt(glm::dot(b_velocity_vect, b_velocity_vect));

  const glm::dvec3 a_magnitude_in_a_direction = a_hitpoint_direction * a_velocity_magnitude;
  const glm::dvec3 b_magnitude_in_b_direction = b_hitpoint_direction * b_velocity_magnitude;
 
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
  

  // a_motion.velocity += (new_velocity_vectors.second);
  a_motion.velocity = (new_velocity_vectors.second);
  // std::cout << "new_a_velocity: "<< a_motion.velocity.x << ", " << a_motion.velocity.y << ", " << a_motion.velocity.z << '\n';
  a_motion.direction = glm::normalize(a_motion.velocity);
  // if (isnan(a_motion.direction.x) || isnan(a_motion.direction.y) || isnan(a_motion.direction.y)) {
    // a_motion.direction = a_motion.velocity;
  // }
  // std::cout << "former_b_velocity: "<< b_motion.velocity.x << ", " << b_motion.velocity.y << ", " << b_motion.velocity.z << '\n';
  // b_motion.velocity += (new_velocity_vectors.first + new_velocity_vectors.second);
  // b_motion.velocity += new_velocity_vectors.first;
  b_motion.velocity = new_velocity_vectors.first;
  // std::cout << "new_b_velocity: "<< b_motion.velocity.x << ", " << b_motion.velocity.y << ", " << b_motion.velocity.z << '\n';
  b_motion.direction = glm::normalize(b_motion.velocity);
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
    const glm::dvec3& p_center, const double p_radius, 
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

  std::cout << "normal_vec: "
            << normal_vec.x << ", " 
            << normal_vec.y << ", " 
            << normal_vec.z << '\n';
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
      p_center, p_radius, motion.direction * motion.speed, mass, face_vertices, elasticity);
  
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

  const glm::dvec3 collision_point = pce3d::space_map::findPointOfIndex(
    a_rigid_object.entity_index_collision_map.at(entity_b)[0],
    pce3d::Core3D::SPACE_MAP_DIMENSIONS, 
    pce3d::Core3D::COLLISION_METER_INDEX_RATIO);

  /* find entity a hitpoint */
  if (a_rigid_object.entity_vertex_collision_map.find(entity_b) 
   != a_rigid_object.entity_vertex_collision_map.end())
  {
    std::cout << "entity A: vertex collision" << '\n';
    const uint32_t a_hitpoint_id = a_rigid_object.entity_vertex_collision_map.at(entity_b);
    a_hit_point = a_rigid_object.vertices.at(a_hitpoint_id);
    if (a_rigid_object.radius != 0)
    {
      std::cout << "entity A is a particle" << '\n';
      const glm::dvec3 hit_point_direction = glm::normalize(collision_point - a_position.actual_center_of_mass);
      a_hit_point += a_rigid_object.radius * hit_point_direction;
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



glm::dvec3 calculateMomentumVectorAtSurfacePoint(
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

  const double angle = pce3d::maths::calculateAngleDegreesBetweenVectors(face_normal_line, center_to_point_vector)
                       / 3.14159265 * 180.0;
  // std::cout << "point angle relative to center: " << angle << '\n';

  /* A: calculate linear component */
  const double linear_allocation_percentage = angle == 0 ? 1.0 : 90.0 / angle;
  // std::cout << "linear allocation: " << linear_allocation_percentage << '\n';
  const glm::dvec3 linear_momentum_component = motion.direction * motion.speed * linear_allocation_percentage * rigid_object.mass;

  // std::cout << "speed: " << motion.speed << '\n';
  // std::cout << "motion.direction: "
            // << motion.direction.x << ", " 
            // << motion.direction.y << ", " 
            // << motion.direction.z << '\n';
  // std::cout << "linear_momentum_component: "
            // << linear_momentum_component.x << ", " 
            // << linear_momentum_component.y << ", " 
            // << linear_momentum_component.z << '\n';


  /* B: calculate rotational component */
  const double rotation_direction = pce::math::sign(motion.rotational_speed);
  const glm::dvec3 incrementally_rotated_point = pce::rotateVector3byAngleAxis(
    center_to_point_vector,
    0.001, 
    motion.rotational_axis);

  const glm::dvec3 rotation_velocity_vector 
    = glm::normalize(center_to_point_vector + incrementally_rotated_point) * motion.rotational_speed;

  
  const double rotational_allocation_percentage = angle == 90.0 ? 90.0 / .001 : (90.0 / (90.0 - angle)) - 1.0;
  // std::cout << "rotational allocation: " << rotational_allocation_percentage << '\n';
  const glm::dvec3 rotational_momentum_component = rotation_velocity_vector * rotational_allocation_percentage  * rigid_object.mass;
  /* C: combine */
  // std::cout << "rotational_momentum_component: "
            // << rotational_momentum_component.x << ", " 
            // << rotational_momentum_component.y << ", " 
            // << rotational_momentum_component.z << '\n';

  const glm::dvec3 total_momentum = linear_momentum_component + rotational_momentum_component;
  std::cout << "total_momentum: "
            << total_momentum.x << ", " 
            << total_momentum.y << ", " 
            << total_momentum.z << '\n';

  return (total_momentum);
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

  // std::cout << "a_hitpoint: "
            // << a_hitpoint.x << ", " 
            // << a_hitpoint.y << ", " 
            // << a_hitpoint.z << '\n';
  // std::cout << "b_hitpoint: "
            // << b_hitpoint.x << ", " 
            // << b_hitpoint.y << ", " 
            // << b_hitpoint.z << '\n';
  
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

  const glm::dvec3 a_momentum = calculateMomentumVectorAtSurfacePoint(
    a_hitpoint,
    a_face,
    a_rigid_object,
    a_position,
    a_motion
  );

  const glm::dvec3 b_momentum = calculateMomentumVectorAtSurfacePoint(
    b_hitpoint,
    b_face,
    b_rigid_object,
    b_position,
    b_motion
  );

}



}
}



#endif /* physicsFunctions_cpp */
