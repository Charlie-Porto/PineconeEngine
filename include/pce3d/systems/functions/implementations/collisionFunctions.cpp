#ifndef collisionFunctions_cpp
#define collisionFunctions_cpp

#include "../collisionFunctions.hpp"

namespace pce3d {
namespace collision {

bool determineIfMovementVectorsIndicateCollision(
    glm::dvec3 a_movement_vector
  , glm::dvec3 b_movement_vector
  , const pce::RigidObject& a_rigid_object
  , const pce::RigidObject& b_rigid_object
)
{
if (a_movement_vector == glm::dvec3(0, 0, 0))
  {
    a_movement_vector = b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1);
  }
  if (b_movement_vector == glm::dvec3(0, 0, 0))
  {
    b_movement_vector = a_rigid_object.vertices.at(1) - b_rigid_object.vertices.at(1);
  }

  const double directions_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_movement_vector, b_movement_vector);

  /* if the direction angle is > 90.0, we know a collision is occuring */
  if (directions_angle > 90.0)
  {
    const glm::dvec3 collision_point = a_rigid_object.vertices.at(1)
                                     + (glm::normalize(b_rigid_object.vertices.at(1) 
                                                     - a_rigid_object.vertices.at(1)))
                                     * a_rigid_object.radius;
    
    return true;
  }
  else
  {
    /* if NOT POINTING TOWARD EACH OTHER, check if longer vector is behind shorter; if so, collision. */
    glm::dvec3 smaller = a_movement_vector;
    if (pce3d::maths::calcMagV3(a_movement_vector) > pce3d::maths::calcMagV3(b_movement_vector))
    {
      smaller = b_movement_vector;
    }

    a_movement_vector = a_movement_vector - smaller;
    b_movement_vector = b_movement_vector - smaller;

    if (a_movement_vector == glm::dvec3(0, 0, 0))
    {
      a_movement_vector = b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1);
    }
    if (b_movement_vector == glm::dvec3(0, 0, 0))
    {
      b_movement_vector = a_rigid_object.vertices.at(1) - b_rigid_object.vertices.at(1);
    }

    const double directions_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
      a_movement_vector, b_movement_vector);

    /* if the direction angle is > 90.0, we know a collision is occuring */
    if (directions_angle > 90.0)
    {
      const glm::dvec3 collision_point = a_rigid_object.vertices.at(1)
                                      + (glm::normalize(b_rigid_object.vertices.at(1) 
                                                      - a_rigid_object.vertices.at(1)))
                                      * a_rigid_object.radius;
      
      return true;
    }
  }
  return false;
}



std::pair<bool, glm::dvec3> determineIfParticlesAreCollidingAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
)
{
  const double centers_distance = pce3d::maths::calculateDistanceBetweenVectors(
    a_rigid_object.vertices.at(1), b_rigid_object.vertices.at(1));
  
  const double radius_sum = a_rigid_object.radius + b_rigid_object.radius;

  if (radius_sum < centers_distance)
  {
    return std::make_pair(false, glm::dvec3(0, 0, 0));
  }
  if (a_motion.speed == 0 && b_motion.speed == 0)
  {
    return std::make_pair(false, glm::dvec3(0, 0, 0));
  }

  /* proceeding only if touching; now to see if moving toward each other netly*/
  glm::dvec3 a_direction = a_motion.direction * a_motion.speed;
  glm::dvec3 b_direction = b_motion.direction * b_motion.speed;
  
  if (determineIfMovementVectorsIndicateCollision(a_direction, b_direction, a_rigid_object, b_rigid_object))
  {
    return std::make_pair(true, a_rigid_object.vertices.at(1) 
                       + (glm::normalize(b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1))
                       * a_rigid_object.radius));
  }
  return std::make_pair(false, glm::dvec3(0, 0, 0));
}



std::pair<bool, glm::dvec3> determineIfParticleIsCollidingWithComplexBodAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
)
{
  /* 0. find complex bod's closest vertex to the particle center */
  uint32_t closest_vertex_id = 1;
  double closest_vertex_distance = 100000.0;
  for (auto const& [id, vertex] : b_rigid_object.vertices)
  {
    const double distance = pce3d::maths::calculateDistanceBetweenVectors(
      a_rigid_object.vertices.at(1), vertex);
    
    if (distance < closest_vertex_distance)
    {
      closest_vertex_distance = distance;
      closest_vertex_id = id;
    }
  }

  glm::dvec3 potential_point_of_contact = glm::dvec3(b_rigid_object.vertices.at(closest_vertex_id));

  /* 1. check if particle is colliding with closest vertex */
  if (closest_vertex_distance >= a_rigid_object.radius)
  {
    /* 2. if NOT COLLIDING WITH CLOSEST VERTEX, get closest face */
    uint32_t closest_face_id = 1;
    double closest_face_corner_distance = 10000.0;
    assert(b_rigid_object.vertex_face_corner_map.find(closest_vertex_id) 
        != b_rigid_object.vertex_face_corner_map.end());
    for (auto const& [face_id, face_corner_id] : b_rigid_object.vertex_face_corner_map.at(closest_vertex_id))
    {
      assert(b_rigid_object.face_corner_map.find(face_corner_id) != b_rigid_object.face_corner_map.end());

      const glm::dvec3 face_corner_point = b_rigid_object.face_corner_map.at(face_corner_id);
      const double distance = pce3d::maths::calculateDistanceBetweenVectors(
        a_rigid_object.vertices.at(1), face_corner_point);
      
      if (distance < closest_face_corner_distance)
      {
        closest_face_corner_distance = distance;
        closest_face_id = face_id;
      }
    }

    /* 3. get closest face's closest point to particle */
    const glm::dvec3 closest_face_point_to_particle = pce3d::maths::calculateClosestPointInPlaneToPoint(
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_face_id)[0]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_face_id)[1]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_face_id)[2]),
      a_rigid_object.vertices.at(1));
    
    /* 4. determine if distance(closest face's closest point, particle_center) is <= radius */
    const double distance = pce3d::maths::calculateDistanceBetweenVectors(
      closest_face_point_to_particle, a_rigid_object.vertices.at(1));
    
    if (distance > a_rigid_object.radius)
    {
      return std::make_pair(false, glm::dvec3(0, 0, 0));
    }
    else
    {
      potential_point_of_contact = closest_face_point_to_particle; 
    }
  }

  /* 5. evaluate direction vectors if objects are, in fact, touching */
  const glm::dvec3 a_direction = a_motion.direction;
  glm::dvec3 b_direction = pce3d::physics2::calculateRotationalVelocityOfPointOnObject(
    potential_point_of_contact, 
    b_position.actual_center_of_mass,
    b_motion.rotational_speed,
    b_motion.rotational_axis);
  
  b_direction = b_direction + b_motion.direction * b_motion.speed;

  if (determineIfMovementVectorsIndicateCollision(a_direction, b_direction, a_rigid_object, b_rigid_object))
  {
    return std::make_pair(true, potential_point_of_contact);
  }
  else 
  {
    return std::make_pair(false, glm::dvec3(0, 0, 0));
  }
}
 


std::pair<bool, glm::dvec3> determineIfComplexBodsAreCollidingAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const pce::Position& a_position
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
)
{
  
}


}
}



#endif /* collisionFunctions_cpp */
