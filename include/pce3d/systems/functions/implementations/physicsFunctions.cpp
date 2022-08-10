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

  // std::cout << "normal_vec: "
  //           << normal_vec.x << ", " 
  //           << normal_vec.y << ", " 
  //           << normal_vec.z << '\n';
  const glm::dvec3 reverse_velocity_vect = -p_velocity_vect;
  new_velocity_vector = pce::rotateVector3byAngleAxis(reverse_velocity_vect, 180.0, normal_vec) * elasticity;
  // std::cout << "new velocity: "
  //           << new_velocity_vector.x << ", " 
  //           << new_velocity_vector.y << ", " 
  //           << new_velocity_vector.z << '\n';
  return new_velocity_vector;
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
  motion.previous_resting_position = p_center;
  motion.duration = 0.05;
  motion.speed = sqrt(glm::dot(motion.velocity, motion.velocity));
  
  


}




}
}



#endif /* physicsFunctions_cpp */
