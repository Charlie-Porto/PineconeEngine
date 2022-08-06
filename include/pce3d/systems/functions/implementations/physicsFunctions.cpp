#ifndef physicsFunctions_cpp
#define physicsFunctions_cpp

#include "../physicsFunctions.hpp"


namespace pce3d {
namespace physics {

glm::dvec3 calculateParticlePositionGivenTime(
    const glm::dvec3& initial_position, const glm::dvec3& initial_velocity, 
    double time_change, double gravitational_force_applied,
    double& duration) {

  duration += time_change;
  glm::dvec3 path_traveled = initial_velocity * duration;   
  path_traveled.y += (0.5 * pow(duration, 2.0) * GRAVITY * gravitational_force_applied);
  return glm::dvec3(initial_position + path_traveled);
}



bool determineIfParticlesAreColliding(
    const glm::dvec3& a_position, const double a_radius,
    const glm::dvec3& b_position, const double b_radius) {
  
  const double total_radius = a_radius + b_radius;
  const glm::dvec3 diff_vect = a_position - b_position;
  const double centers_distance = sqrt(glm::dot(diff_vect, diff_vect));
  return total_radius > centers_distance ? true : false;
}


std::pair<glm::dvec3, glm::dvec3> calculateVelocityVectorsAfterTwoParticleCollision(
    const glm::dvec3& a_center, const double a_radius, const glm::dvec3& a_velocity_vect, const double a_mass,
    const glm::dvec3& b_center, const double b_radius, const glm::dvec3& b_velocity_vect, const double b_mass) {
  
  const glm::dvec3 a_hitpoint_direction = glm::normalize(b_center - a_center);
  const glm::dvec3 b_hitpoint_direction = glm::normalize(a_center - b_center);

  const double a_angle = acos(glm::dot(a_hitpoint_direction, a_velocity_vect)
                                /(sqrt(glm::dot(a_hitpoint_direction, a_hitpoint_direction))
                                * sqrt(glm::dot(a_velocity_vect, a_velocity_vect)))) / PI * 180.0;

  const double b_angle = acos(glm::dot(b_hitpoint_direction, b_velocity_vect)
                              /(sqrt(glm::dot(b_hitpoint_direction, b_hitpoint_direction))
                              * sqrt(glm::dot(b_velocity_vect, b_velocity_vect)))) / PI * 180.0;

  // std::cout << "a_angle: " << a_angle << '\n';
  // std::cout << "b_angle: " << b_angle << '\n';
 
  const double a_directness = (90.0 - a_angle)/90.0;
  const double b_directness = (90.0 - b_angle)/90.0;

  // std::cout << "a_directness: " << a_directness << '\n';
  // std::cout << "b_directness: " << b_directness << '\n';

  // const glm::dvec3 a_provision = a_velocity_vect * a_directness * b_mass_percentage;
  // const glm::dvec3 b_provision = b_velocity_vect * b_directness * a_mass_percentage;
  const glm::dvec3 a_provision = a_velocity_vect * a_directness;
  const glm::dvec3 b_provision = b_velocity_vect * b_directness;

  return std::make_pair(a_provision, b_provision);
}



bool determineIfParticleIsCollidingWithFace(
    const glm::dvec3& p_center, const double p_radius, 
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices) {

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
    const std::vector<glm::dvec3>& face_vertices, double elasticity) {
  /* pick up here */ 
  glm::dvec3 new_velocity_vector = p_velocity_vect;
  // pce3d::maths::PlaneCartesianForm face_plane = pce3d::maths::calculatePlaneGiven3Points(face_vertices[0], 
                                                                                        //  face_vertices[1], 
                                                                                        //  face_vertices[2]);
  glm::dvec3 normal_vec = glm::normalize(glm::cross(face_vertices[0] - face_vertices[1], 
                                                    face_vertices[2] - face_vertices[1]));

  /* if not colliding, return velocity vector with no changes */
  // if (distance > p_radius) {
  //   return new_velocity_vector;
  // }

  /* make sure we have the correct normal vector direction */
  // if (!pce3d::maths::checkIfPointInPlane(face_vertices[0], face_vertices[1], face_vertices[2], 
  //                                        p_center + normal_vec * distance)) {
  //   normal_vec = -normal_vec; 
  // }
  
  const glm::dvec3 reverse_velocity_vect = -p_velocity_vect;
  new_velocity_vector = pce::rotateVector3byAngleAxis(reverse_velocity_vect, 180.0, normal_vec) * elasticity;
  // new_velocity_vector = pce::rotateVector3byAngleAxis(reverse_velocity_vect, 180.0, normal_vec);
  // new_velocity_vector = new_velocity_vector * (elasticity * initial_speed);
  // double final_speed = sqrt(glm::dot(new_velocity_vector, new_velocity_vector));
  // std::cout <<"final speed: " <<final_speed << '\n';
  return new_velocity_vector;
}






}
}



#endif /* physicsFunctions_cpp */
