#ifndef physics2Functions_cpp
#define physics2Functions_cpp

#include "../physics2Functions.hpp"

namespace pce3d {
namespace physics2 {

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



std::pair<glm::dvec3, glm::dvec3> calculateVelocitiesAfterParticleCollision(
    const glm::dvec3& a_center
  , const glm::dvec3& b_center
  , const double a_mass
  , const double b_mass
  , const pce::Motion& a_motion
  , const pce::Motion& b_motion
  , const double a_radius
  , const double total_elasticity
)
{
  glm::dvec3 a_velocity = a_motion.direction * a_motion.speed;
  glm::dvec3 b_velocity = b_motion.direction * b_motion.speed;

  const glm::dvec3 collision_point = a_center + a_radius * glm::normalize(b_center - a_center);

  const glm::dvec3 a_center_to_collision_point_direction = glm::normalize(collision_point - a_center);
  const glm::dvec3 b_center_to_collision_point_direction = glm::normalize(collision_point - b_center);
  
  if (a_velocity == glm::dvec3(0, 0, 0))
  {
    a_velocity = a_center_to_collision_point_direction;
  }
  if (b_velocity == glm::dvec3(0, 0, 0))
  {
    b_velocity = b_center_to_collision_point_direction;
  }
  
  const double angle = double(int(pce3d::maths::calculateAngleDegreesBetweenVectors(a_velocity, b_velocity) 
                     * 1000)
                     % 90000)
                     / 1000.0;
  
  const double a_directness_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_velocity, a_center_to_collision_point_direction);
  const double b_directness_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    b_velocity, b_center_to_collision_point_direction);
  
  const double a_velocity_in_direction_of_collision_point = pce3d::maths::calcMagV3(a_velocity) 
                                                          * cos(a_directness_angle / 180.0 * PI);
  const double b_velocity_in_direction_of_collision_point = pce3d::maths::calcMagV3(b_velocity) 
                                                          * cos(b_directness_angle / 180.0 * PI);
  
  const double adjusted_elasticity = std::max(total_elasticity, total_elasticity / pow(std::max(0.01, 90.0 / angle), 2.0));

  std::cout << "adjusted elasticity: " << adjusted_elasticity << '\n';
  
  const glm::dvec3 a_velocity_adjustment = (b_center_to_collision_point_direction * b_velocity_in_direction_of_collision_point) 
                                         * (a_mass / b_mass);
  const glm::dvec3 b_velocity_adjustment = (a_center_to_collision_point_direction * a_velocity_in_direction_of_collision_point) 
                                         * (b_mass / a_mass);
  
  const glm::dvec3 new_a_velocity = (a_motion.direction * a_motion.speed 
                                  + a_velocity_adjustment - b_velocity_adjustment)
                                  * adjusted_elasticity;
  const glm::dvec3 new_b_velocity = (b_motion.direction * b_motion.speed 
                                  + b_velocity_adjustment - a_velocity_adjustment)
                                  * adjusted_elasticity;

  std::cout << "previous_a_velocity: "
            << a_velocity.x << ", "
            << a_velocity.y << ", "
            << a_velocity.z << '\n';
  std::cout << "previous_b_velocity: "
            << b_velocity.x << ", "
            << b_velocity.y << ", "
            << b_velocity.z << '\n';

  std::cout << "new_a_velocity: "
            << new_a_velocity.x << ", "
            << new_a_velocity.y << ", "
            << new_a_velocity.z << '\n';
  std::cout << "new_b_velocity: "
            << new_b_velocity.x << ", "
            << new_b_velocity.y << ", "
            << new_b_velocity.z << '\n';

  return std::make_pair(new_a_velocity, new_b_velocity);
}



void updateBothEntityInfoAfterTwoParticleCollision(
    const glm::dvec3& a_center
  , const double a_radius
  , pce::Motion& a_motion
  , const double a_mass
  , const glm::dvec3& b_center
  , const double b_radius
  , pce::Motion& b_motion
  , const double b_mass
  , const double total_elasticity
)
{
  std::pair<glm::dvec3, glm::dvec3> new_velocities = calculateVelocitiesAfterParticleCollision(
    a_center,
    b_center,
    a_mass,
    b_mass,
    a_motion,
    b_motion,
    a_radius,
    total_elasticity
  );

  a_motion.velocity = new_velocities.first;
  b_motion.velocity = new_velocities.second;
  
  a_motion.previous_resting_position = a_center;
  b_motion.previous_resting_position = b_center;

  a_motion.duration = 0.01;
  b_motion.duration = 0.01;
  
  a_motion.speed = pce3d::maths::calcMagV3(new_velocities.first);
  b_motion.speed = pce3d::maths::calcMagV3(new_velocities.second);

  a_motion.direction = glm::normalize(new_velocities.first);
  b_motion.direction = glm::normalize(new_velocities.second);
}



glm::dvec3 calculateRotationalVelocityOfPointOnObject(
    const glm::dvec3 object_center
  , const glm::dvec3 point_on_object
  , const double rotational_speed_degrees
  , const glm::dvec3 axis_of_rotation
)
/* 
find radius of the circle being drawn by the point's motion
- normalize the point to the object center
- rotate 180 degrees about axis, find the distance / 2.0
- given radians per second and radius, determine distance traveled per second
*/
{
  const glm::dvec3 adjusted_point = point_on_object - object_center;
  const glm::dvec3 rotated_point = pce::rotateVector3byAngleAxis(
    adjusted_point, 180.0, axis_of_rotation);
  
  const double travel_circle_radius = 0.5 * pce3d::maths::calculateDistanceBetweenVectors(
    adjusted_point, rotated_point);
  
  const double distance_traveled_per_second = pow(travel_circle_radius, 2.0) 
                                            * (rotational_speed_degrees / 180.0 * PI);

  const glm::dvec3 orthogonal_direction = pce::rotateVector3byAngleAxis(adjusted_point, 90.0, axis_of_rotation);
  
  return (orthogonal_direction * distance_traveled_per_second);
}




void updateBothEntityInfoAfterParticleComplexbodCollision(
    const glm::dvec3& collision_point
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , pce::Motion& b_motion
  , const double total_elasticity
)
{

}



}
}




#endif /* physics2Functions_cpp */
