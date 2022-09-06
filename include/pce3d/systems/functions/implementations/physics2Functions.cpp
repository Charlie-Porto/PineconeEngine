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
    const collision::CollisionReport& collision_report
  , const pce::RigidObject& a_rigid_object
  , pce::Motion& a_motion
  , const pce::RigidObject& b_rigid_object
  , const pce::Position& b_position
  , pce::Motion& b_motion
  , const double total_elasticity
)
{
  glm::dvec3 a_force_direction_on_b = (collision_report.point_of_contact - a_rigid_object.vertices.at(1));

  /* calculate momentum vectors */
  glm::dvec3 a_momentum = a_motion.direction * a_motion.speed * a_rigid_object.mass;
  if (a_motion.speed == 0)
  {
    a_momentum = glm::normalize(a_force_direction_on_b);
  }

  if (collision_report.b_collision_type == collision::face)
  {
    const uint32_t& face_id = collision_report.b_collision_type_area_id;
    auto const& face_vertex_map = b_rigid_object.face_vertex_map;
    auto const& vertices = b_rigid_object.vertices;
    a_force_direction_on_b = pce3d::maths::calculateNormalVectorInDirectionOfPoint(
      vertices.at(face_vertex_map.at(face_id)[0]),
      vertices.at(face_vertex_map.at(face_id)[1]),
      vertices.at(face_vertex_map.at(face_id)[2]),
      b_position.actual_center_of_mass);
  }

  const double b_leverage = physics::calculateLeverageAtPointInDirection(
    b_position, collision_report.point_of_contact, a_force_direction_on_b);
  const double mass_at_point = b_rigid_object.mass * b_leverage;
  std::cout << "mass_at_point:" << mass_at_point << '\n';

  const glm::dvec3 b_travel = b_motion.speed == 0 
    ? -a_force_direction_on_b : b_motion.speed * b_motion.direction;
  const glm::dvec3 b_momentum = (((b_travel)
                              + calculateRotationalVelocityOfPointOnObject(
                                  b_position.actual_center_of_mass,
                                  collision_report.point_of_contact,
                                  b_motion.rotational_speed,
                                  b_motion.rotational_axis)) 
                              * (b_rigid_object.mass * b_leverage));
  
  std::cout << "b_momentum: "
            << b_momentum.x << ", "
            << b_momentum.y << ", "
            << b_momentum.z << '\n';
  /* calculate momentum vector component in direction of collision */
  const double a_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(a_force_direction_on_b, a_momentum));
  const double b_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(-a_force_direction_on_b, b_momentum));
  const double a_directness = (90.0 - a_directness_angle) / 90.0;
  const double b_directness = (90.0 - b_directness_angle) / 90.0;
  // if (a_directness > 0 && b_directness > 0)
  if (true)
  {
  std::cout << "a_directness: " << a_directness << '\n';
  std::cout << "b_directness: " << b_directness << '\n';
  
  const double momentums_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_momentum, b_momentum);
  std::cout << "momentums_angle: " << momentums_angle << '\n';
  
  const double adjusted_elasticity = std::max(total_elasticity, total_elasticity / pow(std::max(0.01, 180. / momentums_angle), 2.0)); 
  std::cout << "adjusted elasticity: " << adjusted_elasticity << '\n';
  const double a_velocity_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(a_momentum / a_rigid_object.mass)
                                                          * cos(a_directness_angle / 180.0 * PI)); 
  std::cout << "velocity at point: " <<  pce3d::maths::calcMagV3(b_momentum / mass_at_point)  << '\n';
  const double b_velocity_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(b_momentum / mass_at_point) 
                                                          * cos(b_directness_angle / 180.0 * PI)); 
  std::cout << "a_velocity_in_direction_of_collision_point: " << a_velocity_in_direction_of_collision_point << '\n';
  std::cout << "b_velocity_in_direction_of_collision_point: " << b_velocity_in_direction_of_collision_point << '\n';
  const glm::dvec3 normed_a_force_direction = glm::normalize(a_force_direction_on_b);
  const glm::dvec3 a_momentum_adjustment = (-normed_a_force_direction * b_velocity_in_direction_of_collision_point)
                                         * (mass_at_point / a_rigid_object.mass);
                                        //  * (a_rigid_object.mass / mass_at_point);
  const glm::dvec3 b_momentum_adjustment = (normed_a_force_direction * a_velocity_in_direction_of_collision_point)
                                         * (a_rigid_object.mass / mass_at_point);
                                        //  * (mass_at_point / a_rigid_object.mass);
  const glm::dvec3 new_a_velocity = (a_motion.direction * a_motion.speed 
                                  + (a_momentum_adjustment - b_momentum_adjustment)
                                  * (mass_at_point / (mass_at_point + a_rigid_object.mass)))
                                  * adjusted_elasticity;
  const glm::dvec3 new_b_velocity = (b_motion.direction * b_motion.speed 
                                  + (b_momentum_adjustment - a_momentum_adjustment)
                                  * (a_rigid_object.mass / (mass_at_point + a_rigid_object.mass)))
                                  * adjusted_elasticity;

  std::cout << "new_a_velocity: "
            << new_a_velocity.x << ", "
            << new_a_velocity.y << ", "
            << new_a_velocity.z << '\n';
  std::cout << "new_b_velocity: "
            << new_b_velocity.x << ", "
            << new_b_velocity.y << ", "
            << new_b_velocity.z << '\n';


  /* update first entity */
  a_motion.velocity = new_a_velocity;
  a_motion.previous_resting_position = a_rigid_object.vertices.at(1);
  // a_motion.speed = pce3d::maths::calcMagV3(new_a_velocity);
  // a_motion.direction = glm::normalize(new_a_velocity);

  a_motion.duration = 0.01;
  b_motion.duration = 0.01;
  
  physics::distributeAccelerationAtPointBetweenLinearAndRotational(
    b_position, b_rigid_object, b_motion, collision_report.point_of_contact, 
    b_leverage, new_b_velocity, a_force_direction_on_b);
      
  /* calculate exiting a direction */
  // glm::dvec3 a_exit_direction = pce::rotateVector3byAngleAxis(
    // -diremtion_vector, 180.0, a_force_direction_on_b);
  }

}





}
}




#endif /* physics2Functions_cpp */
