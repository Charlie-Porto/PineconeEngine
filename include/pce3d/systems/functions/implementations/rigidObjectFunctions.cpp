#ifndef rigidObjectFunctions_cpp
#define rigidObjectFunctions_cpp

#include "../rigidObjectFunctions.hpp"


namespace pce3d {
namespace physics {

// const double PI = 3.14159265;

double calculateLeverageAtPointInDirection(
    const pce::Position& position
  , const glm::dvec3& point
  , const glm::dvec3& direction_of_force
)
{
  const glm::dvec3& point_to_center = position.actual_center_of_mass - point;
  const double angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(
    direction_of_force, point_to_center));
  // const double leverage = pow((90.0 - angle) / 90.0, 2.0);
  const double leverage = (90.0 - angle) / 90.0;
  return leverage;
}



void distributeAccelerationAtPointBetweenLinearAndRotational(
    const pce::Position& position
  , const pce::RigidObject& rigid_object
  , pce::Motion& motion
  , const glm::dvec3& point
  , const double leverage
  , const glm::dvec3& new_point_velocity
  , const glm::dvec3& normal_vect
  , const double allocated_momentum
  , const double mass_at_point
)
{
  std::cout << "allocated momentum: " << allocated_momentum << '\n';
  // const double momentum_remaining = abs(allocated_momentum) - maths::calcMagV3(new_linear_velocity * mass_at_point);
  double new_rotational_velocity_scalar = pce3d::maths::calcMagV3(new_point_velocity) * ((1.0 - leverage));
  std::cout <<  "new_rotational_velocity_scalar: " << new_rotational_velocity_scalar << '\n';
  // const double momentum_scalar_at_point = std::min(momentum_remaining, new_rotational_velocity_scalar * mass_at_point);
  const double momentum_scalar_at_point = std::min(allocated_momentum, new_rotational_velocity_scalar * mass_at_point);
  new_rotational_velocity_scalar = momentum_scalar_at_point / mass_at_point;

  const glm::dvec3 center_normed_point = point - position.actual_center_of_mass;

  // motion.velocity = new_linear_velocity;
  motion.previous_resting_position = position.actual_center_of_mass;
  // std::cout << "new_linear_velocity: "
            // << new_linear_velocity.x << ", "
            // << new_linear_velocity.y << ", "
            // << new_linear_velocity.z << '\n';

  /* adjust rotational axis */
  const glm::dvec3 magnified_rot_axis = motion.rotational_axis * motion.rotational_speed;
  glm::dvec3 magnified_new_axis = glm::cross(
    (position.actual_center_of_mass - point),
    (normal_vect)) * new_rotational_velocity_scalar;

  const glm::dvec3 new_axis_a = glm::normalize(magnified_rot_axis + magnified_new_axis);
  // const glm::dvec3 new_axis_b = glm::normalize(magnified_rot_axis - magnified_new_axis);

  // const glm::dvec3 estimated_next_point = point + glm::normalize(new_point_velocity);
  // const glm::dvec3 normalized_point = point - position.actual_center_of_mass;
  // const glm::dvec3 theoretical_rotated_point_a = pce::rotateVector3byAngleAxis(
    // normalized_point, 20.0, new_axis_a);
  // const glm::dvec3 theoretical_rotated_point_b = pce::rotateVector3byAngleAxis(
    // normalized_point, 20.0, new_axis_b);
  
  // const double distance_a = pce3d::maths::calculateDistanceBetweenVectors(
    // estimated_next_point, theoretical_rotated_point_a);
  // const double distance_b = pce3d::maths::calculateDistanceBetweenVectors(
    // estimated_next_point, theoretical_rotated_point_b);

  glm::dvec3 new_axis = -new_axis_a; 
  // if (distance_b < distance_a)
  // {
    // new_axis = new_axis_b;
  // }

  const double rotation_circle_radius = maths::calculateDistanceBetweenPointAndLine(
    -new_axis, new_axis, center_normed_point);

  std::cout << "rotation_circle_radius: " << rotation_circle_radius << '\n';
  const double theta_speed = new_rotational_velocity_scalar / ( rotation_circle_radius );

  std::cout << "previous_axis: "
            << motion.rotational_axis.x << ", "
            << motion.rotational_axis.y << ", "
            << motion.rotational_axis.z << '\n';
  motion.rotational_axis = new_axis;
  std::cout << "new_axis: "
            << new_axis.x << ", "
            << new_axis.y << ", "
            << new_axis.z << '\n';

  std::cout << "previous_rotational_velocity: " << motion.rotational_speed << '\n';
  // motion.rotational_speed = new_rotational_velocity;
  motion.rotational_speed = theta_speed;
  std::cout << "new_rotational_velocity: " << theta_speed << '\n';
  std::cout << "momentum_scalar_at_point: " << momentum_scalar_at_point << '\n';

  const double remaining = std::max(0.0, allocated_momentum - momentum_scalar_at_point);
  std::cout << "remaining: " << remaining << '\n';
  
  std::cout << "mass: " << rigid_object.mass <<'\n';
  const glm::dvec3 new_motion = glm::normalize(new_point_velocity) * (remaining / (rigid_object.mass));
  std::cout << "mass at point: " << rigid_object.mass * leverage << '\n';
  const double new_motion_mag = maths::calcMagV3(new_motion);
  std::cout << "new linear velocity scalar: " << new_motion_mag << '\n';
  std::cout << "previous linear velocity: "
            << motion.direction.x * motion.speed << ", "
            << motion.direction.y * motion.speed << ", "
            << motion.direction.z * motion.speed << '\n';
  std::cout << "new_linear_velocity: "
            << new_motion.x << ", "
            << new_motion.y << ", "
            << new_motion.z << '\n';

  if (!isnan(new_motion_mag))
  {
    motion.velocity = new_motion;
  }
}


}
}



#endif /* rigidObjectFunctions_cpp */
