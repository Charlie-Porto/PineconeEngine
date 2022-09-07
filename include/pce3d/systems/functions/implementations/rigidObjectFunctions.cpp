#ifndef rigidObjectFunctions_cpp
#define rigidObjectFunctions_cpp

#include "../rigidObjectFunctions.hpp"


namespace pce3d {
namespace physics {

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
)
{
  const glm::dvec3 new_linear_velocity = new_point_velocity * leverage;
  const double new_rotational_velocity = pce3d::maths::calcMagV3(new_point_velocity) * (1.0 - leverage);
  motion.velocity = new_linear_velocity;
  motion.previous_resting_position = position.actual_center_of_mass;
  std::cout << "new_linear_velocity: "
            << new_linear_velocity.x << ", "
            << new_linear_velocity.y << ", "
            << new_linear_velocity.z << '\n';

  if (leverage < 0.99)
  {
    /* adjust rotational axis */
    const glm::dvec3 magnified_rot_axis = motion.rotational_axis * motion.rotational_speed;
    glm::dvec3 magnified_new_axis = glm::cross(
      (position.actual_center_of_mass - point),
      (normal_vect)) * new_rotational_velocity;

    const glm::dvec3 new_axis_a = glm::normalize(magnified_rot_axis + magnified_new_axis);
    const glm::dvec3 new_axis_b = glm::normalize(magnified_rot_axis + -magnified_new_axis);
    

    const glm::dvec3 estimated_next_point = point + glm::normalize(new_point_velocity);
    const glm::dvec3 normalized_point = point - position.actual_center_of_mass;
    const glm::dvec3 theoretical_rotated_point_a = pce::rotateVector3byAngleAxis(
      normalized_point, 2.0, new_axis_a);
    const glm::dvec3 theoretical_rotated_point_b = pce::rotateVector3byAngleAxis(
      normalized_point, 2.0, new_axis_b);
    
    const double distance_a = pce3d::maths::calculateDistanceBetweenVectors(
      estimated_next_point, theoretical_rotated_point_a);
    const double distance_b = pce3d::maths::calculateDistanceBetweenVectors(
      estimated_next_point, theoretical_rotated_point_b);

    glm::dvec3 new_axis = new_axis_a; 
    if (distance_b < distance_a)
    {
      new_axis = new_axis_b;
    }

    motion.rotational_axis = new_axis;
  std::cout << "new_axis: "
            << new_axis.x << ", "
            << new_axis.y << ", "
            << new_axis.z << '\n';


    motion.rotational_speed = new_rotational_velocity;
  std::cout << "new_rotational_velocity: "
            << new_rotational_velocity << '\n';
  }
}


}
}



#endif /* rigidObjectFunctions_cpp */
