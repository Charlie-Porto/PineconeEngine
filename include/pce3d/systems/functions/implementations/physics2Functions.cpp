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
  , const bool b_is_deadbod
)
{
  if (b_is_deadbod)
  {
    updateParticleInfoAfterComplexDeadbodCollsion(
      collision_report,
      a_rigid_object,
      a_motion,
      total_elasticity
    );
  }
  else
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
  // std::cout << "mass_at_point: " << mass_at_point << '\n';
  // std::cout << "particle_mass: " << a_rigid_object.mass << '\n';

  const glm::dvec3 b_travel = b_motion.speed == 0 
    ? -a_force_direction_on_b : b_motion.speed * b_motion.direction;
  const glm::dvec3 b_momentum = (((b_travel)
                              + calculateRotationalVelocityOfPointOnObject(
                                  b_position.actual_center_of_mass,
                                  collision_report.point_of_contact,
                                  b_motion.rotational_speed,
                                  b_motion.rotational_axis)) 
                              * (b_rigid_object.mass * b_leverage));
  
  std::cout << "a_force_direction_on_b:"
            << a_force_direction_on_b.x << ", "
            << a_force_direction_on_b.y << ", "
            << a_force_direction_on_b.z << '\n';
  std::cout << "a_momentum: "
            << a_momentum.x << ", "
            << a_momentum.y << ", "
            << a_momentum.z << '\n';
  std::cout << "b_momentum: "
            << b_momentum.x << ", "
            << b_momentum.y << ", "
            << b_momentum.z << '\n';
  /* calculate momentum vector component in direction of collision */
  double a_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(a_force_direction_on_b, a_momentum));
  if (isnan(a_directness_angle)) { a_directness_angle = 0.0; }
  double b_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(-a_force_direction_on_b, b_momentum));
  if (isnan(b_directness_angle)) { b_directness_angle = 0.0; }

  const double a_directness = abs((90.0 - a_directness_angle) / 90.0);
  const double b_directness = abs((90.0 - b_directness_angle) / 90.0);
  // if (a_directness > 0 && b_directness > 0)
  std::cout << "a_directness_angle: " << a_directness_angle << '\n';
  std::cout << "b_directnessa_angle: " << b_directness_angle << '\n';
  std::cout << "a_directness: " << a_directness << '\n';
  std::cout << "b_directness: " << b_directness << '\n';
  
  const double momentums_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_momentum, b_momentum);
  
  const double adjusted_elasticity = std::min(1.0, std::max(total_elasticity, total_elasticity / pow(std::max(0.01, 180 / momentums_angle), 2.0))); 
  std::cout << "adjusted elasticity: " << adjusted_elasticity << '\n';
  const double a_momentum_in_direction_of_collision_point = double(abs(pce3d::maths::calcMagV3(a_momentum)
                                                          * cos(a_directness_angle / 180.0 * PI))); 
  std::cout << "velocity at point: " <<  pce3d::maths::calcMagV3(b_momentum / mass_at_point)  << '\n';
  const double b_momentum_in_direction_of_collision_point = double(abs(pce3d::maths::calcMagV3(b_momentum) 
                                                          * cos(b_directness_angle / 180.0 * PI))); 
  std::cout << "a_momentum_in_direction_of_collision_point: " << a_momentum_in_direction_of_collision_point << '\n';
  std::cout << "b_momentum_in_direction_of_collision_point: " << b_momentum_in_direction_of_collision_point << '\n';
  const glm::dvec3 normed_a_force_direction = glm::normalize(a_force_direction_on_b);
  std::cout << "normed_a_force_direction: "
            << normed_a_force_direction.x << ", "
            << normed_a_force_direction.y << ", "
            << normed_a_force_direction.z << '\n';
  const glm::dvec3 a_momentum_adjustment = (-normed_a_force_direction * b_momentum_in_direction_of_collision_point);
                                        //  * (mass_at_point / a_rigid_object.mass);
                                        //  * (a_rigid_object.mass / mass_at_point);
  const glm::dvec3 b_momentum_adjustment = (normed_a_force_direction * a_momentum_in_direction_of_collision_point);
                                        //  * (a_rigid_object.mass / mass_at_point);
                                        //  * (mass_at_point / a_rigid_object.mass);
  std::cout << "a_momentum_adjustment: "
            << a_momentum_adjustment.x << ", "
            << a_momentum_adjustment.y << ", "
            << a_momentum_adjustment.z << '\n';
  std::cout << "b_momentum_adjustment: "
            << b_momentum_adjustment.x << ", "
            << b_momentum_adjustment.y << ", "
            << b_momentum_adjustment.z << '\n';
  
  glm::dvec3 new_a_velocity = (a_motion.direction * a_motion.speed 
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

  
  if (collision_report.b_collision_type == collision::face)
  {
    ensureParticleVelocityNotIntoObjectFace(-a_force_direction_on_b, new_a_velocity);
  }
  /* update first entity */
  a_motion.velocity = new_a_velocity / a_rigid_object.mass;
  // std::cout << "new_a_velocity: "
            // << new_a_velocity.x << ", "
            // << new_a_velocity.y << ", "
            // << new_a_velocity.z << '\n';
  a_motion.previous_resting_position = a_rigid_object.vertices.at(1);
  // a_motion.speed = pce3d::maths::calcMagV3(new_a_velocity);
  // a_motion.direction = glm::normalize(new_a_velocity);

  a_motion.duration = 0.01;
  if (!b_is_deadbod)
  {
    b_motion.duration = 0.01;
    physics::distributeAccelerationAtPointBetweenLinearAndRotational(
      b_position, b_rigid_object, b_motion, collision_report.point_of_contact, 
      b_leverage, new_b_velocity, a_force_direction_on_b,
      maths::calcMagV3(new_b_velocity), mass_at_point);
  }

  }
}



void updateParticleInfoAfterComplexDeadbodCollsion(
    const collision::CollisionReport& collision_report
  , const pce::RigidObject& a_rigid_object
  , pce::Motion& a_motion
  , const double total_elasticity
)
{
  const glm::dvec3 complexbod_point_normal_vect = a_rigid_object.vertices.at(1) - collision_report.point_of_contact; 
  const double directness_angle = maths::calculateAngleDegreesBetweenVectors(
    -complexbod_point_normal_vect, a_motion.direction);

  double adjusted_elasticity = std::max(total_elasticity, total_elasticity / std::max(0.01, directness_angle / 180.0)); 
  adjusted_elasticity = std::min(1.0, adjusted_elasticity);
  
  const double new_speed = a_motion.speed * adjusted_elasticity;

  const glm::dvec3 new_direction = glm::normalize(pce::rotateVector3byAngleAxis(
    -a_motion.direction, 180.0, complexbod_point_normal_vect));
  
  const glm::dvec3 new_velocity = new_direction * new_speed;
  // const glm::dvec3 prev_velocity = a_motion.direction * a_motion.speed;
  a_motion.previous_resting_position = a_rigid_object.vertices.at(1);
  a_motion.duration = 0.01;
  
  a_motion.velocity = new_velocity;
}




void updateBothEntityInfoAfterComplexbodComplexbodCollision(
    const collision::CollisionReport& collision_report
  , const pce::RigidObject& a_rigid_object
  , pce::Motion& a_motion
  , const pce::Position& a_position
  , const pce::RigidObject& b_rigid_object
  , pce::Motion& b_motion
  , const pce::Position& b_position
  , const double total_elasticity
  , const bool b_is_deadbod
  , const bool a_is_deadbod
)
{
  /* get points of collision */
  glm::dvec3 a_closest_point = collision_report.point_of_contact;
  glm::dvec3 b_closest_point = collision_report.point_of_contact;

  if (collision_report.a_collision_type == collision::vertex)
  {
    a_closest_point = a_rigid_object.vertices.at(collision_report.a_collision_type_area_id);
  }
  else if (collision_report.a_collision_type == collision::edge)
  {
    a_closest_point = maths::findClosestPointOnVec3LineToVec3(
      a_rigid_object.vertices.at(a_rigid_object.edges.at(collision_report.a_collision_type_area_id).first),
      a_rigid_object.vertices.at(a_rigid_object.edges.at(collision_report.a_collision_type_area_id).second),
      collision_report.point_of_contact);
  }
  else if (collision_report.a_collision_type == collision::face)
  {
    a_closest_point = maths::calculateClosestPointInPlaneToPoint(
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(collision_report.a_collision_type_area_id)[0]),
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(collision_report.a_collision_type_area_id)[1]),
      a_rigid_object.vertices.at(a_rigid_object.face_vertex_map.at(collision_report.a_collision_type_area_id)[2]),
      collision_report.point_of_contact);
  }
  if (collision_report.b_collision_type == collision::vertex)
  {
    b_closest_point = b_rigid_object.vertices.at(collision_report.b_collision_type_area_id);
  }
  else if (collision_report.b_collision_type == collision::edge)
  {
    b_closest_point = maths::findClosestPointOnVec3LineToVec3(
      b_rigid_object.vertices.at(b_rigid_object.edges.at(collision_report.b_collision_type_area_id).first),
      b_rigid_object.vertices.at(b_rigid_object.edges.at(collision_report.b_collision_type_area_id).second),
      collision_report.point_of_contact);
  }
  else if (collision_report.b_collision_type == collision::face)
  {
    b_closest_point = maths::calculateClosestPointInPlaneToPoint(
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(collision_report.b_collision_type_area_id)[0]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(collision_report.b_collision_type_area_id)[1]),
      b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(collision_report.b_collision_type_area_id)[2]),
      collision_report.point_of_contact);
  }
  

  // std::cout << "a_closest_point: "
  //           << a_closest_point.x << ", "
  //           << a_closest_point.y << ", "
  //           << a_closest_point.z << '\n';
  // std::cout << "b_closest_point: "
  //           << b_closest_point.x << ", "
  //           << b_closest_point.y << ", "
  //           << b_closest_point.z << '\n';
  glm::dvec3 a_force_direction_on_b = collision_report.point_of_contact - a_closest_point;
  if (a_force_direction_on_b == glm::dvec3(0, 0, 0))
  {
    a_force_direction_on_b = -(collision_report.point_of_contact - b_closest_point);
  }
  // std::cout << "a_force_direction_on_b: "
  //           << a_force_direction_on_b.x << ", "
  //           << a_force_direction_on_b.y << ", "
  //           << a_force_direction_on_b.z << '\n';


  
  /* calculate each leverage at point and mass at point */
  const double a_leverage = abs(physics::calculateLeverageAtPointInDirection(
    a_position, a_closest_point, -a_force_direction_on_b));
  const double a_mass_at_point = a_rigid_object.mass * a_leverage;
  const double b_leverage = abs(physics::calculateLeverageAtPointInDirection(
    b_position, b_closest_point, a_force_direction_on_b));
  const double b_mass_at_point = b_rigid_object.mass * b_leverage;
  
  std::cout << "a_leverage: " << a_leverage << '\n';
  std::cout << "b_leverage: " << b_leverage << '\n';
  std::cout << "a_mass_at_point: " << a_mass_at_point << '\n';
  std::cout << "b_mass_at_point: " << b_mass_at_point << '\n';

  /* calculate each momentum at point */
  glm::dvec3 a_travel = a_motion.speed == 0 
    ? a_force_direction_on_b : a_motion.speed * a_motion.direction;
  glm::dvec3 a_linear_momentum = a_travel * a_mass_at_point;
  glm::dvec3 a_rotational_momentum = calculateRotationalVelocityOfPointOnObject(
    a_position.actual_center_of_mass,
    a_closest_point,
    a_motion.rotational_speed,
    a_motion.rotational_axis) * a_mass_at_point;
  
  glm::dvec3 a_momentum = a_linear_momentum + a_rotational_momentum;
  
  glm::dvec3 b_travel = b_motion.speed == 0 
    ? -a_force_direction_on_b : b_motion.speed * b_motion.direction;

  glm::dvec3 b_linear_momentum = b_travel * b_mass_at_point;
  glm::dvec3 b_rotational_momentum = calculateRotationalVelocityOfPointOnObject(
    b_position.actual_center_of_mass,
    b_closest_point,
    b_motion.rotational_speed,
    b_motion.rotational_axis) * b_mass_at_point;
  
  glm::dvec3 b_momentum = b_linear_momentum + b_rotational_momentum;
  
  // const glm::dvec3 a_momentum = (((a_travel)
  //                             + calculateRotationalVelocityOfPointOnObject(
  //                                 a_position.actual_center_of_mass,
  //                                 a_closest_point,
  //                                 a_motion.rotational_speed,
  //                                 a_motion.rotational_axis)) 
  //                             * a_mass_at_point);
  
  // const glm::dvec3 b_travel = b_motion.speed == 0 
  //   ? -a_force_direction_on_b : b_motion.speed * b_motion.direction;
  // const glm::dvec3 b_momentum = (((b_travel)
  //                             + calculateRotationalVelocityOfPointOnObject(
  //                                 b_position.actual_center_of_mass,
  //                                 collision_report.point_of_contact,
  //                                 b_motion.rotational_speed,
  //                                 b_motion.rotational_axis)) 
                              // * b_mass_at_point);
                            
  const glm::dvec3 total_starting_linear_momentum = a_linear_momentum + b_linear_momentum;
  const double magnitude_starting_linear_momentum = pce3d::maths::calcMagV3(total_starting_linear_momentum);
  const glm::dvec3 total_starting_rotational_momentum = a_rotational_momentum + b_rotational_momentum;
  const double magnitude_starting_rotational_momentum = pce3d::maths::calcMagV3(total_starting_rotational_momentum);
  const glm::dvec3 total_starting_momentum = a_momentum + b_momentum;
  const double magnitude_starting_momentum = pce3d::maths::calcMagV3(total_starting_momentum);


  /* calculate directness angles and directness */
  const double a_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(a_force_direction_on_b, a_momentum));
  const double b_directness_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(-a_force_direction_on_b, b_momentum));
  // const double a_directness = (90.0 - a_directness_angle) / 90.0;
  // const double b_directness = (90.0 - b_directness_angle) / 90.0;

  /* calculate momentums angle and adjusted elasticity */
  const double momentums_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_momentum, b_momentum);
  const double adjusted_elasticity = std::min(1.0, std::max(total_elasticity, total_elasticity / pow(std::max(0.01, 180 / momentums_angle), 2.0))); 
  std::cout << "adjusted elasticity: " << adjusted_elasticity << '\n';

  /* calculate momentum adjustments */
  // const double a_velocity_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(a_momentum / a_mass_at_point)
  const double a_momentum_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(a_momentum)
                                                          * cos(a_directness_angle / 180.0 * PI)); 
  // const double b_velocity_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(b_momentum / b_mass_at_point) 
  const double b_momentum_in_direction_of_collision_point = abs(pce3d::maths::calcMagV3(b_momentum)
                                                          * cos(b_directness_angle / 180.0 * PI)); 

  const glm::dvec3 normed_a_force_direction = glm::normalize(a_force_direction_on_b);
  const glm::dvec3 a_momentum_adjustment = (-normed_a_force_direction * b_momentum_in_direction_of_collision_point);
  const glm::dvec3 b_momentum_adjustment = (normed_a_force_direction * a_momentum_in_direction_of_collision_point);
   

  /* calculate updated velocities at points */
  glm::dvec3 new_a_momentum = ((a_motion.direction * a_motion.speed) 
                            + (a_momentum_adjustment - b_momentum_adjustment)
                            // + ((a_momentum_adjustment - b_momentum_adjustment) / a_mass_at_point)
                            * (b_mass_at_point / (b_mass_at_point + a_mass_at_point)))
                            * adjusted_elasticity;
  glm::dvec3 new_b_momentum = ((b_motion.direction * b_motion.speed)
                            + (b_momentum_adjustment - a_momentum_adjustment)
                            // + ((b_momentum_adjustment - a_momentum_adjustment) / b_mass_at_point)
                            * (a_mass_at_point / (b_mass_at_point + a_mass_at_point)))
                            * adjusted_elasticity;

  const double a_momentum_scalar = maths::calcMagV3(new_a_momentum);
  const double b_momentum_scalar = maths::calcMagV3(new_b_momentum);
  const double total_momentum_scalar = (a_momentum_scalar + b_momentum_scalar);
  
  const double a_allocation = a_momentum_scalar / total_momentum_scalar;
  const double b_allocation = b_momentum_scalar / total_momentum_scalar;

  std::cout << "a_allocation: " << a_allocation << '\n';
  std::cout << "b_allocation: " << b_allocation << '\n';

  std::cout << "total momentum: " << total_momentum_scalar << '\n';

  // new_a_momentum = glm::normalize(new_a_momentum) * (magnitude_starting_momentum * a_allocation);
  // new_b_momentum = glm::normalize(new_b_momentum) * (magnitude_starting_momentum * b_allocation);

  /* distribute updated velocities at points between linear and rotational motion */
  // if (collision_report.b_collision_type == collision::face)
  // {
  //   ensureParticleVelocityNotIntoObjectFace(-a_force_direction_on_b, new_a_velocity);
  // }
  // if (collision_report.a_collision_type == collision::face)
  // {
  //   ensureParticleVelocityNotIntoObjectFace(a_force_direction_on_b, new_b_velocity);
  // }

  glm::dvec3 new_a_velocity = new_a_momentum / a_mass_at_point;
  glm::dvec3 new_b_velocity = new_b_momentum / b_mass_at_point;

  const double a_allocated_conserved_momentum = std::min(magnitude_starting_momentum * a_allocation, a_momentum_scalar);
  const double b_allocated_conserved_momentum = std::min(magnitude_starting_momentum * b_allocation, b_momentum_scalar);

  std::cout << "a_allocated_conserved_momentum: " << a_allocated_conserved_momentum << '\n';
  std::cout << "b_allocated_conserved_momentum: " << b_allocated_conserved_momentum << '\n';

  b_motion.duration = 0.01;
  physics::distributeAccelerationAtPointBetweenLinearAndRotational(
    b_position, b_rigid_object, b_motion, b_closest_point, 
    b_leverage, new_b_velocity, -a_force_direction_on_b,
    b_allocated_conserved_momentum, b_mass_at_point);

  a_motion.duration = 0.01;
  physics::distributeAccelerationAtPointBetweenLinearAndRotational(
    a_position, a_rigid_object, a_motion, a_closest_point, 
    a_leverage, new_a_velocity, a_force_direction_on_b,
    a_allocated_conserved_momentum, a_mass_at_point);
  
  /* all the rest of this function is for logging while devving */
  a_travel = a_motion.velocity;
  a_linear_momentum = a_travel * a_mass_at_point;
  a_rotational_momentum = calculateRotationalVelocityOfPointOnObject(
    a_position.actual_center_of_mass,
    a_closest_point,
    a_motion.rotational_speed,
    a_motion.rotational_axis) * a_mass_at_point;
  
  a_momentum = a_linear_momentum + a_rotational_momentum;
  
  b_travel = b_motion.velocity;
  b_linear_momentum = b_travel * b_mass_at_point;
  b_rotational_momentum = calculateRotationalVelocityOfPointOnObject(
    b_position.actual_center_of_mass,
    b_closest_point,
    b_motion.rotational_speed,
    b_motion.rotational_axis) * b_mass_at_point;
  
  b_momentum = b_linear_momentum + b_rotational_momentum;
                              
  const glm::dvec3 total_ending_linear_momentum = a_linear_momentum + b_linear_momentum;
  const double magnitude_ending_linear_momentum = pce3d::maths::calcMagV3(total_ending_linear_momentum);
  const glm::dvec3 total_ending_rotational_momentum = a_rotational_momentum + b_rotational_momentum;
  const double magnitude_ending_rotational_momentum = pce3d::maths::calcMagV3(total_ending_rotational_momentum);
  const glm::dvec3 total_ending_momentum = a_momentum + b_momentum;
  const double magnitude_ending_momentum = pce3d::maths::calcMagV3(total_ending_momentum);

  std::cout << "STARTING LINEAR MOMENTUM SCALAR: " << magnitude_starting_linear_momentum << '\n';
  std::cout << "ENDING LINEAR MOMENTUM SCALAR: " << magnitude_ending_linear_momentum << '\n';
  
  std::cout << "STARTING ROTATIONAL MOMENTUM SCALAR: " << magnitude_starting_rotational_momentum << '\n';
  std::cout << "ENDING ROTATIONAL MOMENTUM SCALAR: " << magnitude_ending_rotational_momentum << '\n';

  std::cout << "STARTING MOMENTUM SCALAR: " << magnitude_starting_momentum << '\n';
  std::cout << "ENDING MOMENTUM SCALAR: " << magnitude_ending_momentum << '\n';
}


}
}




#endif /* physics2Functions_cpp */
