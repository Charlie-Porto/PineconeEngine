#ifndef physics2Functions_hpp
#define physics2Functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
new physics functions
-----------------------------------------------------------------*/

#include <cmath>
#include <utility>
#include <algorithm>

#include "rigidObjectFunctions.hpp"
#include "physicsSupportFunctions.hpp"
#include "../objects/CollisionReport.hpp"

#include "../../maths/functions/quaternion_functions.hpp"
#include "../../maths/functions/vector_functions.hpp"
#include "../../maths/functions/plane_functions.hpp"

namespace pce3d {
namespace physics2 {

const double GRAVITY = -9.81;
const double PI = 3.14159265;


glm::dvec3 calculateParticlePositionGivenTime(
    const glm::dvec3& initial_position
  , const glm::dvec3& initial_velocity
  , double time_change
  , double gravitational_force_applied
  , double& duration
);


std::pair<glm::dvec3, glm::dvec3> calculateVelocitiesAfterParticleCollision(
    const glm::dvec3& a_center
  , const glm::dvec3& b_center
  , const double a_mass
  , const double b_mass
  , const pce::Motion& a_motion
  , const pce::Motion& b_motion
  , const double a_radius
  , const double total_elasticity
);


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
);


glm::dvec3 calculateRotationalVelocityOfPointOnObject(
    const glm::dvec3 object_center
  , const glm::dvec3 point_on_object
  , const double rotational_speed_degrees
  , const glm::dvec3 axis_of_rotation
);


void updateBothEntityInfoAfterParticleComplexbodCollision(
    const collision::CollisionReport& collision_report
  , const pce::RigidObject& a_rigid_object
  , pce::Motion& a_motion
  , const pce::RigidObject& b_rigid_object
  , const pce::Position& b_position
  , pce::Motion& b_motion
  , const double total_elasticity
);



}
}




#endif /* physics2Functions_hpp */
