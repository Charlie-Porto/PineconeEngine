#ifndef rigidObjectFunctions_hpp
#define rigidObjectFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for rigid objects that assist the Physics System
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include "../../maths/functions/vector_functions.hpp"

namespace pce3d {
namespace physics {

double calculateLeverageAtPointInDirection(
    const pce::Position& position
  , const glm::dvec3& point
  , const glm::dvec3& direction_of_force
);


void distributeAccelerationAtPointBetweenLinearAndRotational(
    const pce::Position& position
  , const pce::RigidObject& rigid_object
  , pce::Motion& motion
  , const glm::dvec3& point
  , const double leverage
  , const glm::dvec3& new_point_velocity
  , const glm::dvec3& normal_vect
);

}
}





#endif /* rigidObjectFunctions_hpp */
