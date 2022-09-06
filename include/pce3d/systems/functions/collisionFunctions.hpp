#ifndef collisionFunctions_hpp
#define collisionFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the Physics System detect collisions
-----------------------------------------------------------------*/

#include <utility>
#include <glm/vec3.hpp>
#include "physics2Functions.hpp"
#include "../../maths/functions/vector_functions.hpp"
#include "../../maths/functions/plane_functions.hpp"
#include "../../maths/functions/quaternion_functions.hpp"
#include "../objects/CollisionReport.hpp"

namespace pce3d {
namespace collision {

bool determineIfMovementVectorsIndicateCollision(
    glm::dvec3 a_movement_vector
  , glm::dvec3 b_movement_vector
  , const pce::RigidObject& a_rigid_object
  , const pce::RigidObject& b_rigid_object
  , const glm::dvec3& a_center
  , const glm::dvec3& b_center
);

std::pair<bool, glm::dvec3> determineIfParticlesAreCollidingAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
);

CollisionReport determineIfParticleIsCollidingWithComplexBodAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
);
 
CollisionReport determineIfComplexBodsAreCollidingAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const pce::Position& a_position
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
);

}
}





#endif /* collisionFunctions_hpp */
