#ifndef collisionFunctions_hpp
#define collisionFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the Physics System detect collisions
-----------------------------------------------------------------*/

#include <utility>

namespace pce3d {
namespace collision {

std::pair<bool, glm::dvec3> determineIfParticlesAreCollidingAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
);

std::pair<bool, glm::dvec3> determineIfParticleIsCollidingWithComplexBodAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
);
 
std::pair<bool, glm::dvec3> determineIfComplexBodsAreCollidingAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
);

}
}





#endif /* collisionFunctions_hpp */
