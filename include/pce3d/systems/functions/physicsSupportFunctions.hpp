#ifndef physicsSupportFunctions_hpp
#define physicsSupportFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
non-core physics functions to assist core physics functions
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "../../maths/functions/vector_functions.hpp"
#include "../../maths/functions/quaternion_functions.hpp"

namespace pce3d {
namespace physics2 {

void ensureParticleVelocityNotIntoObjectFace(
    const glm::dvec3& face_normal_vector
  , glm::dvec3& particle_velocity
);

}
}





#endif /* physicsSupportFunctions_hpp */
