#ifndef forge_cube_hpp
#define forge_cube_hpp

#include <vector>
#include "../../maths/functions/pce_psuedo_randomness.hpp"
#include "../rectangular_prism_forging.hpp"

namespace pce3d {
namespace forge {

uint32_t forgeCube(
    const glm::dvec3& center
  , const double side_length = 5.0
  , const glm::dvec3& velocity = glm::dvec3(0, 0, 0)
  , const glm::dvec3& axis = glm::dvec3(1, 0, 0)
  , const double spin_speed = 0.0
  , const std::vector<int>& color = getRandomColor()
  , const gravity = 0.0
  , const dead = false
);


}
}




#endif /* forge_cube_hpp */
