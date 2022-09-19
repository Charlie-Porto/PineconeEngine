#ifndef forge_cube_cpp
#define forge_cube_cpp

#include "../forge_cube.hpp"


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
)
{
  return forgeRectPrismEntity(
    side_length, side_length, side_length,
    center, 
    0.0, glm::dvec3(0, 0, 0),
    color,
    true,
    gravity,
    velocity,
    axis,
    spin_speed
  );
}


}
}




#endif /* forge_cube_cpp */
