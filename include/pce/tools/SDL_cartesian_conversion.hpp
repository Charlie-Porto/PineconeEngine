#ifndef SDL_cartesian_conversion_hpp
#define SDL_cartesian_conversion_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to convert SDL coordinates to cartesian coordinates, and vice versa.
-----------------------------------------------------------------*/

#include <glm/vec2.hpp>
#include <ezprint.cpp>

namespace pce {
namespace convert {

glm::vec2 convertCartesianCoordinatesToSDL(glm::vec2 point) {
  const int sdl_x = point.x + int(1000/2);
  const int sdl_y = -point.y + int(672/2);
  return glm::vec2(sdl_x, sdl_y);
}


glm::vec2 convertSDLCoordinatesToCartesian(glm::vec2 point) {
  const int cart_x = point.x - int(1000/2);
  const int cart_y = -point.y + int(672/2);
  return glm::vec2(cart_x, cart_y);
}

}
}




#endif /* SDL_cartesian_conversion_hpp */
