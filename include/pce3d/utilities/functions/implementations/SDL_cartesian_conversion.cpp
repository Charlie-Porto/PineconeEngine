#ifndef SDL_cartesian_conversion_cpp
#define SDL_cartesian_conversion_cpp


#include "../SDL_cartesian_conversion.hpp"

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




#endif /* SDL_cartesian_conversion_cpp */
