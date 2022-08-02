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

glm::vec2 convertCartesianCoordinatesToSDL(glm::vec2 point);

glm::vec2 convertSDLCoordinatesToCartesian(glm::vec2 point);

}
}




#endif /* SDL_cartesian_conversion_hpp */
