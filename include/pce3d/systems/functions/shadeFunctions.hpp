#ifndef shadeFunctions_hpp
#define shadeFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the shade system
-----------------------------------------------------------------*/

#include <iostream>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "radarFunctions.hpp"
#include "../../maths/functions/sphere_functions.hpp"
#include "../../utilities/functions/SDL_cartesian_conversion.hpp"
#include "../../pce3d.hpp"

namespace pce3d {
namespace shade {

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec);
/* returns a double between 0 and 1 */
/* 0 = black */
/* 1 = pure natural color */

using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;


void calculateFaceBrightnessForSpherePixels(const glm::dvec3& light_direction,
                                            const glm::dvec3& sphere_center,
                                            const double sphere_radius,
                                            const PixelMap& outline_pixels,
                                            pce::PixelShadeMap& pixel_shades);

}
}




#endif /* shadeFunctions_hpp */
