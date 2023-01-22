#ifndef shadeFunctions_hpp
#define shadeFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the shade system
-----------------------------------------------------------------*/

#include <iostream>
#include <cmath>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "radarFunctions.hpp"
#include "../../maths/functions/sphere_functions.hpp"
#include "../../maths/functions/rounding_functions.hpp"
#include "../../utilities/functions/SDL_cartesian_conversion.hpp"
#include "../../pce3d.hpp"

extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
namespace shade {

using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;
using SurfaceZoneBrightnessMap = std::unordered_map<glm::dvec3, double>;

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec);

double calculatePixelBrightness(const glm::dvec3& light_direction,
                                const glm::dvec3& sphere_center,
                                const double sphere_radius,
                                const glm::dvec2& pixel);

void calculateFaceBrightnessForSpherePixels(const glm::dvec3& light_direction,
                                            const glm::dvec3& sphere_center,
                                            const double sphere_radius,
                                            const PixelMap& outline_pixels,
                                            pce::PixelShadeMap& pixel_shades);

void calculateBrightnessForSpherePixelsSmart(const glm::dvec3& light_direction,
                                             const glm::dvec3& sphere_center,
                                             const double sphere_radius,
                                             const int distance_from_camera,
                                             const PixelMap& outline_pixels,
                                             pce::PixelShadeMap& pixel_shades,
                                             double& ratio);

void mapSpherePixelsToBrightnessZones(
    pce::PixelShadeMap& pixel_shade_map
  , const PixelMap& outline_pixels
  , const SurfaceZoneBrightnessMap& camera_transformed_surface_zone_brightness_map
  , const glm::dvec3& light_direction
  , const glm::dvec3& sphere_center
  , const double sphere_radius
  , const double zone_granularity
);

double calculateBrightnessAtSphereSurfaceZone(
    const glm::dvec3& zone
  , const glm::dvec3& sphere_center
  , const double radius 
  , const glm::dvec3& light_direction
);

std::vector<glm::dvec3> getSurfaceZonesAtSpherePoint(
    const glm::dvec3& surface_point
  , const glm::dvec3& sphere_center
  , const double radius
  , const double zone_granularity_constant
);

void doSphereLightScan(
    const glm::dvec3& sphere_center
  , const double radius
  , SurfaceZoneBrightnessMap& surface_zone_brightness_map 
  , const glm::dvec3& light_direction
  , const double zone_granularity
);


}
}




#endif /* shadeFunctions_hpp */
