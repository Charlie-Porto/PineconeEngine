#ifndef RadarSystem_cpp
#define RadarSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to reliably detect the positions of entities
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

#include <ezprint.cpp>
#include <vezprint.cpp>
#include <math_objects/LineVectorForm.cpp>


#include "subsystems/spacePixelConversionFunctions.cpp"
#include "subsystems/simpleDrawingFunctions.cpp"
#include "../System.cpp"

extern ControlPanel control;

namespace pce{
class RadarSystem : public ISystem {
public:

void UpdateEntities() {
  // ezp::print_item("updating radar");
  pce::quickdraw::DrawCrossHairs();
  for (auto const& entity : entities) {
    auto const& position = control.GetComponent<pce::Position>(entity);
    auto& radar = control.GetComponent<pce::Radar>(entity);

    radar.view_sphere_hitpoint = glm::normalize((position.rotated));
    radar.hitpoint_corresponding_pixel = -pce::pix_map::fastconvertPointOnViewSphereToPixel(
      radar.view_sphere_hitpoint, glm::dvec3(0, 0, 0)
    ) / global_const::pixel_squish_ratio;
    radar.distance_from_player = sqrt(glm::dot(position.rotated, position.rotated));
    
    
    // pce::quickdraw::drawPixelAtVec2(radar.hitpoint_corresponding_pixel, {170, 200, 50, 255});


   
  }
}

private:
};
}
#endif /* RadarSystem_cpp */
