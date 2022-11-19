#ifndef objectRenderFunctions_cpp
#define objectRenderFunctions_cpp

#include "../objectRenderFunctions.hpp"


namespace pce3d {
namespace render {


void renderSphereObject(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
  , pce::Position position
)
{
  if (rigid_object.vertex_distance_map.at(1) < 20.0) 
  {
    const std::vector<int> ncolor 
           = {int(surface.color[0] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel 
                                                               * Core3D::ORDINARY_ZOOM_INDEX_)),
              int(surface.color[1] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel 
                                                               * Core3D::ORDINARY_ZOOM_INDEX_)),
              int(surface.color[2] * shade.pixel_shade_map.at(position.center_of_mass_radar_pixel
                                                               * Core3D::ORDINARY_ZOOM_INDEX_)),
              255};

          pce::quickdraw::drawFilledCircleClean(position.center_of_mass_radar_pixel, rigid_object.radius * 800.0 / rigid_object.vertex_distance_map.at(1), ncolor);
        
  }
  else pce::render::renderFilledCircleShaded(shade.pixel_shade_map, surface.color);
}


void renderCylinder(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
);


void renderPolyhedron(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
);


}
}





#endif /* objectRenderFunctions_cpp */
