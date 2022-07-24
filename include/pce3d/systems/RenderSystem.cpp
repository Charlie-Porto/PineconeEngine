#ifndef RenderSystem_cpp
#define RenderSystem_cpp

#include <pcecs/ecs/System.cpp>
#include "OrderForRenderSystem.cpp"
#include "../utilities/functions/quickdraw.hpp"

extern ControlPanel control;

namespace pce3d {
class RenderSystem : public ISystem {
public:

  void UpdateEntities() {
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto const& surface = control.GetComponent<pce::Surface>(entity);
      pce::quickdraw::drawCircle(position.center_of_mass_radar_pixel, 10.0/(position.distance_from_camera/3.0), surface.color, 10.0);

      for (auto const& [id, pixel] : rigid_object.vertex_pixels) {
        pce::quickdraw::drawCircle(pixel, 10.0/(position.distance_from_camera/3.0), surface.color, 10.0);
      }


    }
  }

private:
};
}
#endif /* RenderSystem_cpp */
