#ifndef pce3d_hpp
#define pce3d_hpp

#include <pcecs/pcecs.hpp>

#include "utilities/camera.cpp" 

/* components */
#include "components/rigid_object_component.cpp"
#include "components/position_component.cpp"
#include "components/local_rotation_component.cpp"
#include "components/surface_component.cpp"
#include "components/face_shade_component.cpp"

/* systems */
#include "systems/CameraTransformSystem.cpp"
#include "systems/RadarSystem.cpp"
#include "systems/RenderSystem.cpp"
#include "systems/OrderForRenderSystem.cpp"
#include "systems/CameraOperatorSystem.cpp"
#include "systems/ShadeSystem.cpp"


extern ControlPanel control;
namespace pce3d {

class Core3D {
public:
  Core3D();

  void RegisterCoreComponents();
  void RegisterCoreSystems();
  void UpdateCore3D();

private:
  Camera camera_;
  
  /* systems */
  std::shared_ptr<pce3d::CameraTransformSystem> camera_transform_system_;
  std::shared_ptr<pce3d::CameraOperatorSystem> camera_operator_system_;
  std::shared_ptr<pce3d::RadarSystem> radar_system_;
  std::shared_ptr<pce3d::RenderSystem> render_system_;
  std::shared_ptr<pce3d::ShadeSystem> shade_system_;
  std::shared_ptr<pce3d::OrderForRenderSystem> render_order_system_;

   

};


}



#endif /* pce3d_hpp */
