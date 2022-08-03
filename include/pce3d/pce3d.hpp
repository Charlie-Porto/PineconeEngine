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
#include "components/force_component.cpp"
#include "components/motion_component.cpp"

/* systems */
#include "systems/CameraTransformSystem.cpp"
#include "systems/RadarSystem.cpp"
#include "systems/RenderSystem.cpp"
#include "systems/OrderForRenderSystem.cpp"
#include "systems/CameraOperatorSystem.cpp"
#include "systems/ShadeSystem.cpp"
#include "systems/SpaceMapSystem.cpp"
#include "systems/PhysicsSystem.cpp"


extern ControlPanel control;

namespace pce3d {

class Core3D {
public:
  Core3D(const glm::dvec3 light_flow_direction = glm::dvec3(-0.1, -0.9, -0.2), 
         const double lense_curve_index = 0.1,
         const double ordinary_zoom_index = 15.0);

  void RegisterCoreComponents();
  void RegisterCoreSystems();
  void PrepareForAllSystemsGo();
  void UpdateCore3D();

  /* pce3d statics */
  static glm::dvec3 LIGHT_FLOW_DIRECTION_;
  static double LENSE_CURVE_;
  static double ORDINARY_ZOOM_INDEX_;

private:
  Camera camera_;
  
  /* systems */
  std::shared_ptr<pce3d::CameraTransformSystem> camera_transform_system_;
  std::shared_ptr<pce3d::CameraOperatorSystem> camera_operator_system_;
  std::shared_ptr<pce3d::RadarSystem> radar_system_;
  std::shared_ptr<pce3d::RenderSystem> render_system_;
  std::shared_ptr<pce3d::ShadeSystem> shade_system_;
  std::shared_ptr<pce3d::OrderForRenderSystem> render_order_system_;
  std::shared_ptr<pce3d::SpaceMapSystem> space_map_system_;
  std::shared_ptr<pce3d::PhysicsSystem> physics_system_;


};


}



#endif /* pce3d_hpp */
