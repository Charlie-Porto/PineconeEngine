#ifndef Core3D_cpp
#define Core3D_cpp

#include "Core3D.hpp"


/***** not yet included *****/

/* maths */
#include "maths/functions/implementations/quaternion_functions.cpp"
#include "maths/functions/sign.hpp"

/* utilities */
#include "utilities/functions/implementations/quickdraw.cpp"
#include "utilities/functions/implementations/raster_functions.cpp"
#include "utilities/functions/implementations/render_functions.cpp"
#include "utilities/objects/implementations/virtual_keyboard.cpp"

/* system functions */
#include "systems/functions/implementations/radarFunctions.cpp"
#include "systems/functions/implementations/orderForRenderFunctions.cpp"
#include "systems/functions/implementations/cameraOperatorFunctions.cpp"


namespace pce3d {

Core3D::Core3D() {
  camera_ = Camera{
    .position = glm::dvec3(0.0, 0.0, 0.0),
    .view_direction = glm::dvec3(0.0, 0.0, 1.0),
    .focus_distance = 20.0,
    .zoom_amount = 1.0,
    .lense_curvature = 1.0,
    .lense_width = 1.0,
    .rotation_versor = glm::dquat(1.0, 0, 0, 0)
  };

  RegisterCoreComponents();
  RegisterCoreSystems();
}


void Core3D::RegisterCoreComponents() {
  control.RegisterComponent<pce::RigidObject>();
  control.RegisterComponent<pce::Position>();
  control.RegisterComponent<pce::LocalRotation>();
  control.RegisterComponent<pce::Surface>();
}


void Core3D::RegisterCoreSystems() {
  camera_operator_system_ = control.RegisterSystem<pce3d::CameraOperatorSystem>();

  camera_transform_system_ = control.RegisterSystem<pce3d::CameraTransformSystem>();
  control.AssignSystemComponents<pce3d::CameraTransformSystem, pce::Position>();

  radar_system_ = control.RegisterSystem<pce3d::RadarSystem>();
  control.AssignSystemComponents<pce3d::RadarSystem, pce::Position>();

  render_system_ = control.RegisterSystem<pce3d::RenderSystem>();
  control.AssignSystemComponents<pce3d::RenderSystem, pce::Position, pce::Surface>();

  render_order_system_ = control.RegisterSystem<pce3d::OrderForRenderSystem>();
  control.AssignSystemComponents<pce3d::RenderSystem, pce::Position>();
}


void Core3D::UpdateCore3D() {
  camera_operator_system_->UpdateCamera(camera_);
  camera_transform_system_->UpdateEntities(-camera_.position, camera_.rotation_versor);
  radar_system_->UpdateEntities();
  // render_order_system_->UpdateEntities();
  render_system_->UpdateEntities();
}


}


#endif /* Core3D_cpp */
