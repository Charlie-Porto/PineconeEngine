#ifndef pce3d_cpp
#define pce3d_cpp

#include "pce3d.hpp"

glm::dvec3 pce3d::Core3D::LIGHT_FLOW_DIRECTION_ = glm::dvec3(-0.1, -0.9, -0.2);
double pce3d::Core3D::LENSE_CURVE_ = 0.1;
double pce3d::Core3D::ORDINARY_ZOOM_INDEX_ = 10.0;


/***** not yet included *****/

/* maths */
#include "maths/functions/implementations/quaternion_functions.cpp"
#include "maths/functions/implementations/plane_functions.cpp"
#include "maths/functions/sign.hpp"

/* utilities */
#include "utilities/functions/implementations/quickdraw.cpp"
#include "utilities/functions/implementations/raster_functions.cpp"
#include "utilities/functions/implementations/render_functions.cpp"
#include "utilities/functions/implementations/triangle_raster_functions.cpp"
#include "utilities/objects/implementations/virtual_keyboard.cpp"

/* system functions */
#include "systems/functions/implementations/radarFunctions.cpp"
#include "systems/functions/implementations/renderFunctions.cpp"
#include "systems/functions/implementations/orderForRenderFunctions.cpp"
#include "systems/functions/implementations/cameraOperatorFunctions.cpp"
#include "systems/functions/implementations/shadeFunctions.cpp"

/* entity forging */
#include "entity_forging/implementations/sphere_forging.cpp"
#include "entity_forging/implementations/sheet_forging.cpp"
#include "entity_forging/implementations/rectangular_prism_forging.cpp"
#include "entity_forging/implementations/triangle_pyramid_forging.cpp"
#include "entity_forging/implementations/square_pyramid_forging.cpp"

namespace pce3d {



Core3D::Core3D(const glm::dvec3 light_flow_direction,
               const double lense_curve_index,
               const double ordinary_zoom_index) {

  camera_ = Camera{
    .position = glm::dvec3(0.0, 0.0, 0.0),
    .view_direction = glm::dvec3(0.0, 0.0, -1.0),
    .focus_distance = 20.0,
    .zoom_amount = 1.0,
    .lense_curvature = lense_curve_index,
    .lense_width = 1.0,
    .rotation_versor = glm::dquat(1.0, 0, 0, 0)
  };

  RegisterCoreComponents();
  RegisterCoreSystems();

  pce3d::Core3D::LIGHT_FLOW_DIRECTION_ = light_flow_direction;
  pce3d::Core3D::LENSE_CURVE_ = lense_curve_index;
  pce3d::Core3D::ORDINARY_ZOOM_INDEX_ = ordinary_zoom_index;
}


void Core3D::RegisterCoreComponents() {
  control.RegisterComponent<pce::RigidObject>();
  control.RegisterComponent<pce::Position>();
  control.RegisterComponent<pce::LocalRotation>();
  control.RegisterComponent<pce::Surface>();
  control.RegisterComponent<pce::FaceShade>();
}


void Core3D::RegisterCoreSystems() {
  camera_operator_system_ = control.RegisterSystem<pce3d::CameraOperatorSystem>();

  camera_transform_system_ = control.RegisterSystem<pce3d::CameraTransformSystem>();
  control.AssignSystemComponents<pce3d::CameraTransformSystem, pce::Position, pce::RigidObject>();

  radar_system_ = control.RegisterSystem<pce3d::RadarSystem>();
  control.AssignSystemComponents<pce3d::RadarSystem, pce::Position, pce::RigidObject>();

  render_system_ = control.RegisterSystem<pce3d::RenderSystem>();
  control.AssignSystemComponents<pce3d::RenderSystem, pce::Position, pce::Surface, pce::RigidObject, pce::FaceShade>();

  shade_system_ = control.RegisterSystem<pce3d::ShadeSystem>();
  control.AssignSystemComponents<pce3d::ShadeSystem, pce::RigidObject, pce::Position, pce::FaceShade>();

  render_order_system_ = control.RegisterSystem<pce3d::OrderForRenderSystem>();
  control.AssignSystemComponents<pce3d::OrderForRenderSystem, pce::Position>();
}


void Core3D::UpdateCore3D() {
  camera_operator_system_->UpdateCamera(camera_);
  camera_transform_system_->UpdateEntities(-camera_.position, camera_.rotation_versor);
  radar_system_->UpdateEntities();
  shade_system_->UpdateEntities();
  render_order_system_->UpdateEntities();
  render_system_->UpdateEntities(render_order_system_->order_of_render_);
}


}


#endif /* pce3d_cpp */
