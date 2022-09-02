#ifndef pce3d_cpp
#define pce3d_cpp

#include "pce3d.hpp"

glm::dvec3 pce3d::Core3D::LIGHT_FLOW_DIRECTION_ = glm::dvec3(-0.1, -0.9, -0.2);
double pce3d::Core3D::LENSE_CURVE_ = 0.1;
double pce3d::Core3D::ORDINARY_ZOOM_INDEX_ = 15.0;
double pce3d::Core3D::COLLISION_METER_INDEX_RATIO = 1.0;
glm::ivec3 pce3d::Core3D::SPACE_MAP_DIMENSIONS = glm::ivec3(10000, 10000, 10000);
glm::dvec3 pce3d::Core3D::HARD_BOUNDARIES = glm::dvec3(10000, 10000, 10000);
glm::dvec3 pce3d::Core3D::MAP_CENTER = glm::dvec3(0, 0, 0);


#include "modules.cpp"


namespace pce3d {



Core3D::Core3D(const glm::dvec3 hard_boundaries,
               const glm::dvec3 light_flow_direction,
               const double lense_curve_index,
               const double ordinary_zoom_index) {

  camera_ = Camera{
    .position = glm::dvec3(-30.0, 0.0, 0.0),
    .view_direction = glm::dvec3(0.0, 0.0, 1.0),
    .focus_distance = 20.0,
    .zoom_amount = 1.0,
    .lense_curvature = lense_curve_index,
    .lense_width = 1.0,
    .rotation_versor = glm::dquat(1.0, 0, 0, 0)
  };

  RegisterCoreComponents();
  RegisterCoreSystems();

  pce3d::Core3D::HARD_BOUNDARIES = hard_boundaries;
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
  control.RegisterComponent<pce::Force>();
  control.RegisterComponent<pce::Motion>();
  control.RegisterComponent<pce::Radar>();
  control.RegisterComponent<pce::Render>();
  control.RegisterComponent<pce::OrderOfRenderRegistration>();
}


void Core3D::RegisterCoreSystems() {
  camera_operator_system_ = control.RegisterSystem<pce3d::CameraOperatorSystem>();

  camera_transform_system_ = control.RegisterSystem<pce3d::CameraTransformSystem>();
  control.AssignSystemComponents<pce3d::CameraTransformSystem, pce::Position, pce::RigidObject>();

  radar_system_ = control.RegisterSystem<pce3d::RadarSystem>();
  control.AssignSystemComponents<pce3d::RadarSystem, pce::Position, pce::RigidObject, pce::Radar>();

  render_system_ = control.RegisterSystem<pce3d::RenderSystem>();
  render_system_->setOrdinaryZoomIndex(pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  control.AssignSystemComponents<pce3d::RenderSystem, pce::Position, pce::Surface, pce::RigidObject, pce::FaceShade, pce::Radar>();

  shade_system_ = control.RegisterSystem<pce3d::ShadeSystem>();
  shade_system_->setOrdinaryZoomIndex(pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  control.AssignSystemComponents<pce3d::ShadeSystem, pce::RigidObject, pce::Position, pce::FaceShade>();

  render_order_system_ = control.RegisterSystem<pce3d::OrderForRenderSystem>();
  control.AssignSystemComponents<pce3d::OrderForRenderSystem, pce::Position, pce::RigidObject, pce::Radar, pce::Render>();
  
  register_for_render_order_system_ = control.RegisterSystem<pce3d::RegisterForOrderOfRenderSystem>();
  control.AssignSystemComponents<pce3d::RegisterForOrderOfRenderSystem, pce::Radar, pce::Render, pce::RigidObject, pce::OrderOfRenderRegistration>();

  space_map_system_ = control.RegisterSystem<pce3d::SpaceMapSystem>();
  space_map_system_->SetMainParameters(pce3d::Core3D::COLLISION_METER_INDEX_RATIO, pce3d::Core3D::SPACE_MAP_DIMENSIONS);
  control.AssignSystemComponents<pce3d::SpaceMapSystem, pce::RigidObject>();

  physics_system_ = control.RegisterSystem<pce3d::PhysicsSystem>();
  control.AssignSystemComponents<pce3d::PhysicsSystem, pce::RigidObject, pce::Force, pce::Motion, pce::Position>();
}


void Core3D::PrepareForAllSystemsGo() {
  shade_system_->DoPreLoopSetup();  
  space_map_system_->DoPreLoopSetup();  
  std::cout << "pre-loop setup is complete" << '\n';
}


void Core3D::UpdateCore3D() {
  camera_operator_system_->UpdateCamera(camera_);
  // std::cout << "cam system updated" << '\n';
  camera_transform_system_->UpdateEntities(-camera_.position, camera_.rotation_versor);
  // std::cout << "cam transform system updated" << '\n';
  space_map_system_->UpdateEntities();
  // std::cout << "space_map system updated" << '\n';
  physics_system_->UpdateEntities(space_map_system_->potential_colliding_entities_);
  // std::cout << "physics system updated" << '\n';
  radar_system_->UpdateEntities();
  // std::cout << "radar system updated" << '\n';
  shade_system_->UpdateEntities(camera_.rotation_versor, -camera_.position); 
  // std::cout << "shade system updated" << '\n';
  register_for_render_order_system_->RegisterUnRegisteredEntities();
  // std::cout << "order register system updated" << '\n';
  render_order_system_->UpdateEntities(register_for_render_order_system_->order_of_ordering_);
  // std::cout << "render order system updated" << '\n';
  render_system_->UpdateEntities(render_order_system_->order_list_);
  // std::cout << "render system updated" << '\n';
  // space_map_system_->drawMapPointsInSpace(camera_.rotation_versor, -camera_.position);
  dev_render_system.RenderPoints(-camera_.position, camera_.rotation_versor);
}

}


#endif /* pce3d_cpp */
