#ifndef OrderForRenderSystem_cpp
#define OrderForRenderSystem_cpp

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include <pcecs/ecs/System.cpp>
#include <ezprint.cpp>

#include "functions/orderForRenderFunctions.hpp"
#include "objects/orderTag.hpp"

extern ControlPanel control;

namespace pce3d {

class OrderForRenderSystem : public ISystem {
public:
  
  void UpdateEntities() {
    order_of_render_.clear();
    order_list_.clear();
    particles_to_order_last_.clear();
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      if (rigid_object.radius != 0) {particles_to_order_last_.push_back(entity); continue; }
      auto const& radar = control.GetComponent<pce::Radar>(entity);

      /* THE New NEW NEW */
      auto order_tag_ = orderTag{};
      order_tag_.entity = entity;
      order_tag_.closest_vertex_distance = radar.closest_vertex_distance;
      order_tag_.farthest_vertex_distance = radar.farthest_vertex_distance;

      pce3d::render_order::insertEntityIntoOrderMap(order_tag_, 
                                                    rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id),
                                                    order_list_, 0);
      

    }
    for (auto const& entity : particles_to_order_last_) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto const& radar = control.GetComponent<pce::Radar>(entity);

      /* THE New NEW NEW */
      auto order_tag_ = orderTag{};
      order_tag_.entity = entity;
      order_tag_.closest_vertex_distance = radar.closest_vertex_distance;
      order_tag_.farthest_vertex_distance = radar.farthest_vertex_distance;

      pce3d::render_order::insertEntityIntoOrderMap(order_tag_, 
                                                    rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id),
                                                    order_list_, 0);
      
    }
  }

  std::vector<std::pair<uint32_t, double>> order_of_render_;
  std::vector<orderTag> order_list_;
private:
  std::vector<uint32_t> particles_to_order_last_;
};


}

#endif /* OrderForRenderSystem_cpp */
