#ifndef OrderForRenderSystem_cpp
#define OrderForRenderSystem_cpp

#include <utility>
#include <vector>
#include <pcecs/ecs/System.cpp>
#include <ezprint.cpp>

#include "functions/orderForRenderFunctions.hpp"

extern ControlPanel control;

namespace pce3d {
class OrderForRenderSystem : public ISystem {
public:
  
  void PrintOrderVector() {
    for (auto const& entity_pair : order_of_render_) {
      ezp::print_labeled_item("entity: ", entity_pair.first);
      ezp::print_labeled_item("distance from camera: ", entity_pair.second);
    }
  }

  void UpdateEntities() {
    order_of_render_.clear();
    for (auto const& entity : entities) {
      auto const& position = control.GetComponent<pce::Position>(entity);
      uint32_t centity = entity;
      std::pair<uint32_t, double> entity_with_distance = std::make_pair(centity, position.distance_from_camera);
      pce3d::render_order::insertEntityIntoRenderOrderVectorLinear(
        entity_with_distance, order_of_render_);
    }

    // PrintOrderVector();
  }

  std::vector<std::pair<uint32_t, double>> order_of_render_;

};



}

#endif /* OrderForRenderSystem_cpp */
