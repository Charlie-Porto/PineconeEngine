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
  OrderForRenderSystem() : order_tag_(orderTag{}) {}
  
  void UpdateEntities() {
    order_of_render_.clear();
    for (auto const& entity : entities) {
      auto const& position = control.GetComponent<pce::Position>(entity);
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);

      /* depreciating this stuff */
      uint32_t centity = entity;
      std::pair<uint32_t, double> entity_with_distance = std::make_pair(centity, position.distance_from_camera);
      pce3d::render_order::insertEntityIntoRenderOrderVectorLinear(entity_with_distance, order_of_render_);
      /* -------------- */
      

      /* THE NEW NEW */
      order_tag_.entity = entity;
      uint32_t id_closest_vertex = 1;
      uint32_t id_farthest_vertex = 1;
      double distance_closest_vertex = rigid_object.vertex_distance_map.at(id_closest_vertex);
      double distance_farthest_vertex = rigid_object.vertex_distance_map.at(id_farthest_vertex);;

      /* get closest and furthest vertex */
      for (auto const& [id, distance] : rigid_object.vertex_distance_map) {
        if (distance < distance_closest_vertex) {
          id_closest_vertex = id;
          distance_closest_vertex = distance;
        }
        else if (distance > distance_farthest_vertex) {
          id_farthest_vertex = id;
          distance_farthest_vertex = distance;
        }
      }

      order_tag_.closest_vertex_distance = distance_closest_vertex;
      order_tag_.farthest_vertex_distance = distance_farthest_vertex;
      
      std::pair<bool, size_t> insert_info = pce3d::render_order::tryInsertEntityIntoRenderOrderMap(
                                                order_tag_, order_list_);
      
      if (!insert_info.first) {
        pce3d::render_order::insertEntityBetweenVerticesIntoRenderOrderMapAtIndex(
                                 order_tag_, insert_info.second, order_list_);
      }

    }
  }

  std::vector<std::pair<uint32_t, double>> order_of_render_;
  std::vector<orderTag> order_list_;
private:
  orderTag order_tag_;
};


}

#endif /* OrderForRenderSystem_cpp */
