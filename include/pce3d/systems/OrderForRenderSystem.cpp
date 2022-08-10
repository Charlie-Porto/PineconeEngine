#ifndef OrderForRenderSystem_cpp
#define OrderForRenderSystem_cpp

#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include <pcecs/ecs/System.cpp>
#include <ezprint.cpp>

#include "functions/orderForRenderFunctions.hpp"
#include "objects/orderTag.hpp"

#include "objects/OrderRenderListNode.hpp"

extern ControlPanel control;

namespace pce3d {

class OrderForRenderSystem : public ISystem {
public:

  // void UpdateEntities() 
  // {
  //   // std::cout << "updating order of render" << '\n';
  //   order_list_.clear();
  //   particles_to_order_last_.clear();
  //   for (auto const& entity : entities) {
  //     auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
  //     if (rigid_object.radius != 0) {particles_to_order_last_.push_back(entity); continue; }
  //     auto const& radar = control.GetComponent<pce::Radar>(entity);

  //     /* THE New NEW NEW */
  //     auto order_tag_ = orderTag{};
  //     order_tag_.entity = entity;
  //     order_tag_.closest_vertex_distance = radar.closest_vertex_distance;
  //     order_tag_.farthest_vertex_distance = radar.farthest_vertex_distance;

      // pce3d::render_order::insertEntityIntoOrderMapBinary(order_tag_, order_list_);
  //   }
  //   for (auto const& entity : particles_to_order_last_) {
  //     // std::cout << "particles entered now" << '\n';
  //     auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
  //     auto const& radar = control.GetComponent<pce::Radar>(entity);

  //     /* THE New NEW NEW */
  //     auto order_tag_ = orderTag{};
  //     order_tag_.entity = entity;
  //     order_tag_.closest_vertex_distance = radar.closest_vertex_distance;
      // order_tag_.closest_vertex_location = rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id);
  //     order_tag_.farthest_vertex_distance = radar.farthest_vertex_distance;

  //     // pce3d::render_order::insertEntityIntoOrderMapBinary(order_tag_, order_list_);
  //     pce3d::render_order::insertEntityIntoOrderMapStartingAtEnd(order_tag_, order_list_);
  //     for (auto const& order_tag : order_list_) {
  //       std::cout << order_tag.entity << '\n';
  //     }
  //   }
    // std::cout << "---" << '\n';
  // }

  void UpdateEntities(std::vector<std::pair<uint32_t, double>> order_of_ordering) 
  {
    // std::cout << "---" << '\n';
    for (auto const& mpair : order_of_ordering)
    {
      // std::cout << "entity: " << mpair.first <<'\n';
    }
    order_list_.clear();
    size_t i = 0;

    orderTag head_node_tag = orderTag{};

    while (i < 1)
    {
      uint32_t entity = order_of_ordering[i].first;
      if (!control.CheckIfEntityStillExists(entity))
      {
        order_of_ordering.erase(order_of_ordering.begin() + i);
        continue;
      }
      auto const& radar = control.GetComponent<pce::Radar>(entity);
      auto const& render = control.GetComponent<pce::Render>(entity);
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);

      if (render.just_registered) { control.RemoveComponent<pce::OrderOfRenderRegistration>(entity); }

      head_node_tag.entity = entity;
      head_node_tag.closest_vertex_distance = radar.closest_vertex_distance;
      head_node_tag.closest_vertex_location = rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id);
      head_node_tag.farthest_vertex_distance = radar.farthest_vertex_distance;

      ++i;
    }

    pce3d::render_order::OrderRenderListNode* head_node = new pce3d::render_order::OrderRenderListNode(head_node_tag);


    for (size_t i = 1; i != order_of_ordering.size(); ++i)
    {
      const uint32_t entity = order_of_ordering[i].first;
      if (!control.CheckIfEntityStillExists(entity))
      {
        order_of_ordering.erase(order_of_ordering.begin() + i);
        continue;
      }
      auto const& radar = control.GetComponent<pce::Radar>(entity);
      auto const& render = control.GetComponent<pce::Render>(entity);
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);

      if (render.just_registered) { control.RemoveComponent<pce::OrderOfRenderRegistration>(entity); }

      auto order_tag_ = orderTag{};
      order_tag_.entity = entity;
      order_tag_.closest_vertex_distance = radar.closest_vertex_distance;
      order_tag_.closest_vertex_location = rigid_object.camera_transformed_vertices.at(radar.closest_vertex_id);
      order_tag_.farthest_vertex_distance = radar.farthest_vertex_distance;

      pce3d::render_order::OrderRenderListNode* node = new pce3d::render_order::OrderRenderListNode(order_tag_);
      head_node->InsertNodeInTree(node);
      // pce3d::render_order::insertEntityIntoOrderMapBinary(order_tag_, order_list_);
    }

    order_list_ = head_node->GetListAtNode();
    delete head_node;
    for (auto const& entity : order_list_) {
      std::cout << entity << '\n';
    }
  }


  // std::vector<orderTag> order_list_;
  std::vector<uint32_t> order_list_;
private:
  std::vector<uint32_t> particles_to_order_last_;
};


}

#endif /* OrderForRenderSystem_cpp */
