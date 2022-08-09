#ifndef RegisterForOrderOfRenderSystem_cpp
#define RegisterForOrderOfRenderSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
a prep-only system that establishes an ordering of all elements to be used
by the OrderOfRenderSystem to determine the actual order of rendering
of entites.

This system helps to prevent unclean render ordering due to comparisons of 
large entities to small entities, as opposed to small entities to large entities 
-----------------------------------------------------------------*/

#include <utility>
#include <vector>
#include <pcecs/ecs/System.cpp>
#include "../../maths/functions/vector_functions.hpp"

extern ControlPanel control;

namespace pce3d {
class RegisterForOrderOfRenderSystem : public ISystem 
{
public:
RegisterForOrderOfRenderSystem() : order_of_ordering_({}) {}

void RegisterUnRegisteredEntities() 
{
  for (auto const& entity : entities) 
  {
    auto& render = control.GetComponent<pce::Render>(entity); 
    auto const& radar = control.GetComponent<pce::Radar>(entity); 
    auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 

    const glm::vec3 v_closest_location = rigid_object.vertices.at(radar.closest_vertex_id);
    const glm::vec3 v_farthest_location = rigid_object.vertices.at(radar.farthest_vertex_id);
    const double distance = pce3d::maths::calculateDistanceBetweenVectors(v_closest_location, v_farthest_location);

    bool inserted = false;
    for (size_t i = 0; i != order_of_ordering_.size(); ++i)
    {
      if (distance > order_of_ordering_[i].second)
      {
        order_of_ordering_.insert(order_of_ordering_.begin() + i, std::make_pair(entity, distance));
        inserted = true;
        break;
      }
    }
    if (!inserted) { order_of_ordering_.push_back(std::make_pair(entity, distance)); }
    render.just_registered = true;
    render.is_registered = true;
  }
}

std::vector<std::pair<uint32_t, double>> order_of_ordering_;

};
}
#endif /* RegisterForOrderOfRenderSystem_cpp */
