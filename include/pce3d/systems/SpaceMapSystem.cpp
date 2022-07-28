#ifndef SpaceMapSystem_cpp
#define SpaceMapSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for mapping physical 3space onto a smaller set of data
-----------------------------------------------------------------*/

#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>
#include <glm_hash.hpp>
#include <pcecs/ecs/System.cpp>
#include "functions/spaceMapFunctions.hpp"

#include "../utilities/functions/quickdraw.hpp"

extern ControlPanel control;

namespace pce3d{
class SpaceMapSystem : public ISystem {
public:
  
  SpaceMapSystem(const double meter_index_ratio = 1.0, const glm::ivec3 map_dimensions = glm::ivec3(10000, 10000, 10000))
    : meter_index_ratio_(meter_index_ratio), map_dimensions_(map_dimensions) {
    deadbod_space_map_ = {};
    livebod_space_map_ = {};
  }
  
  void DoPreLoopSetup() {
    for (auto const& entity : entities) {
      std::cout << "doing preloop setup for SpaceMapSystem for entity: " << entity << '\n';
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (!rigid_object.is_deadbod) {continue;}
      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.camera_transformed_vertices, map_dimensions_, meter_index_ratio_);

      /* put deadbod location -> entity mapping into map */
      for (auto const& index : indices) {
        if (deadbod_space_map_.find(index) == deadbod_space_map_.end()) {
          deadbod_space_map_[index] = {entity};
        } else {
          if (!std::count(deadbod_space_map_.at(index).begin(), deadbod_space_map_.at(index).end(), entity)) {
             deadbod_space_map_.at(index).push_back(entity); 
          }
        }
      }
    }

    for (auto const& [id, deadbods] : deadbod_space_map_) {
      for (auto const& entity : deadbods) {
        // std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
        const glm::dvec3 point = pce3d::space_map::findPointOfIndex(id, map_dimensions_, meter_index_ratio_);
        std::cout << "MAPSYS: " << entity << " | " << "location: " << point.x << ", " << point.y << ", " << point.z << '\n';
      }
    }
  }


  void UpdateEntities() {
    std::cout << "updating" << '\n';
    livebod_space_map_.clear();
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (rigid_object.is_deadbod) { continue; }

      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.camera_transformed_vertices, map_dimensions_, meter_index_ratio_);


      /* put vertices into livebody space map */
      for (auto const& index : indices) {
        if (livebod_space_map_.find(index) == livebod_space_map_.end()) {
          livebod_space_map_[index] = {entity};
        } else {
          if (!std::count(livebod_space_map_.at(index).begin(), livebod_space_map_.at(index).end(), entity)) {
             livebod_space_map_.at(index).push_back(entity); 
          }
        }
      }

    }

    for (auto const& [id, livebods] : livebod_space_map_) {
      for (auto const& entity : livebods) {
        std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
        const glm::dvec3 point = pce3d::space_map::findPointOfIndex(id, map_dimensions_, meter_index_ratio_);
        std::cout << "MAPSYS: " << entity << " | " << "point: " << point.x << ", " << point.y << ", " << point.z << '\n';
      }
    }
  }

  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_space_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_space_map_;


  double meter_index_ratio_;
  glm::ivec3 map_dimensions_;
private:

};
}


#endif /* SpaceMapSystem_cpp */
