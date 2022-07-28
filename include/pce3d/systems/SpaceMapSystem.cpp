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
  
  SpaceMapSystem(const double meter_index_ratio = 1.0) {
    meter_index_ratio_ = meter_index_ratio;
    deadbod_space_map_ = {};
    livebod_space_map_ = {};
  }
  
  void DoPreLoopSetup() {
    for (auto const& entity : entities) {
      std::cout << "doing preloop setup for SpaceMapSystem for entity: " << entity << '\n';
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 

      if (!rigid_object.is_deadbod) {continue;}
      
      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.camera_transformed_vertices, meter_index_ratio_);

      // /* put deadbod location -> entity mapping into map */
      // for (auto const& index : indices) {
      //   if (deadbod_space_map_.find(index) == deadbod_space_map_.end()) {
      //     deadbod_space_map_[index] = {entity};
      //   } else {
      //     if (!std::count(deadbod_space_map_.at(index).begin(), deadbod_space_map_.at(index).end(), entity)) {
      //        deadbod_space_map_.at(index).push_back(entity); 
      //     }
      //   }

      //   for (auto const& mindex : indices) {
      //     if (index == mindex) { continue; }
      //     const glm::ivec3 midpoint = glm::ivec3((index.x + mindex.x)/2, (index.y + mindex.y)/2, (index.z + mindex.z)/2);
      //     const glm::dvec3 dmidpoint = glm::dvec3(midpoint.x, midpoint.y, midpoint.z);
      //     if (deadbod_space_map_.find(midpoint) == deadbod_space_map_.end()) {
      //       deadbod_space_map_[midpoint] = {entity};
      //     } else {
      //       if (!std::count(deadbod_space_map_.at(midpoint).begin(), deadbod_space_map_.at(midpoint).end(), entity)) {
      //         deadbod_space_map_.at(midpoint).push_back(entity); 
      //       }
      //     }
      //   }

      // }
    }

    for (auto const& [id, deadbods] : deadbod_space_map_) {
      for (auto const& entity : deadbods) {
        std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
      }
    }
  }


  void UpdateEntities() {
    for (auto const& entity : entities) {

    }
  }

  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_space_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_space_map_;


  double meter_index_ratio_;
private:

};
}


#endif /* SpaceMapSystem_cpp */
