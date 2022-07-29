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
  
  SpaceMapSystem(const double meter_index_ratio = 6.0, const glm::ivec3 map_dimensions = glm::ivec3(10000, 10000, 10000))
    : meter_index_ratio_(meter_index_ratio), map_dimensions_(map_dimensions) {
    deadbod_map_ = {};
    livebod_map_ = {};
    restingbod_map_ = {};
    potential_colliding_entities_ = {};
  }
  
  void DoPreLoopSetup() {
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (!rigid_object.is_deadbod && !rigid_object.is_restingbod) {continue;}

      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);

      
      if (rigid_object.is_deadbod) {
        /* put deadbod location -> entity mapping into map */
        for (auto const& index : indices) {
          if (deadbod_map_.find(index) == deadbod_map_.end()) {
            deadbod_map_[index] = {entity};
          } else {
            if (!std::count(deadbod_map_.at(index).begin(), deadbod_map_.at(index).end(), entity)) {
              deadbod_map_.at(index).push_back(entity); 
            }
          }
        }
      }
      if (rigid_object.is_restingbod) {
        /* put restingbod location -> entity mapping into map */
        for (auto const& index : indices) {
          if (restingbod_map_.find(index) == restingbod_map_.end()) {
            restingbod_map_[index] = {entity};
          } else {
            if (!std::count(restingbod_map_.at(index).begin(), restingbod_map_.at(index).end(), entity)) {
              restingbod_map_.at(index).push_back(entity); 
            }
          }
        }
      }
    }

    // for (auto const& [id, deadbods] : deadbod_map_) {
    //   for (auto const& entity : deadbods) {
    //     // std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
    //     const glm::dvec3 point = pce3d::space_map::findPointOfIndex(id, map_dimensions_, meter_index_ratio_);
    //     // std::cout << "MAPSYS: " << entity << " | " << "location: " << point.x << ", " << point.y << ", " << point.z << '\n';
    //   }
    // }
    // for (auto const& [id, resting_bods] : restingbod_map_) {
    //   for (auto const& entity : resting_bods) {
    //     // std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
    //     // const glm::dvec3 point = pce3d::space_map::findPointOfIndex(id, map_dimensions_, meter_index_ratio_);
    //     // std::cout << "MAPSYS: " << entity << " | " << "location: " << point.x << ", " << point.y << ", " << point.z << '\n';
    //   }
    // }
  }

/* ---------------------------------------- update --------------------------------- */
  void UpdateEntities() {
    livebod_map_.clear();
    potential_colliding_entities_.clear();
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (rigid_object.is_deadbod || rigid_object.is_restingbod) { continue; }

      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);

      /* put vertices into livebody space map */
      for (auto const& index : indices) {
        if (livebod_map_.find(index) == livebod_map_.end()) {
          livebod_map_[index] = {entity};
        } else {
          if (!std::count(livebod_map_.at(index).begin(), livebod_map_.at(index).end(), entity)) {
             livebod_map_.at(index).push_back(entity); 
             potential_colliding_entities_[entity] = livebod_map_.at(index)[0];
          }
        }
        if (restingbod_map_.find(index) != restingbod_map_.end()) {
          potential_colliding_entities_[entity] = restingbod_map_.at(index)[0];
        }
        if (deadbod_map_.find(index) != deadbod_map_.end()) {
          potential_colliding_entities_[entity] = deadbod_map_.at(index)[0];
        }
      }

    }

    // for (auto const& [id, livebods] : livebod_map_) {
      // for (auto const& entity : livebods) {
        // std::cout << "entity: " << entity << " | " << "location: " << id.x << ", " << id.y << ", " << id.z << '\n';
        // const glm::dvec3 point = pce3d::space_map::findPointOfIndex(id, map_dimensions_, meter_index_ratio_);
        // std::cout << "MAPSYS: " << entity << " | " << "point: " << point.x << ", " << point.y << ", " << point.z << '\n';
      // }
    // }

    // for (auto const& [first, second] : potential_colliding_entities_) {
      // std::cout << "potential colliding entities: " << first << ", " << second << '\n';
    // }
  }

  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> restingbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_map_;
  std::unordered_map<uint32_t, uint32_t> potential_colliding_entities_;


  double meter_index_ratio_;
  glm::ivec3 map_dimensions_;
private:

};
}


#endif /* SpaceMapSystem_cpp */
