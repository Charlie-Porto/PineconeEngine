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
#include "functions/radarFunctions.hpp"
#include "../utilities/functions/quickdraw.hpp"

extern ControlPanel control;

namespace pce3d{
class SpaceMapSystem : public ISystem {
public:
  
  SpaceMapSystem(const double meter_index_ratio = 2.0, const glm::ivec3 map_dimensions = glm::ivec3(10000, 10000, 10000))
    : meter_index_ratio_(meter_index_ratio), map_dimensions_(map_dimensions) {
    deadbod_map_ = {};
    livebod_map_ = {};
    restingbod_map_ = {};
    potential_colliding_entities_ = {};
 }
  
/* ---------------------------------------- setup --------------------------------- */
  void DoPreLoopSetup() {
    std::cout << "Doing Pre-Loop Setup" << '\n';
    for (auto const& entity : entities) {
      auto & rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (!rigid_object.is_deadbod && !rigid_object.is_restingbod) {continue;}

      // const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);
      const std::vector<glm::ivec3> vertex_indices = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);
      std::vector<glm::ivec3> indices{};
      indices.insert(indices.end(), vertex_indices.begin(), vertex_indices.end());

      for (auto const& [face, vertex_ids] : rigid_object.face_vertex_map) {
        std::cout << "getting face indices" << '\n';
        const std::vector<glm::ivec3> face_indices = space_map::findRectFaceIndices(rigid_object.face_vertex_map.at(face),
                                                                                    rigid_object.vertices,
                                                                                    map_dimensions_,
                                                                                    meter_index_ratio_);
        indices.insert(indices.end(), face_indices.begin(), face_indices.end());
        for (auto const& index : face_indices) {
          rigid_object.index_face_map[index] = face;
          rigid_object.face_index_map[face] = index;
        }
      }

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
  }
 

/* ---------------------------------------- draw --------------------------------- */
void drawMapPointsInSpace(const glm::dquat& cam_versor, const glm::dvec3& cam_transform) {
  std::cout << "-----" << '\n';
  for (auto const& [point, entity] : deadbod_map_) {
    // std::cout << "point: " << point.x << ", " << point.y << ", " << point.z << '\n';
    const glm::dvec3 converted_point = pce3d::space_map::findPointOfIndex(point, map_dimensions_, meter_index_ratio_);
    glm::dvec3 p = converted_point;
    // std::cout << "p: " << p.x << ", " << p.y << ", " << p.z << '\n';
    glm::dvec3 rotated_point = converted_point - cam_transform;
    double distance = sqrt(glm::dot(rotated_point, rotated_point));
    rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
    const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
    const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
    pce::quickdraw::drawCircle(pixel, 10.0 / distance, {12, 200, 200, 255});
  }
}

/* ---------------------------------------- update --------------------------------- */
/* passing the camera versor to render index points during developement */
  void UpdateEntities() 
  {
    livebod_map_.clear();
    restingbod_map_.clear();
    potential_colliding_entities_.clear();
    for (auto const& entity : entities) 
    {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (rigid_object.is_deadbod) { continue; }
      const std::vector<glm::ivec3> indices = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);
      // std::cout << "first index: " << indices[0].x << ", " << indices[0].y << ", " << indices[0].z << '\n';
      
      if (rigid_object.is_restingbod)
      {
        for (auto const& index : indices) {
          if (restingbod_map_.find(index) == restingbod_map_.end()) 
          {
            restingbod_map_[index] = {entity};
            if (livebod_map_.find(index) != livebod_map_.end())
            {
              potential_colliding_entities_[livebod_map_.at(index)[0]] = entity;
            }
          } 
        }
        continue;
      }

      /* put vertices into livebody space map */
      for (auto const& index : indices) 
      {
        bool check_for_collision = true;
        if (livebod_map_.find(index) == livebod_map_.end()) 
        {
          livebod_map_[index] = {entity};
        } else {
          if (!std::count(livebod_map_.at(index).begin(), livebod_map_.at(index).end(), entity)) {
             livebod_map_.at(index).push_back(entity); 
             potential_colliding_entities_[entity] = livebod_map_.at(index)[0];
             std::cout << "SpaceMapSystem: collision with livebod" << '\n';
             continue;
          }
        }

        if (check_for_collision)
        {
          if (restingbod_map_.find(index) != restingbod_map_.end()) {
            potential_colliding_entities_[entity] = restingbod_map_.at(index)[0];
            continue;
          }
          if (deadbod_map_.find(index) != deadbod_map_.end()) {
            uint32_t deadbod_entity = deadbod_map_.at(index)[0];
            potential_colliding_entities_[entity] = deadbod_entity; 
            auto& deadbod_rigid_object = control.GetComponent<pce::RigidObject>(deadbod_entity);
            deadbod_rigid_object.entity_face_collision_map[entity] 
              = deadbod_rigid_object.index_face_map.at(index);
            // std::cout << "SpaceMapSystem: collision with deadbod" << '\n';
          }
        }
      }


    }
  }

  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> restingbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_map_;
  std::unordered_map<uint32_t, uint32_t> potential_colliding_entities_;

  double meter_index_ratio_;
  glm::ivec3 map_dimensions_;

};
}


#endif /* SpaceMapSystem_cpp */
