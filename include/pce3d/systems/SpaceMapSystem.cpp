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
#include "../utilities/functions/glm_hash.hpp"
#include <pcecs/ecs/System.cpp>
#include "functions/spaceMapFunctions.hpp"
#include "functions/radarFunctions.hpp"
#include "../utilities/functions/quickdraw.hpp"

extern ControlPanel control;

namespace pce3d{
class SpaceMapSystem : public ISystem {
public:
  
  SpaceMapSystem(const double meter_index_ratio = 1.0, const glm::ivec3 map_dimensions = glm::ivec3(10000, 10000, 10000))
  // SpaceMapSystem(const double meter_index_ratio = 2.0, const glm::ivec3 map_dimensions = glm::ivec3(10000, 10000, 10000))
    : meter_index_ratio_(meter_index_ratio), map_dimensions_(map_dimensions) {
    deadbod_map_ = {};
    livebod_map_ = {};
    livebod_vertex_map_ = {};
    restingbod_map_ = {};
    potential_colliding_entities_ = {};
 }

  void SetMainParameters(const double meter_index_ratio, const glm::ivec3 map_dimensions)
  {
    meter_index_ratio_ = meter_index_ratio;
    map_dimensions_ = map_dimensions;
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
      // indices.insert(indices.end(), vertex_indices.begin(), vertex_indices.end());

      for (auto const& [face, vertex_ids] : rigid_object.face_vertex_map) {
        
        std::vector<glm::ivec3> face_indices{};
        switch (vertex_ids.size()) 
        {
          case 4: 
            face_indices = space_map::findRectFaceIndices(rigid_object.face_vertex_map.at(face),
                                                          rigid_object.vertices,
                                                          map_dimensions_,
                                                          meter_index_ratio_);
            break;
          case 3:
            face_indices = space_map::findTriangleFaceIndices(rigid_object.face_vertex_map.at(face),
                                                              rigid_object.vertices,
                                                              map_dimensions_,
                                                              meter_index_ratio_);
            break;
          default:
            break;
        }
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
      else if (rigid_object.is_restingbod) {
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
      const glm::dvec3 converted_point = pce3d::space_map::findPointOfIndex(point, map_dimensions_, meter_index_ratio_);
      glm::dvec3 rotated_point = converted_point - cam_transform;
      double distance = sqrt(glm::dot(rotated_point, rotated_point));
      rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
      const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
      pce::quickdraw::drawCircle(pixel, 10.0 / distance, {12, 200, 200, 255});
    }
    for (auto const& [point, entity] : livebod_map_) {
      const glm::dvec3 converted_point = pce3d::space_map::findPointOfIndex(point, map_dimensions_, meter_index_ratio_);
      glm::dvec3 rotated_point = converted_point - cam_transform;
      double distance = sqrt(glm::dot(rotated_point, rotated_point));
      rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
      const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
      pce::quickdraw::drawCircle(pixel, 10.0 / distance, {12, 200, 200, 255});
    }
  }

/* ---------------------------------------- update --------------------------------- */
  void UpdateEntities() 
  {
    livebod_map_.clear();
    livebod_vertex_map_.clear();
    restingbod_map_.clear();
    potential_colliding_entities_.clear();
    for (auto const& entity : entities) 
    {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (rigid_object.is_deadbod) { continue; }

      rigid_object.entity_face_collision_map.clear();
      rigid_object.entity_index_collision_map.clear();
      
      const std::vector<glm::ivec3> vertex_indices 
        = space_map::findIndicesGivenVertices(rigid_object.vertices, map_dimensions_, meter_index_ratio_);
      const std::unordered_map<uint32_t, glm::ivec3> labeled_vertex_indices 
        = space_map::findIndicesGivenVerticesLabeled(rigid_object.vertices, map_dimensions_, meter_index_ratio_);
      
      bool check_for_collision = true; 
      
      /* update position: complex livebody */
      if (!rigid_object.is_restingbod)
      {
        for (auto const& [id, index] : labeled_vertex_indices)
        {
          livebod_vertex_map_[index][entity] = id;
        }


        // std::cout << "updating complex livebod position" << '\n';
        std::vector<glm::ivec3> indices{};
        for (auto const& [face, vertex_ids] : rigid_object.face_vertex_map) {
          // std::cout << "face: " << face << '\n';    
          std::vector<glm::ivec3> face_indices{};
          switch (vertex_ids.size()) 
          {
            case 1:
              face_indices = vertex_indices;
              break;
            case 4: 
              // std::cout << "case 4" << '\n';
              face_indices = space_map::findRectFaceIndices(rigid_object.face_vertex_map.at(face),
                                                            rigid_object.vertices,
                                                            map_dimensions_,
                                                            meter_index_ratio_);
              break;
            case 3:
              face_indices = space_map::findTriangleFaceIndices(rigid_object.face_vertex_map.at(face),
                                                                rigid_object.vertices,
                                                                map_dimensions_,
                                                                meter_index_ratio_);
              break;
            default:
              break;
          }

          indices.insert(indices.end(), face_indices.begin(), face_indices.end());

          for (auto const& index : face_indices) 
          {
            rigid_object.index_face_map[index] = face;
            rigid_object.face_index_map[face] = index;

            if (livebod_map_.find(index) == livebod_map_.end()) 
            {
              livebod_map_[index] = {entity};
              livebod_index_face_map_[index][entity] = face;
            } 

            else 
            {
              if (!std::count(livebod_map_.at(index).begin(), livebod_map_.at(index).end(), entity)) {
                livebod_map_.at(index).push_back(entity); 
                potential_colliding_entities_[entity] = livebod_map_.at(index)[0];

                std::cout << "livebod collision detected: "<< entity << '\n';

                auto& other_rigid_object = control.GetComponent<pce::RigidObject>(livebod_map_.at(index)[0]);

                bool original_entity_finished = false;
                bool other_entity_finished = false;
                if (livebod_vertex_map_.find(index) != livebod_vertex_map_.end())
                {
                  if (livebod_vertex_map_.at(index).find(entity) != livebod_vertex_map_.at(index).end())
                  {
                    rigid_object.entity_vertex_collision_map[livebod_map_.at(index)[0]] 
                      = livebod_vertex_map_.at(index).at(entity);
                    original_entity_finished = true;
                  }
                }
                if (!original_entity_finished)
                {
                  rigid_object.entity_index_collision_map[livebod_map_.at(index)[0]] = {index};
                  rigid_object.entity_face_collision_map[livebod_map_.at(index)[0]] = face;
                }

                if (livebod_vertex_map_.find(index) != livebod_vertex_map_.end())
                {
                  if (livebod_vertex_map_.at(index).find(livebod_map_.at(index)[0]) != livebod_vertex_map_.at(index).end())
                  {
                    other_rigid_object.entity_vertex_collision_map[livebod_map_.at(index)[0]] 
                      = livebod_vertex_map_.at(index).at(livebod_map_.at(index)[0]);
                    other_entity_finished = true;
                  }
                }
                if (!other_entity_finished)
                {
                  other_rigid_object.entity_index_collision_map[livebod_map_.at(index)[0]] = {index};
                  other_rigid_object.entity_face_collision_map[entity] = livebod_index_face_map_.at(index).at(livebod_map_.at(index)[0]);
                }

                // if (rigid_object.entity_index_collision_map.find(livebod_map_.at(index)[0])
                  // == rigid_object.entity_index_collision_map.end())
                // {
                  rigid_object.entity_index_collision_map[livebod_map_.at(index)[0]] = {index};
                // }
                // else
                // {
                  // rigid_object.entity_index_collision_map.at(livebod_map_.at(index)[0]).push_back(index);
                // }
              }
            }
          }

        }
      }

      // std::cout << "first index: " << indices[0].x << ", " << indices[0].y << ", " << indices[0].z << '\n';
      // std::cout << "restingbods" << '\n'; 
      else if (rigid_object.is_restingbod)
      {
        for (auto const& index : vertex_indices) {
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


      if (check_for_collision)
      {
        for (auto const& index : vertex_indices)
        {
          if (restingbod_map_.find(index) != restingbod_map_.end()) 
          {
            potential_colliding_entities_[entity] = restingbod_map_.at(index)[0];
            continue;
          }
          else if (deadbod_map_.find(index) != deadbod_map_.end()) 
          {
            uint32_t deadbod_entity = deadbod_map_.at(index)[0];
            potential_colliding_entities_[entity] = deadbod_entity; 
            auto& deadbod_rigid_object = control.GetComponent<pce::RigidObject>(deadbod_entity);
            deadbod_rigid_object.entity_face_collision_map[entity] = deadbod_rigid_object.index_face_map.at(index);
            rigid_object.entity_face_collision_map[deadbod_entity] = rigid_object.index_face_map.at(index);
          }
        }
      }
    }
    // std::cout << "space map system: success" << '\n'; 
  }

  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> restingbod_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> livebod_vertex_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> livebod_index_face_map_;
  std::unordered_map<uint32_t, uint32_t> potential_colliding_entities_;

  double meter_index_ratio_;
  glm::ivec3 map_dimensions_;

};
}


#endif /* SpaceMapSystem_cpp */
