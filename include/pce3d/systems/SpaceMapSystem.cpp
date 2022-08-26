#ifndef SpaceMapSystem_cpp
#define SpaceMapSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for mapping the virtual 3space onto a relatively small set of data.
- used for collision detection
-----------------------------------------------------------------*/

#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <vector>

#include <glm/vec3.hpp>

#include <pcecs/ecs/System.cpp>

#include "functions/spaceMapFunctions.hpp"
#include "functions/radarFunctions.hpp"
#include "../utilities/functions/glm_hash.hpp"
#include "../utilities/functions/quickdraw.hpp"

extern ControlPanel control;

namespace pce3d {
class SpaceMapSystem : public ISystem 
{
public:
  
  SpaceMapSystem() :
      deadbod_map_({})
    , deadbod_index_vertex_map_({})
    , deadbod_index_edge_map_({})
    , deadbod_index_face_map_({})
    , restingbod_map_({})
    , restingbod_index_vertex_map_({})
    , restingbod_index_edge_map_({})
    , restingbod_index_face_map_({})
    , livebod_map_({})
    , livebod_index_vertex_map_({})
    , livebod_index_edge_map_({})
    , livebod_index_face_map_({})
    , potential_colliding_entities_({})
    , meter_index_ratio_(0)
    , map_dimensions_(glm::ivec3(0, 0, 0))
  {}


  void SetMainParameters(const double meter_index_ratio, const glm::ivec3 map_dimensions)
  {
    meter_index_ratio_ = meter_index_ratio;
    map_dimensions_ = map_dimensions;
  }


  void DoPreLoopSetup()
  {
    for (auto const& entity : entities) 
    {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (!rigid_object.is_deadbod && !rigid_object.is_restingbod) { continue; }
      auto& bod_map = rigid_object.is_deadbod ? deadbod_map_ : restingbod_map_;
      auto& bod_vertex_map = rigid_object.is_deadbod ? deadbod_index_vertex_map_ : restingbod_index_vertex_map_;
      auto& bod_edge_map = rigid_object.is_deadbod ? deadbod_index_edge_map_ : restingbod_index_edge_map_;

      std::unordered_map<uint32_t, glm::ivec3>  vertex_indices = pce3d::space_map::updateBodVertexMap(
        entity,
        rigid_object,
        bod_vertex_map,
        map_dimensions_,
        meter_index_ratio_ 
      );

      std::unordered_map<uint32_t, std::vector<glm::ivec3>>  edge_indices = pce3d::space_map::updateBodEdgeMap(
        entity,
        rigid_object,
        bod_edge_map,
        map_dimensions_,
        meter_index_ratio_ 
      );
        
      pce3d::space_map::doPreLoopMapUpdate(entity, rigid_object, bod_map, map_dimensions_, meter_index_ratio_);
    }
  }


  void drawMapPointsInSpace(const glm::dquat& cam_versor, const glm::dvec3& cam_transform) {
    for (auto const& [point, entity] : deadbod_map_) {
      const glm::dvec3 converted_point = pce3d::space_map::findPointOfIndex(point, map_dimensions_, meter_index_ratio_);
      glm::dvec3 rotated_point = converted_point - cam_transform;
      double distance = sqrt(glm::dot(rotated_point, rotated_point));
      rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
      const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
      std::vector<int> color = {12, 200, 200, 255};
      if (deadbod_index_vertex_map_.find(point) != deadbod_index_vertex_map_.end()) 
      { color = {200, 18, 59, 255}; }
      else if (deadbod_index_edge_map_.find(point) != deadbod_index_edge_map_.end()) 
      { color = {20, 183, 19, 255}; }
      pce::quickdraw::drawCircle(pixel, 10.0 / distance, color);
    }
    for (auto const& [point, entity] : livebod_map_) {
      const glm::dvec3 converted_point = pce3d::space_map::findPointOfIndex(point, map_dimensions_, meter_index_ratio_);
      glm::dvec3 rotated_point = converted_point - cam_transform;
      double distance = sqrt(glm::dot(rotated_point, rotated_point));
      rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
      const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
      std::vector<int> color = {12, 200, 200, 255};
      if (livebod_index_vertex_map_.find(point) != livebod_index_vertex_map_.end()) 
      { color = {200, 18, 59, 255}; }
      else if (livebod_index_edge_map_.find(point) != livebod_index_edge_map_.end()) 
      { color = {20, 183, 19, 255}; }
      pce::quickdraw::drawCircle(pixel, 10.0 / distance, color);
    }
  }


  void UpdateEntities() 
  {
    livebod_map_.clear();
    livebod_index_vertex_map_.clear();
    livebod_index_edge_map_.clear();
    livebod_index_face_map_.clear();
    potential_colliding_entities_.clear();
    restingbod_map_.clear();

    for (auto const& entity : entities) 
    {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      if (rigid_object.is_deadbod) { continue; }

      rigid_object.index_face_map.clear();
      rigid_object.face_index_map.clear();
      rigid_object.index_vertex_map.clear();
      rigid_object.vertex_index_map.clear();
      
      rigid_object.entity_face_collision_map.clear();
      rigid_object.entity_edge_collision_map.clear();
      rigid_object.entity_vertex_collision_map.clear();
      rigid_object.entity_index_collision_map.clear();
       
      
      std::unordered_map<uint32_t, glm::ivec3>  vertex_indices = pce3d::space_map::updateBodVertexMap(
        entity,
        rigid_object,
        livebod_index_vertex_map_,
        map_dimensions_,
        meter_index_ratio_ 
      );
      // std::cout << "entity: " << entity << '\n';
      // std::cout << "vertex indices updated" << '\n';

      std::unordered_map<uint32_t, std::vector<glm::ivec3>>  edge_indices = pce3d::space_map::updateBodEdgeMap(
        entity,
        rigid_object,
        livebod_index_edge_map_,
        map_dimensions_,
        meter_index_ratio_ 
      );
      // std::cout << "edge indices updated" << '\n';
      
      pce3d::space_map::updateLiveBodIndicesAndCheckForLiveBodCollision(
        entity,
        rigid_object,
        map_dimensions_,
        meter_index_ratio_,
        livebod_map_,
        livebod_index_vertex_map_,
        livebod_index_edge_map_,
        livebod_index_face_map_,
        potential_colliding_entities_
      );

      // std::cout << "live bod collision checked" << '\n';
      pce3d::space_map::checkForCollisionWithNonLiveBods(
        entity,
        rigid_object,
        vertex_indices,
        map_dimensions_,
        meter_index_ratio_,
        deadbod_map_,
        restingbod_map_,
        potential_colliding_entities_,
        livebod_index_vertex_map_,
        livebod_index_edge_map_
      );
    }

    // std::cout << "listing potential colliding entities" << '\n';
    // for (auto const& [ea, eb] : potential_colliding_entities_)
    // {
    //   std::cout << ea << ", " << eb << '\n';
    // }
  }


  std::unordered_map<glm::ivec3, std::vector<uint32_t>> deadbod_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> deadbod_index_vertex_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> deadbod_index_edge_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> deadbod_index_face_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> restingbod_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> restingbod_index_vertex_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> restingbod_index_edge_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> restingbod_index_face_map_;
  std::unordered_map<glm::ivec3, std::vector<uint32_t>> livebod_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> livebod_index_vertex_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> livebod_index_edge_map_;
  std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>> livebod_index_face_map_;
  std::unordered_map<uint32_t, uint32_t> potential_colliding_entities_;

  double meter_index_ratio_;
  glm::ivec3 map_dimensions_;

};
}
#endif /* SpaceMapSystem_cpp */
