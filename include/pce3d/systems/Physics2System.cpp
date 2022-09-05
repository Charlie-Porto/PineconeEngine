#ifndef Physics2System_cpp
#define Physics2System_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
updated physics system
-----------------------------------------------------------------*/

#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <pcecs/ecs/System.cpp>

#include <glm/vec3.hpp>

/* include Core Manager for access to time */
#include <pceSDL/core/CoreManager.hpp>

#include "functions/collisionFunctions.hpp"
#include "functions/physics2Functions.hpp"


extern ControlPanel control;

namespace pce3d {
class Physics2System : public ISystem {
public:
  Physics2System() : previous_time_(0.0), time_change_(0.0)
  {}
  
  void VetPotentialCollisions(
    const std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& potential_collision_entity_map,
    const std::unordered_map<uint32_t, glm::ivec3>& potential_collision_index_map
  )
  {
    for (auto const& [id, entity_pair] : potential_collision_entity_map)
    {
      auto const& entity_a = entity_pair.first;
      auto const& entity_b = entity_pair.second;

      auto const& a_rigid_object = control.GetComponent<pce::RigidObject>(entity_a);
      auto const& b_rigid_object = control.GetComponent<pce::RigidObject>(entity_b);
      auto const& a_motion = control.GetComponent<pce::Motion>(entity_a);
      auto const& b_motion = control.GetComponent<pce::Motion>(entity_b);
      auto const& a_position = control.GetComponent<pce::Position>(entity_a);
      auto const& b_position = control.GetComponent<pce::Position>(entity_b);
      
      /* do calc for particle-particle tip */
      if (a_rigid_object.radius != 0 && b_rigid_object.radius != 0)
      {
        std::pair<bool, glm::dvec3> if_colliding_and_where = collision::determineIfParticlesAreCollidingAndWhere(
          entity_a,
          a_rigid_object,
          a_motion,
          entity_b,
          b_rigid_object,
          b_motion
        );

        if (if_colliding_and_where.first)
        {
          collision_entity_pair_map_[id] = entity_pair;
          assert(potential_collision_index_map.find(id) != potential_collision_index_map.end());
          collision_location_map_[id] = potential_collision_index_map.at(id);
        }
      }
      /* do calc for particle-complexbod tip (b is complex bod) */
      else if (a_rigid_object.radius != 0)
      {
        std::pair<bool, glm::dvec3> if_colliding_and_where = collision::determineIfParticleIsCollidingWithComplexBodAndWhere(
          entity_a,
          a_rigid_object,
          a_motion,
          entity_b,
          b_rigid_object,
          b_motion,
          b_position
        );
        if (if_colliding_and_where.first)
        {
          collision_entity_pair_map_[id] = entity_pair;
          assert(potential_collision_index_map.find(id) != potential_collision_index_map.end());
          collision_location_map_[id] = potential_collision_index_map.at(id);
        }
      }
      /* do calc for particle-complexbod tip (a is complex bod) */
      else if (b_rigid_object.radius != 0)
      {
        std::pair<bool, glm::dvec3> if_colliding_and_where = collision::determineIfParticleIsCollidingWithComplexBodAndWhere(
          entity_b,
          b_rigid_object,
          b_motion,
          entity_a,
          a_rigid_object,
          a_motion,
          a_position
        );

        if (if_colliding_and_where.first)
        {
          collision_entity_pair_map_[id] = std::make_pair(entity_b, entity_a);
          assert(potential_collision_index_map.find(id) != potential_collision_index_map.end());
          collision_location_map_[id] = potential_collision_index_map.at(id);
        }
      } 
    }
  }

  void PrintCollisions()
  {
    for (auto const& [id, entities] : collision_entity_pair_map_)
    {
      std::cout << "entity " << entities.first << " -> " << "entity " << entities.second << '\n';
    }
  }
  
  void CalculateCollisionResults()
  {
    for (auto const& [id, epair] : collision_entity_pair_map_)
    {
      const uint32_t entity_a = epair.first;
      const uint32_t entity_b = epair.second;

      auto& a_rigid_object = control.GetComponent<pce::RigidObject>(entity_a);
      auto& b_rigid_object = control.GetComponent<pce::RigidObject>(entity_b);
      auto& a_position = control.GetComponent<pce::Position>(entity_a);
      auto& b_position = control.GetComponent<pce::Position>(entity_b);
      auto& a_motion = control.GetComponent<pce::Motion>(entity_a);
      auto& b_motion = control.GetComponent<pce::Motion>(entity_b);
      auto& a_surface = control.GetComponent<pce::Surface>(entity_a);
      auto& b_surface = control.GetComponent<pce::Surface>(entity_b);
      auto& a_force = control.GetComponent<pce::Force>(entity_a);
      auto& b_force = control.GetComponent<pce::Force>(entity_a);

      if (a_rigid_object.radius != 0 && b_rigid_object.radius != 0)
      {
        physics2::updateBothEntityInfoAfterTwoParticleCollision(
          a_position.actual_center_of_mass, 
          a_rigid_object.radius,
          a_motion, 
          a_rigid_object.mass,
          b_position.actual_center_of_mass, 
          b_rigid_object.radius,
          b_motion, 
          b_rigid_object.mass,
          a_surface.collision_elasticity_index * b_surface.collision_elasticity_index);
      }
      else if (a_rigid_object.radius != 0 || b_rigid_object.radius != 0)
      {

      }

    }
  }


  void UpdateEntities(
    const std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& potential_collision_entity_map,
    const std::unordered_map<uint32_t, glm::ivec3>& potential_collision_index_map
  )
  {
    time_change_ = pce::CoreManager::time_ - previous_time_;
    previous_time_ = pce::CoreManager::time_;

    collision_entity_pair_map_.clear();
    collision_location_map_.clear();
    
    if (!potential_collision_entity_map.empty())
    {
      VetPotentialCollisions(potential_collision_entity_map, potential_collision_index_map);
      
      if(!collision_entity_pair_map_.empty())
      {
        PrintCollisions();
        CalculateCollisionResults();
      }
    }
     
    for (auto const& entity : entities) 
    {
      /* update kinematics */
      auto& motion = control.GetComponent<pce::Motion>(entity);
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);
      auto const& force = control.GetComponent<pce::Force>(entity);

      const glm::dvec3 new_position = pce3d::physics::calculateParticlePositionGivenTime(
        motion.previous_resting_position, motion.velocity, time_change_,
        force.of_gravity, motion.duration);
      glm::dvec3 position_change = new_position - position.actual_center_of_mass;

      motion.direction = glm::normalize(position_change);
      motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;
      position.actual_center_of_mass = new_position;

      for (auto& [id, vertex] : rigid_object.vertices)
      {
        vertex = glm::dvec3(vertex + position_change);

        if (motion.rotational_speed > 0.01 && rigid_object.radius == 0)
        {
          const double rotation_amount = motion.rotational_speed * time_change_; 
          const glm::dvec3 normalized_vertex = vertex - position.actual_center_of_mass;
          const glm::dvec3 rotated_point
              = pce::rotateVector3byAngleAxis(normalized_vertex, rotation_amount, motion.rotational_axis);
          vertex = rotated_point + position.actual_center_of_mass;
        }
      }
      for (auto& [id, corner] : rigid_object.face_corner_map)
      {
        corner = glm::dvec3(corner + position_change);
        if (motion.rotational_speed > .01  && rigid_object.radius == 0)
        {
          const double rotation_amount = motion.rotational_speed * time_change_; 
          const glm::dvec3 normalized_corner = corner - position.actual_center_of_mass;
          const glm::dvec3 rotated_point
              = pce::rotateVector3byAngleAxis(normalized_corner, rotation_amount, motion.rotational_axis);
          corner = rotated_point + position.actual_center_of_mass;
        }
      }
    }
  }


private:
  std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> collision_entity_pair_map_;
  std::unordered_map<uint32_t, glm::dvec3> collision_location_map_;

  double previous_time_;
  double time_change_;

};
}
#endif /* Physics2System_cpp */
