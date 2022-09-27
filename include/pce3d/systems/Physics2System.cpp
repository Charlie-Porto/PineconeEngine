#ifndef Physics2System_cpp
#define Physics2System_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
updated physics system
-----------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <string>
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
      
      // const std::string a_type = a_rigid_object.radius == 0 ? "complex" : "particle";
      // const std::string b_type = b_rigid_object.radius == 0 ? "complex" : "particle";
      // std::cout << "atype: " << a_type << '\n';
      // std::cout << "btype: " << b_type << '\n';

      
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
          collision::CollisionReport collision_report = collision::CollisionReport{ .entity_a = entity_a, .entity_b = entity_b };
          collision_report.collision_occuring = true;
          collision_report.point_of_contact = if_colliding_and_where.second;
          collision_report.a_collision_type = collision::vertex;
          collision_report.a_collision_type_area_id = 1;
          collision_report.b_collision_type = collision::vertex;
          collision_report.b_collision_type_area_id = 1;

          collision_report_map_[id] = collision_report;
        }
      }
      /* do calc for particle-complexbod tip (b is complex bod) */
      else if (a_rigid_object.radius != 0 && b_rigid_object.radius == 0)
      {
        std::cout << "entityA: particle, entityB: complex" << '\n';
        collision::CollisionReport collision_report = collision::determineIfParticleIsCollidingWithComplexBodAndWhere(
          entity_a,
          a_rigid_object,
          a_motion,
          entity_b,
          b_rigid_object,
          b_motion,
          b_position
        );
        if (collision_report.collision_occuring)
        {
          collision_report_map_[id] = collision_report;
        }
      }
      /* do calc for particle-complexbod tip (a is complex bod) */
      else if (b_rigid_object.radius != 0 && a_rigid_object.radius == 0)
      {
        std::cout << "entityA: complex, entityB: particle" << '\n';
        collision::CollisionReport collision_report = collision::determineIfParticleIsCollidingWithComplexBodAndWhere(
          entity_b,
          b_rigid_object,
          b_motion,
          entity_a,
          a_rigid_object,
          a_motion,
          a_position
        );

        if (collision_report.collision_occuring)
        {
          collision_report_map_[id] = collision_report;
        }
      } 
      else if (a_rigid_object.radius == 0 && b_rigid_object.radius == 0)
      {
        collision::CollisionReport collision_report = collision::determineIfComplexBodsAreCollidingAndWhere(
          potential_collision_index_map.at(id),
          entity_a,
          a_rigid_object,
          a_motion,
          b_position,
          entity_b,
          b_rigid_object,
          b_motion,
          b_position
        );
        if (collision_report.collision_occuring)
        {
          // std::cout << "adding complex-complex collision to map" << '\n';
          collision_report_map_[id] = collision_report;
        }
      }
    }
  }

  void PrintCollisions()
  {
    for (auto const& [id, report] : collision_report_map_)
    {
      std::string a_collision_type_str = "vertex";
      std::string b_collision_type_str = "vertex";
      switch (report.a_collision_type)
      {
        case collision::edge:
          a_collision_type_str = "edge";
          break;
        case collision::face:
          a_collision_type_str = "face";
          break;
        default:
          break;
      }
      switch (report.b_collision_type)
      {
        case collision::edge:
          b_collision_type_str = "edge";
          break;
        case collision::face:
          b_collision_type_str = "face";
          break;
        default:
          break;
      }

      std::cout << "entity " << report.entity_a << ": " << a_collision_type_str
                << " -> " 
                << "entity " << report.entity_b << ": " << b_collision_type_str << '\n';
      
    }
  }
  
  void CalculateCollisionResults()
  {
    for (auto const& [id, collision_report] : collision_report_map_)
    {
      if (entity_collision_map_.find(collision_report.entity_a) != entity_collision_map_.end())
      {
        if (entity_collision_map_.at(collision_report.entity_a).find(collision_report.entity_b) 
         != entity_collision_map_.at(collision_report.entity_a).end())
        {
          continue;
        }
      }
      entity_collision_map_[collision_report.entity_a] = {{collision_report.entity_b, id}};
      entity_collision_map_[collision_report.entity_b] = {{collision_report.entity_a, id}};


      const uint32_t entity_a = collision_report.entity_a;
      const uint32_t entity_b = collision_report.entity_b;

      auto& a_rigid_object = control.GetComponent<pce::RigidObject>(entity_a);
      auto& b_rigid_object = control.GetComponent<pce::RigidObject>(entity_b);
      auto& a_position = control.GetComponent<pce::Position>(entity_a);
      auto& b_position = control.GetComponent<pce::Position>(entity_b);
      auto& a_motion = control.GetComponent<pce::Motion>(entity_a);
      auto& b_motion = control.GetComponent<pce::Motion>(entity_b);
      auto& a_surface = control.GetComponent<pce::Surface>(entity_a);
      auto& b_surface = control.GetComponent<pce::Surface>(entity_b);
      // auto const& a_mass_dist = control.GetComponent<pce::MassDistribution>(entity_a);
      // auto const& b_mass_dist = control.GetComponent<pce::MassDistribution>(entity_b);

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
      else if (a_rigid_object.radius == 0 && b_rigid_object.radius == 0)
      {
        physics2::updateBothEntityInfoAfterComplexbodComplexbodCollision(
          collision_report,
          a_rigid_object,
          a_motion,
          a_position,
          b_rigid_object,
          b_motion,
          b_position,
          a_surface.collision_elasticity_index * b_surface.collision_elasticity_index,
          b_rigid_object.is_deadbod,
          a_rigid_object.is_deadbod);
      }
      else if (a_rigid_object.radius != 0 || b_rigid_object.radius != 0)
      {
        // auto& particle_entity = entity_a;
        auto& particle_rigid_object = a_rigid_object;
        // auto& particle_position = a_position;
        auto& particle_motion = a_motion;
        auto& particle_surface = a_surface;
        // auto& complex_entity = entity_b;
        auto& complexbod_rigid_object = b_rigid_object;
        auto& complexbod_position = b_position;
        auto& complexbod_motion = b_motion;
        auto& complexbod_surface = b_surface;
        
        if (b_rigid_object.radius != 0)
        {
          // & particle_entity = entity_b;
          // & particle_rigid_object = b_rigid_object;
          // & particle_position = b_position;
          // & particle_motion = b_motion;
          // & particle_surface = b_surface;
          // & complex_entity = entity_a;
          // & complexbod_rigid_object = a_rigid_object;
          // & complexbod_position = a_position;
          // & complexbod_motion = a_motion;
          // & complexbod_surface = a_surface;
        }
        
        physics2::updateBothEntityInfoAfterParticleComplexbodCollision(
          collision_report,
          particle_rigid_object,
          particle_motion,
          complexbod_rigid_object,
          complexbod_position,
          complexbod_motion,
          particle_surface.collision_elasticity_index * complexbod_surface.collision_elasticity_index,
          complexbod_rigid_object.is_deadbod);
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

    collision_report_map_.clear();
    entity_collision_map_.clear();
    
    if (!potential_collision_entity_map.empty())
    {
      VetPotentialCollisions(potential_collision_entity_map, potential_collision_index_map);
      
      if(!collision_report_map_.empty())
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
      
      if (rigid_object.is_deadbod) { continue; }

      const glm::dvec3 new_position = pce3d::physics::calculateParticlePositionGivenTime(
        motion.previous_resting_position, motion.velocity, time_change_,
        force.of_gravity, motion.duration);
      glm::dvec3 position_change = new_position - position.actual_center_of_mass;

      motion.direction = glm::normalize(position_change);
      if (isnan(motion.direction.x)) { motion.direction = glm::dvec3(0, 0, 0); }
      motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;
      position.actual_center_of_mass = new_position;

      for (auto& [id, v] : rigid_object.vertices)
      {
        v = glm::dvec3(v + position_change);

        if (motion.rotational_speed > 0.01 && rigid_object.radius == 0)
        {
          const double rotation_amount = motion.rotational_speed * time_change_; 
          const glm::dvec3 normalized_vertex = v - position.actual_center_of_mass;
          const glm::dvec3 rotated_point
              = pce::rotateVector3byAngleAxis(normalized_vertex, rotation_amount, motion.rotational_axis);
          v = rotated_point + position.actual_center_of_mass;
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
  std::unordered_map<uint32_t, collision::CollisionReport> collision_report_map_;
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>> entity_collision_map_;

  double previous_time_;
  double time_change_;

};
}
#endif /* Physics2System_cpp */
