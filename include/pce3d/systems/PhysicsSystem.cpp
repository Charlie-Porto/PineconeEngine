#ifndef PhysicsSystem_cpp
#define PhysicsSystem_cpp

/*----------------------------------------------------------------|
--------------------- System Description -------------------------|
system for handling physics
-----------------------------------------------------------------*/

#include <algorithm>
#include <iostream>
#include <pcecs/ecs/System.cpp>

/* include Core Manager for access to time */
#include <pceSDL/core/CoreManager.hpp>

#include "functions/physicsFunctions.hpp"
#include "../maths/functions/vector_functions.hpp"
#include "../maths/functions/quaternion_functions.hpp"
#include "functions/spaceMapFunctions.hpp"

extern ControlPanel control;

namespace pce3d {
class PhysicsSystem : public ISystem {
public:

  PhysicsSystem() 
    : entities_updated_({}), previous_time_(0.0)
  { 
    std::cout << "setting up Physics System" << '\n';
  }

  void checkPotentialCollisions(
    std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> potential_collision_entity_map,
    std::unordered_map<uint32_t, glm::ivec3> potential_collision_index_map
  )
  {
    for (auto const& [collision_id, entity_pair] : potential_collision_entity_map) 
    { 
      const uint32_t entity_a = entity_pair.first;
      const uint32_t entity_b = entity_pair.second;
      // std::cout << "checking potential collision between: " << entity_a << ", " << entity_b << '\n';
      if (std::find(entities_updated_.begin(), entities_updated_.end(), entity_a) != entities_updated_.end()) 
      {
        continue;
      }
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

      if (a_rigid_object.is_complex_livebod && b_rigid_object.is_complex_livebod)
      {
        if (a_force.sequential_collisions_by_entity.find(entity_b) == a_force.sequential_collisions_by_entity.end() 
        ||  a_force.sequential_collisions_by_entity.at(entity_b) == 0)
        {
          std::cout << "PHYSICS SYSTEM: CHECKING FOR COLLISION WITH COMPLEX LIVEBOD" << '\n';

          pce3d::physics::updateEntityDataFromLiveBodCollision(
            entity_a,
            entity_b,
            a_rigid_object,
            a_position,
            a_surface,
            a_motion,
            a_force,
            b_rigid_object,
            b_position,
            b_surface,
            b_motion,
            b_force
          );
        }
        ++a_force.sequential_collisions_by_entity[entity_b];
        if (a_force.sequential_collisions_by_entity.at(entity_b) > 1)
        {
          a_force.sequential_collisions_by_entity[entity_b] = 0;
        }

        continue;
      }

      else if (a_rigid_object.is_complex_livebod && b_rigid_object.is_deadbod)
      {
        if (a_force.sequential_collisions_by_entity.find(entity_b) == a_force.sequential_collisions_by_entity.end() 
          ||  a_force.sequential_collisions_by_entity.at(entity_b) == 0) {
          /* assert collision is logged as either a vertex, edge, or face collision */
          assert(a_rigid_object.entity_vertex_collision_map.find(entity_b) != a_rigid_object.entity_vertex_collision_map.end()
              || a_rigid_object.entity_edge_collision_map.find(entity_b) != a_rigid_object.entity_edge_collision_map.end()
              || a_rigid_object.entity_face_collision_map.find(entity_b) != a_rigid_object.entity_face_collision_map.end());
          
          const double net_elasticity = a_surface.collision_elasticity_index * b_surface.collision_elasticity_index;
          
          pce3d::physics::updateComplexLivebodInfoAfterDeadfaceCollision(
            entity_a,
            entity_b,
            a_rigid_object,
            a_position,
            a_surface,
            a_motion,
            {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
              b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
              b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])},
            net_elasticity   
          );
        }
        ++a_force.sequential_collisions_by_entity[entity_b];
        if (a_force.sequential_collisions_by_entity.at(entity_b) > 1
         && a_motion.speed > 1.0)
        {
          a_force.sequential_collisions_by_entity[entity_b] = 0;
        }

        
      }
      
      /* handle live-dead collision */
      else if (!a_rigid_object.is_deadbod && !a_rigid_object.is_restingbod
                                          && b_rigid_object.is_deadbod) {
        // std::cout << "Physics System: checking for collision between livebod and deadbod" << '\n';
        // std::cout << "entity a: " << entity_a << " | " << "entity b: " << entity_b << '\n';
        assert(b_rigid_object.entity_face_collision_map.find(entity_a) != b_rigid_object.entity_face_collision_map.end());
        assert(b_rigid_object.face_vertex_map.find(b_rigid_object.entity_face_collision_map.at(entity_a)) != b_rigid_object.face_vertex_map.end());
        const bool are_colliding = physics::determineIfParticleIsCollidingWithFace(
          a_position.actual_center_of_mass, a_rigid_object.radius, a_motion.direction * a_motion.speed, a_rigid_object.mass,
          {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])});
        
        if (are_colliding) 
        {
          // std::cout << "Physics System: collision with deadbod" << '\n';

          bool execute_redirection = true;
          bool nerf_new_speed = false;
          if (a_force.sequential_collisions_by_entity.find(entity_b) != a_force.sequential_collisions_by_entity.end() 
          && a_rigid_object.entity_time_collision_map.find(entity_b) != a_rigid_object.entity_time_collision_map.end())
          {
            if (a_force.sequential_collisions_by_entity.at(entity_b) >= 1 
             && pce::CoreManager::time_ - a_rigid_object.entity_time_collision_map.at(entity_b) < 0.7) 
            {
              std::cout << "sequential collisions incremented" << '\n';
              std::cout << a_force.sequential_collisions_by_entity.at(entity_b) << '\n';
              a_motion.velocity = a_motion.velocity * 0.5;
              nerf_new_speed = true;
              // execute_redirection = false;
              ++a_force.sequential_collisions_by_entity[entity_b];
              a_rigid_object.entity_time_collision_map[entity_b] = pce::CoreManager::time_;
              if (a_force.sequential_collisions_by_entity.at(entity_b) > 20) {
                std::cout << "entity" << entity_b << "switched to RestingBod" << '\n';
                a_rigid_object.is_restingbod = true;
                a_force.sequential_collisions_by_entity[entity_b] = 0;
              }
              // else if (a_force.sequential_collisions_by_entity.at(entity_b) > 2
              //  && (int(a_force.sequential_collisions_by_entity.at(entity_b)) % 3 == 0)) {
              //  && a_motion.stationary_counter < 3) {
                // execute_redirection = true;
                // std::cout << "COLLISION DE-NULLIFIED!!!!!" << '\n';
                // a_force.sequential_collisions_by_entity[entity_b] = 2;
              // }
            } 
            else 
            {
              ++a_force.sequential_collisions_by_entity[entity_b];
              a_rigid_object.entity_time_collision_map[entity_b] = pce::CoreManager::time_;
            }
          } 
          else 
          {
            a_force.sequential_collisions_by_entity[entity_b] = 1;
            a_rigid_object.entity_time_collision_map[entity_b] = pce::CoreManager::time_;
          }
          if (execute_redirection) 
          {
            const double net_elasticity = a_surface.collision_elasticity_index * b_surface.collision_elasticity_index;
            // std::cout << "entity_a: " << entity_a<< '\n';
            // std::cout << "pre speed: " << a_motion.speed << '\n';
            // std::cout << "pre direction: "<< a_motion.direction.x << ", " << a_motion.direction.y << ", " << a_motion.direction.z << '\n';

            physics::updateLiveParticleInfoAfterDeadFaceCollision( 
              a_position.actual_center_of_mass, 
              a_rigid_object.radius,
              a_rigid_object.mass,
              a_motion,
              {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
              b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
              b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])},
              net_elasticity);

            if (nerf_new_speed) 
            { 
              glm::vec3 nerfed_velocity =  a_motion.velocity * ((1.0 / a_force.sequential_collisions_by_entity.at(entity_b)) * 2.0);
              const double nerfed_velocity_length = sqrt(glm::dot(nerfed_velocity, nerfed_velocity));
              const double a_motion_velocity_length = sqrt(glm::dot(a_motion.velocity, a_motion.velocity));
              if (nerfed_velocity_length < a_motion_velocity_length)
              {
                a_motion.velocity = nerfed_velocity;
              }
            }
            // std::cout << "speed: " << a_motion.speed << '\n';
            // std::cout << "direction: "<< a_motion.direction.x << ", " << a_motion.direction.y << ", " << a_motion.direction.z << '\n';

          }
        } 
        else 
        { 
          a_force.sequential_collisions_by_entity[entity_b] = 0; 
          a_rigid_object.entity_time_collision_map[entity_b] = 0.0;
        }
      }
      
      // /* handle active-active particle collision */
      if (!a_rigid_object.is_deadbod && !b_rigid_object.is_deadbod && !b_rigid_object.is_complex_livebod
       && !a_rigid_object.is_restingbod && !a_rigid_object.is_complex_livebod) 
      {
        // std::cout << "entity_a: " << entity_a<< '\n';
        // std::cout << "speed: " << a_motion.speed << '\n';
        // std::cout << "direction: "<< a_motion.direction.x << ", " << a_motion.direction.y << ", " << a_motion.direction.z << '\n';
        // std::cout << "entity_b: " << entity_b << '\n';
        // std::cout << "speed: " << b_motion.speed << '\n';
        // std::cout << "direction: "<< b_motion.direction.x << ", " << b_motion.direction.y << ", " << b_motion.direction.z << '\n';
        // std::cout << "a_position: "
        //           << a_position.actual_center_of_mass.x << ", " 
        //           << a_position.actual_center_of_mass.y << ", " 
        //           << a_position.actual_center_of_mass.z << '\n';
        // std::cout << "b_position: "
        //           << b_position.actual_center_of_mass.x << ", " 
        //           << b_position.actual_center_of_mass.y << ", " 
        //           << b_position.actual_center_of_mass.z << '\n';
        // std::cout << "Physics System: collision with livebod" << entity_a << ", " << entity_b <<'\n';
        const bool are_colliding = physics::determineIfParticlesAreColliding(
          a_position.actual_center_of_mass, a_rigid_object.radius, 
          b_position.actual_center_of_mass, b_rigid_object.radius);

        if (are_colliding) 
        {
          b_rigid_object.is_restingbod = false;
          physics::updateBothEntityInfoAfterTwoParticleCollision(
            a_position.actual_center_of_mass, 
            a_rigid_object.radius,
            a_motion, 
            a_rigid_object.mass,
            b_position.actual_center_of_mass, 
            b_rigid_object.radius,
            b_motion, 
            b_rigid_object.mass);
          entities_updated_.push_back(entity_a);

          if (!b_rigid_object.is_deadbod) 
          {
            entities_updated_.push_back(entity_b);
          }
        }

      }
    }
  }


  void UpdateEntities(
    const std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& potential_collision_entity_map,
    const std::unordered_map<uint32_t, glm::ivec3>& potential_collision_index_map
  )
  {
    // std::cout << "---" << '\n';
    entities_updated_.clear();
    if (isnan(pce::CoreManager::time_)) {
      pce::CoreManager::time_ = 0.01;
    }
    time_change_ = std::max(pce::CoreManager::time_ - previous_time_, 0.01);
    time_change_ = std::min(time_change_, 5.0);
    previous_time_ = pce::CoreManager::time_;

    if (!potential_collision_entity_map.empty())
    {
      checkPotentialCollisions(potential_collision_entity_map, potential_collision_index_map);
    }

    for (auto const& entity : entities) 
    {
      // std::cout << "updating entity: " << entity << '\n';
      auto const& force = control.GetComponent<pce::Force>(entity);
      auto& motion = control.GetComponent<pce::Motion>(entity);
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);
      auto& surface = control.GetComponent<pce::Surface>(entity);
      // std::cout << "velocity: "
      //           << motion.velocity.x << ", "
      //           << motion.velocity.y << ", "
      //           << motion.velocity.z << '\n';

      const glm::dvec3 new_position = pce3d::physics::calculateParticlePositionGivenTime(
        motion.previous_resting_position, motion.velocity, time_change_,
        force.of_gravity, motion.duration);

      // std::cout << "velocity: "
      //           << motion.velocity.x << ", "
      //           << motion.velocity.y << ", "
      //           << motion.velocity.z << '\n';
      glm::dvec3 position_change = new_position - position.actual_center_of_mass;
      if (isnan(position_change.x)) {position_change.x = motion.velocity.x; }
      if (isnan(position_change.y)) {position_change.y = motion.velocity.y; }
      if (isnan(position_change.z)) {position_change.z = motion.velocity.z; }

      if (rigid_object.is_complex_livebod)
      {
        position.actual_center_of_mass = new_position;
        
      std::cout << "new_position: "
                << new_position.x << ", "
                << new_position.y << ", "
                << new_position.z << '\n';

        motion.direction = glm::normalize(position_change);
        motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;

        for (auto& [id, vertex] : rigid_object.vertices)
        {
          // std::cout << "vertex before: " << vertex.x << ", " << vertex.y << ", " << vertex.z << '\n';
          vertex = vertex + position_change;
          // std::cout << "vertex after: " << vertex.x << ", " << vertex.y << ", " << vertex.z << '\n';
          if (motion.rotational_speed > 0.01)
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
          corner = corner + position_change;
          if (motion.rotational_speed > .01)
          {
            const double rotation_amount = motion.rotational_speed * time_change_; 
            const glm::dvec3 normalized_corner = corner - position.actual_center_of_mass;
            const glm::dvec3 rotated_point
                = pce::rotateVector3byAngleAxis(normalized_corner, rotation_amount, motion.rotational_axis);
            corner = rotated_point + position.actual_center_of_mass;
          }
        }


        continue;
      }
      
      if (!rigid_object.is_deadbod && !rigid_object.is_restingbod) {
        if (std::find(entities_updated_.begin(), entities_updated_.end(), entity) == entities_updated_.end())
        {
          pce3d::physics::checkForParticleCollisionWithHardBoundary(
            position,
            rigid_object,
            motion,
            surface
          );
        }
        if (sqrt(glm::dot(position_change, position_change)) < 0.01) 
        {
          ++motion.stationary_counter;
        } 
        else 
        { 
          motion.stationary_counter = 0; 
          position.actual_center_of_mass = new_position;
        }
        if (motion.stationary_counter > 15) {
          std::cout << "entity" << entity << "switched to RestingBod" << '\n';
          rigid_object.is_restingbod = true;
          continue;
        } 
        else 
        {
          std::cout << "position_change: "
                    << position_change.x << ", "
                    << position_change.y << ", "
                    << position_change.z << '\n';
          if (sqrt(glm::dot(position_change, position_change)) > 0.01)
          {
            motion.direction = glm::normalize(position_change);
            motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;
          }
          if (isnan(motion.direction.x)) { motion.direction.x = position.actual_center_of_mass.x - motion.previous_resting_position.x; }
          if (isnan(motion.direction.y)) { motion.direction.y = position.actual_center_of_mass.x - motion.previous_resting_position.y; }
          if (isnan(motion.direction.z)) { motion.direction.z = position.actual_center_of_mass.x - motion.previous_resting_position.z; }
          if (isnan(motion.speed)) { motion.speed = 0.0; }
          // std::cout << "entity: " << entity << '\n';
          // std::cout << "speed: " << motion.speed << '\n';
          // std::cout << "direction: "<< motion.direction.x << ", " << motion.direction.y << ", " << motion.direction.z << '\n';
         
          for (auto& [id, vertex] : rigid_object.vertices) {
            vertex = glm::dvec3(vertex + position_change);
          }
        }
      }
    }
  }

private:
  std::vector<uint32_t> entities_updated_;
  double previous_time_;
  double time_change_;
};
}
#endif /* PhysicsSystem_cpp */
