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

extern ControlPanel control;

namespace pce3d {
class PhysicsSystem : public ISystem {
public:

  PhysicsSystem() 
    : entities_updated_({}), previous_time_(0.0)
  { 
    std::cout << "setting up Physics System" << '\n';
  }

  void checkPotentialCollisions(const std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities_)
  {
    for (auto const& [entity_a, entity_b] : potential_colliding_entities_) 
    { 
      std::cout << "checking potential collision between: " << entity_a << ", " << entity_b << '\n';
      if (std::find(entities_updated_.begin(), entities_updated_.end(), entity_a) != entities_updated_.end()) 
      {
        continue;
      }
      auto& a_rigid_object = control.GetComponent<pce::RigidObject>(entity_a);
      auto& b_rigid_object = control.GetComponent<pce::RigidObject>(entity_b);
      auto& a_position = control.GetComponent<pce::Position>(entity_a);
      auto& b_position = control.GetComponent<pce::Position>(entity_b);
      auto& a_motion = control.GetComponent<pce::Motion>(entity_a);
      auto& a_surface = control.GetComponent<pce::Surface>(entity_a);
      auto& b_surface = control.GetComponent<pce::Surface>(entity_b);
      auto& a_force = control.GetComponent<pce::Force>(entity_a);
      
      /* handle live-dead collision */
      if (!a_rigid_object.is_deadbod && !a_rigid_object.is_restingbod
                                     && b_rigid_object.is_deadbod) {
        std::cout << "Physics System: checking for collision" << '\n';
        const bool are_colliding = physics::determineIfParticleIsCollidingWithFace(
          a_position.actual_center_of_mass, a_rigid_object.radius, a_motion.direction * a_motion.speed, a_rigid_object.mass,
          {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])});
        
        std::cout << "are colliding obtained " << '\n';
        if (are_colliding) 
        {
          std::cout << "Physics System: collision with deadbod" << '\n';

          bool execute_redirection = true;
          if (a_force.sequential_collisions_by_entity.find(entity_b) != a_force.sequential_collisions_by_entity.end() 
          && a_rigid_object.entity_time_collision_map.find(entity_b) != a_rigid_object.entity_time_collision_map.end())
          {
            if (a_force.sequential_collisions_by_entity.at(entity_b) >= 1 
             && pce::CoreManager::time_ - a_rigid_object.entity_time_collision_map.at(entity_b) < 0.5) 
            {
              // std::cout << "COLLISION NULLIFIED" << '\n';
              std::cout << a_force.sequential_collisions_by_entity.at(entity_b) << '\n';
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
            
            // std::cout << "speed: " << a_motion.speed << '\n';
            // std::cout << "direction: "<< a_motion.direction.x << ", " << a_motion.direction.y << ", " << a_motion.direction.z << '\n';

          }
        } else { a_force.sequential_collisions_by_entity[entity_b] = 0; }
      }
      
      /* handle active-active collision */
      /* CURRENTLY ONLY PARTICLES */
      if (!a_rigid_object.is_deadbod && !b_rigid_object.is_deadbod
      //  && !a_rigid_object.is_restingbod && !b_rigid_object.is_restingbod) {
       && !a_rigid_object.is_restingbod) 
      {
        auto& b_motion = control.GetComponent<pce::Motion>(entity_b);
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
          // std::cout << "entity" << entity_b << "switched to Livebod" << '\n';
          // std::cout << "Physics System: collision with livebod" << '\n';
             
          physics::updateBothEntityInfoAfterTwoParticleCollision(
            a_position.actual_center_of_mass, 
            a_rigid_object.radius,
            a_motion, 
            a_rigid_object.mass,
            b_position.actual_center_of_mass, 
            b_rigid_object.radius,
            b_motion, 
            b_rigid_object.mass);

          // std::cout << "Physics System: collision with livebod has been executed" << '\n';
          entities_updated_.push_back(entity_a);
          if (!b_rigid_object.is_deadbod) {
            entities_updated_.push_back(entity_b);
          }
        }

      }
    }
  }


  void UpdateEntities(const std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities) {
    // std::cout << "---" << '\n';
    entities_updated_.clear();
    if (isnan(pce::CoreManager::time_)) {
      pce::CoreManager::time_ = 0.01;
    }
    time_change_ = std::max(pce::CoreManager::time_ - previous_time_, 0.01);
    time_change_ = std::min(time_change_, 5.0);
    previous_time_ = pce::CoreManager::time_;
    checkPotentialCollisions(potential_colliding_entities);
    for (auto const& entity : entities) {
      auto const& force = control.GetComponent<pce::Force>(entity);
      auto& motion = control.GetComponent<pce::Motion>(entity);
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);
      
      if (!rigid_object.is_deadbod && !rigid_object.is_restingbod) {
        const glm::dvec3 new_position = pce3d::physics::calculateParticlePositionGivenTime(
          motion.previous_resting_position, motion.velocity, time_change_,
          force.of_gravity, motion.duration);
        
        const glm::dvec3 position_change = new_position - position.actual_center_of_mass;

        // std::cout <<  sqrt(glm::dot(position_change, position_change)) << '\n';
        // std::cout << "speed: " << motion.speed << '\n';
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
          motion.direction = glm::normalize(position_change);
          motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;
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
