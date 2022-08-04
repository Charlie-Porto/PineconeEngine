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

extern ControlPanel control;

namespace pce3d {
class PhysicsSystem : public ISystem {
public:

  PhysicsSystem() { 
    std::cout << "setting up Physics System" << '\n';
    entities_updated_ = {};
  }

  void checkPotentialCollisions(const std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities_) {
    for (auto const& [entity_a, entity_b] : potential_colliding_entities_) { 
      if (std::find(entities_updated_.begin(), entities_updated_.end(), entity_a) != entities_updated_.end()) {
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
      auto& b_force = control.GetComponent<pce::Force>(entity_b);
      
      /* handle active particle collision with dead entity face */
      if (!a_rigid_object.is_deadbod && !a_rigid_object.is_restingbod
                                     && b_rigid_object.is_deadbod) {
        const bool are_colliding = physics::determineIfParticleIsCollidingWithFace(
          a_position.actual_center_of_mass, a_rigid_object.radius, a_motion.direction * a_motion.speed, a_rigid_object.mass,
          {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
           b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])});
        
        
        if (are_colliding) {
          const double net_elasticity = a_surface.collision_elasticity_index * b_surface.collision_elasticity_index;
          const glm::dvec3 nvelocity = physics::calculateVelocityVectorAfterLiveParticleDeadFaceCollision( 
            a_position.actual_center_of_mass, a_rigid_object.radius, a_motion.direction * a_motion.speed, a_rigid_object.mass,
            {b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[0]),
            b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[1]),
            b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(b_rigid_object.entity_face_collision_map.at(entity_a))[2])},
            net_elasticity);
          
          a_motion.velocity = nvelocity;
          a_motion.direction = glm::normalize(a_motion.velocity);
          a_motion.previous_resting_position = a_position.actual_center_of_mass;
          a_motion.duration = 0.1;
          a_motion.speed = sqrt(glm::dot(a_motion.velocity, a_motion.velocity));
          
          if (a_force.sequential_collisions_by_entity.find( entity_b) == a_force.sequential_collisions_by_entity.end()) {
            a_force.sequential_collisions_by_entity[entity_b] = 1;
          } else { ++a_force.sequential_collisions_by_entity.at(entity_b); }
        } else { a_force.sequential_collisions_by_entity[entity_b] = 0; }

        if (a_force.sequential_collisions_by_entity[entity_b] > 5.0) {
          a_rigid_object.is_restingbod = true;
        }
      }
      
      /* handle active-active collision */
      /* CURRENTLY ONLY PARTICLES */
      if (!a_rigid_object.is_deadbod && !b_rigid_object.is_deadbod
       && !a_rigid_object.is_restingbod && !b_rigid_object.is_restingbod) {
        const bool are_colliding = physics::determineIfParticlesAreColliding(
          a_position.actual_center_of_mass, a_rigid_object.radius, 
          b_position.actual_center_of_mass, b_rigid_object.radius);

        if (are_colliding) {
          std::pair<glm::dvec3, glm::dvec3> new_velocity_vectors 
            = physics::calculateVelocityVectorsAfterTwoParticleCollision(
                a_position.actual_center_of_mass, a_rigid_object.radius,
                a_motion.velocity, a_rigid_object.mass,
                b_position.actual_center_of_mass, b_rigid_object.radius,
                b_motion.velocity, b_rigid_object.mass);
          
          a_motion.velocity += new_velocity_vectors.second;
          a_motion.velocity -= new_velocity_vectors.first;
          a_motion.direction = glm::normalize(a_motion.velocity);
          b_motion.velocity += new_velocity_vectors.first;
          b_motion.velocity -= new_velocity_vectors.second;
          b_motion.direction = glm::normalize(b_motion.velocity);

          a_motion.previous_resting_position = a_position.actual_center_of_mass;
          b_motion.previous_resting_position = b_position.actual_center_of_mass;
          a_motion.duration = 0.1;
          b_motion.duration = 0.1;

          entities_updated_.push_back(entity_a);
          if (!b_rigid_object.is_deadbod) {
            entities_updated_.push_back(entity_b);
          }
        }

      }
    }
  }


  void UpdateEntities(const std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities) {
    entities_updated_.clear();
    checkPotentialCollisions(potential_colliding_entities);
    time_change_ = pce::CoreManager::time_ - previous_time_;
    previous_time_ = pce::CoreManager::time_;
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
        position.actual_center_of_mass = new_position;

        // std::cout <<  sqrt(glm::dot(position_change, position_change)) << '\n';
        // std::cout << "speed: " << motion.speed << '\n';
        if (sqrt(glm::dot(position_change, position_change)) < 0.002) {
          ++motion.stationary_counter;
        } else { motion.stationary_counter = 0; }
        if (motion.stationary_counter > 10) {
          rigid_object.is_restingbod = true;
          continue;
        } else {

          motion.direction = glm::normalize(position_change);
          motion.speed = sqrt(glm::dot(position_change, position_change)) / time_change_;
          
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
