#ifndef physicsFunctions_hpp
#define physicsFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions to assist the physics system
-----------------------------------------------------------------*/

#include <cmath>
#include <utility>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "../../maths/functions/plane_functions.hpp"
#include "../../maths/functions/quaternion_functions.hpp"


namespace pce3d {
namespace physics {

const double GRAVITY = -9.81;
const double PI = 3.14159265;

glm::dvec3 calculateParticlePositionGivenTime(
    const glm::dvec3& initial_position, const glm::dvec3& initial_velocity, 
    double time_change, double gravitational_force_applied,
    double& duration);


bool determineIfParticlesAreColliding(
    const glm::dvec3& a_position, const double a_radius,
    const glm::dvec3& b_position, const double b_radius);


std::pair<glm::dvec3, glm::dvec3> calculateVelocityVectorsAfterTwoParticleCollision(
    const glm::dvec3& a_center, const double a_radius, const glm::dvec3& a_velocity_vect, const double a_mass,
    const glm::dvec3& b_center, const double b_radius, const glm::dvec3& b_velocity_vect, const double b_mass);


void updateBothEntityInfoAfterTwoParticleCollision(
    const glm::dvec3& a_center, const double a_radius, pce::Motion& a_motion, const double a_mass,
    const glm::dvec3& b_center, const double b_radius, pce::Motion& b_motion, const double b_mass);


bool determineIfParticleIsCollidingWithFace(
    const glm::dvec3& p_center, const double p_radius, 
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices);


glm::dvec3 calculateVelocityVectorAfterLiveParticleDeadFaceCollision(
    const glm::dvec3& p_center, const double p_radius, 
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices, double elasticity);


glm::dvec3 calculateStartPositionAfterLiveParticleDeadFaceCollision(
    const glm::dvec3 A
  , const glm::dvec3 B
  , const glm::dvec3 C
  , const glm::dvec3 plane_side_direction
  , const glm::dvec3 particle_position
  , const double particle_radius);


void updateLiveParticleInfoAfterDeadFaceCollision(
    const glm::dvec3& p_center, const double p_radius, 
    const double mass, pce::Motion& motion,
    const std::vector<glm::dvec3>& face_vertices, double elasticity);
     



}
}





#endif /* physicsFunctions_hpp */
