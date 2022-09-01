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
#include "../../maths/functions/sign.hpp"
#include "../../pce3d.hpp"


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
    const glm::dvec3& p_velocity_vect, const double p_mass,
    const std::vector<glm::dvec3>& face_vertices, double elasticity);


glm::dvec3 calculateStartPositionAfterLiveParticleDeadFaceCollision(
    const glm::dvec3 A
  , const glm::dvec3 B
  , const glm::dvec3 C
  , const glm::dvec3 plane_side_direction
  , const glm::dvec3 particle_position
  , const double particle_radius
);

void updateLiveParticleInfoAfterDeadFaceCollision(
    const glm::dvec3& p_center, const double p_radius, 
    const double mass, pce::Motion& motion,
    const std::vector<glm::dvec3>& face_vertices, double elasticity
);

std::pair<std::pair<bool, glm::dvec3>, std::pair<bool, glm::dvec3>> 
calculateLiveBodHitPointsAndIfVertex(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::RigidObject& b_rigid_object
  , pce::Position& a_position
  , pce::Position& b_position
);

glm::dvec3 calculateFaceNormalVector(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
);

std::pair<double, double> calculateLinearAndRotationalMomentumAllocationsAtPoint(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
  , const pce::Motion& motion
);

glm::dvec3 calculateMomentumVectorAtSurfacePoint(
    const glm::dvec3 point
  , const uint32_t face
  , const pce::RigidObject& rigid_object
  , const pce::Position& position
  , pce::Motion& motion
);

std::pair<glm::dvec3, glm::dvec3> calculateMomentumVectorsAfterLiveBodCollision(
    const double l_mass   
  , const double s_mass
  , const glm::dvec3 l_momentum
  , const glm::dvec3 s_momentum
  , const pce::Surface& l_surface
  , const pce::Surface& s_surface
  , const glm::dvec3& l_surface_point
  , const glm::dvec3& s_surface_point
  , const glm::dvec3& l_s_hitpoint_wire
);

void updateLinearMotionAfterCollision(
    const double linear_allocation_at_point
  , const double rotational_allocation_at_point
  , const glm::dvec3 new_total_momentum_at_point
  , const double mass
  , pce::Motion& motion
);

void updateRotationalMotionAfterCollision(
    const double rotational_allocation_at_point
  , const double linear_allocation_at_point
  , const glm::dvec3 point
  , const glm::dvec3 center_of_mass
  , const glm::dvec3 new_total_momentum_at_point
  , pce::Motion& motion
  , const double mass
);

void updateEntityDataFromLiveBodCollision(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::Position& a_position
  , pce::Surface& a_surface
  , pce::Motion& a_motion
  , pce::Force& a_force
  , pce::RigidObject& b_rigid_object
  , pce::Position& b_position
  , pce::Surface& b_surface
  , pce::Motion& b_motion
  , pce::Force& b_force
);

void updateComplexLivebodInfoAfterDeadfaceCollision(
    const uint32_t entity_a
  , const uint32_t entity_b
  , pce::RigidObject& a_rigid_object
  , pce::Position& a_position
  , pce::Surface& a_surface
  , pce::Motion& a_motion
  , const std::vector<glm::dvec3>& deadbod_face_vertices
  , double total_surface_elasticity
);

void checkForParticleCollisionWithHardBoundary(
    pce::Position& position
  , pce::RigidObject& rigid_object
  , pce::Motion& motion
  , pce::Surface& surface
);


}
}





#endif /* physicsFunctions_hpp */
