#ifndef SphereFactory_cpp
#define SphereFactory_cpp

#include "../SphereFactory.hpp"


namespace pce3d {
namespace efactory {

SphereFactory::SphereFactory(
    double radius_max
  , double radius_min
  , glm::dvec3 factory_location
  , double x_variance
  , double y_variance
  , double z_variance
  , glm::dvec3 velocity
  , double velocity_component_variance
  , double gravitational_force
  , double gravitational_force_variance
)
  :   radius_max_{radius_max}
    , radius_min_{radius_min} 
    , factory_location_{factory_location}
    , x_variance_{x_variance}
    , y_variance_{y_variance}
    , z_variance_{z_variance}
    , velocity_{velocity}
    , velocity_component_variance_{velocity_component_variance}
    , gravitational_force_{gravitational_force}
    , gravitational_force_variance_{gravitational_force_variance}
{
  std::cout << "sphere factory created" << '\n';
}
  

void SphereFactory::CreateSphereParticle() {
  // const double xpos = pce3d::random::getRandomDoubleByTargetWithVariance(factory_location_.x, x_variance_);
  // const double ypos = pce3d::random::getRandomDoubleByTargetWithVariance(factory_location_.y, y_variance_);
  // const double zpos = pce3d::random::getRandomDoubleByTargetWithVariance(factory_location_.z, z_variance_);
  const double xpos = factory_location_.x;
  const double ypos = factory_location_.y;
  const double zpos = factory_location_.z;
  const glm::dvec3 e_position = glm::dvec3(xpos, ypos, zpos);
  // const double vx = pce3d::random::getRandomDoubleByTargetWithVariance(velocity_.x, velocity_component_variance_);
  // const double vy = pce3d::random::getRandomDoubleByTargetWithVariance(velocity_.y, velocity_component_variance_);
  // const double vz = pce3d::random::getRandomDoubleByTargetWithVariance(velocity_.z, velocity_component_variance_);
  const double v_magnitude = pce3d::random::getRandomDoubleBetweenDoubles(20.0, 40.0);
  // const glm::dvec3 e_velocity = glm::dvec3(vx, vz, vy);
  // const glm::dvec3 e_velocity = velocity_;
  glm::dvec3 e_velocity = pce3d::random::getRandomUnitVector3() * v_magnitude;
  if (isnan(e_velocity.x) || e_velocity.x == 0) {e_velocity.x = 1.0; }
  if (isnan(e_velocity.y) || e_velocity.y == 0) {e_velocity.y = 1.0; }
  if (isnan(e_velocity.z) || e_velocity.z == 0) {e_velocity.z = 1.0; }
  const double e_gforce = gravitational_force_;


  pce3d::forge::forgeSphereEntity(
      radius_max_
    , e_position
    , {200, 10, 100, 255}
    // , pce3d::random::getRandomColor()
    , e_velocity
    , e_gforce);
}
    



}
}





#endif /* SphereFactory_cpp */
