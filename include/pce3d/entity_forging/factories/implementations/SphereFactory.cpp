#ifndef SphereFactory_cpp
#define SphereFactory_cpp

#include "../SphereFactory.hpp"


namespace pce3d {
namespace efactory {

SphereFactory::SphereFactory(
    double radius_max
  , glm::dvec3 factory_location
  , std::vector<int> color
  , glm::dvec3 velocity
  , double velocity_component_variance
  , double gravitational_force
  , double bounciness
)
  :   radius_max_{radius_max}
    , factory_location_{factory_location}
    , color_{color}
    , velocity_{velocity}
    , velocity_component_variance_{velocity_component_variance}
    , gravitational_force_{gravitational_force}
    , bounciness_{bounciness}
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
  std::cout << "velocity_x: " << velocity_.x << '\n';
  std::cout << "velocity_component_variance: " << velocity_component_variance_ << '\n';
  // const glm::dvec3 e_velocity = glm::dvec3(vx, vz, vy);
  // const glm::dvec3 e_velocity = velocity_;
  glm::dvec3 e_velocity = pce3d::random::getRandomUnitVector3() * v_magnitude;
  if ( isnan(e_velocity.x) || e_velocity.x == 0) { e_velocity.x = 1.0; }
  if ( isnan(e_velocity.y) || e_velocity.y == 0) { e_velocity.y = 1.0; }
  if ( isnan(e_velocity.z) || e_velocity.z == 0) { e_velocity.z = 1.0; }
  const double e_gforce = gravitational_force_;

  pce3d::forge::forgeSphereEntity(
      radius_max_
    , e_position
    , color_
    // , pce3d::random::getRandomColor()
    , e_velocity
    , e_gforce
    , bounciness_);
}
    



}
}





#endif /* SphereFactory_cpp */
