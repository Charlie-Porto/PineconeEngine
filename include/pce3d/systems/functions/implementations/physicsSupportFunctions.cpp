#ifndef physicsSupportFunctions_cpp
#define physicsSupportFunctions_cpp

#include "../physicsSupportFunctions.hpp"

namespace pce3d {
namespace physics2 {

void ensureParticleVelocityNotIntoObjectFace(
    const glm::dvec3& face_normal_vector
  , glm::dvec3& particle_velocity
)
{
  const double angle = maths::calculateAngleDegreesBetweenVectors(face_normal_vector, particle_velocity);
  std::cout << "new particle angle with face normal vector: " << angle << '\n';
  if (angle > 90.0)
  {
    const glm::dvec3 axis = glm::cross(particle_velocity, face_normal_vector);

    size_t i = 0;
    double direction = 1.0;
    while (i < 2)
    {
      const double rotation_degrees = (angle - 90.0) * direction;
      glm::dvec3 rotated_velocity = pce::rotateVector3byAngleAxis(
        particle_velocity, rotation_degrees, axis);
      
      const double new_angle = maths::calculateAngleDegreesBetweenVectors(face_normal_vector, rotated_velocity);
      std::cout << "new new angle: " << new_angle << '\n';
      if (new_angle <= 90.0) { particle_velocity = rotated_velocity; break; }
      else 
      {
        ++i;
        direction = -1.0;
        if (i == 2)
        {
          particle_velocity = -particle_velocity;
        }
      }
    }
  }
}



}
}



#endif /* physicsSupportFunctions_cpp */
