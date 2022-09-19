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
  std::cout << "face_normal_vector: "
            << face_normal_vector.x << ", "
            << face_normal_vector.y << ", "
            << face_normal_vector.z << '\n';
  std::cout << "particle_velocity: "
            << particle_velocity.x << ", "
            << particle_velocity.y << ", "
            << particle_velocity.z << '\n';
  const double angle = maths::calculateAngleDegreesBetweenVectors(face_normal_vector, particle_velocity);
  std::cout << "new particle angle with face normal vector: " << angle << '\n';
  if (abs(180.0 - angle) < .001)
  {
    particle_velocity = -particle_velocity; 
    std::cout << "flipping particle velocity" << '\n';
  }
  else if (angle > 90.0)
  {
    const glm::dvec3 axis = glm::cross(particle_velocity, face_normal_vector);

    size_t i = 0;
    double direction = 1.0;
    while (i < 2)
    {
      const double rotation_degrees = (angle - 90.0) * direction;
      glm::dvec3 rotated_velocity = pce::rotateVector3byAngleAxis(
        particle_velocity, rotation_degrees, axis);

      std::cout << "rotation angle: " << rotation_degrees << '\n';
  std::cout << "axis: "
            << axis.x << ", "
            << axis.y << ", "
            << axis.z << '\n';
      
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
  std::cout << "updated particle velocity: "
            << particle_velocity.x << ", "
            << particle_velocity.y << ", "
            << particle_velocity.z << '\n';
}



}
}



#endif /* physicsSupportFunctions_cpp */
