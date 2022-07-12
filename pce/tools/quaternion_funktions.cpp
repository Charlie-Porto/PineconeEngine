#ifndef quaternion_funktions_cpp
#define quaternion_funktions_cpp

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>



namespace qfunc {

const double PI = 3.14159265;

glm::dquat zeroOutQuaternionRealComponent(glm::dquat& quat) {
  quat.w = 0.0;
  return quat;
}

glm::dquat convertVector3toQuaternion(const glm::dvec3& vect) {
  glm::dquat quat;
  quat.w = 0.0;
  quat.x = vect.x;
  quat.y = vect.y;
  quat.z = vect.z;
  return quat;
}
glm::dvec3 convertQuaternionToVector3(const glm::dquat& quat) {
  glm::dvec3 vect;
  vect.x = quat.x;
  vect.y = quat.y;
  vect.z = quat.z;
  return vect;
}

glm::dquat convertAngleAxisToQuaternion(double angle, glm::dvec3 axis) {
  const double radians_angle = (qfunc::PI * (angle / 180.0));
  glm::dquat rotation_quaternion;
  rotation_quaternion.w = cos(radians_angle / 2.0);
  rotation_quaternion.x = axis.x * sin(radians_angle / 2.0);
  rotation_quaternion.y = axis.y * sin(radians_angle / 2.0);
  rotation_quaternion.z = axis.z * sin(radians_angle / 2.0);
  rotation_quaternion = glm::normalize(rotation_quaternion);
  return rotation_quaternion;
}

glm::dvec3 rotateVector3byQuaternion(const glm::dvec3& vect, const glm::dquat& quat) {
  const glm::dquat vect_quat = qfunc::convertVector3toQuaternion(vect);
  glm::dquat inverse_quat = glm::inverse(quat);
  glm::dquat rotated_vect_quat = inverse_quat * vect_quat * quat;
  glm::dvec3 rotated_vect = qfunc::convertQuaternionToVector3(rotated_vect_quat);
  return rotated_vect;
}

glm::dvec3 rotateVector3byAngleAxis(const glm::dvec3& vect, const double& angle, 
                                     const glm::dvec3& axis) {
  glm::dquat rotation_versor = convertAngleAxisToQuaternion(angle, axis);
  return rotateVector3byQuaternion(vect, rotation_versor);
}

}





#endif /* quaternion_funktions_cpp */
