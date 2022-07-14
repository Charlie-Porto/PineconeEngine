#ifndef quaternion_funktions_hpp
#define quaternion_funktions_hpp

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>



namespace pce {

const double PI = 3.14159265;


glm::dquat convertVector3toQuaternion(const glm::dvec3& vect);

glm::dvec3 convertQuaternionToVector3(const glm::dquat& quat);

glm::dquat convertAngleAxisToQuaternion(double angle, glm::dvec3 axis);

glm::dvec3 rotateVector3byQuaternion(const glm::dvec3& vect, const glm::dquat& quat);

glm::dvec3 rotateVector3byAngleAxis(const glm::dvec3& vect, const double& angle, 
                                    const glm::dvec3& axis);



}





#endif /* quaternion_functions_hpp */
