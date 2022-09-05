#ifndef vector_functions_cpp
#define vector_functions_cpp

#include "../vector_functions.hpp"


namespace pce3d {
namespace maths {

inline template <typename T> double calculateDistanceBetweenVectors(const T& A, const T& B) {
  return sqrt(glm::dot(A-B, A-B));
}

inline template <typename T> double calculateAngleDegreesBetweenVectors(const T& A, const T& B) {
  return acos( glm::dot(A, B) / (sqrt(glm::dot(A, A)) * sqrt(glm::dot(B, B)))) / 3.14159265 * 180.0;
}

inline double calcMagV3(const glm::dvec3& A)
{
  return sqrt(glm::dot(A, A));
}


}
}



#endif /* vector_functions_cpp */
