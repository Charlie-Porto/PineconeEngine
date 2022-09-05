#ifndef vector_functions_cpp
#define vector_functions_cpp

#include "../vector_functions.hpp"


namespace pce3d {
namespace maths {


const double PI = 3.14159265;


template <typename T> double calculateDistanceBetweenVectors(const T& A, const T& B) {
  return sqrt(glm::dot(A-B, A-B));
}



template <typename T> double calculateAngleDegreesBetweenVectors(const T& A, const T& B) {
  return acos( glm::dot(A, B) / (sqrt(glm::dot(A, A)) * sqrt(glm::dot(B, B)))) / 3.14159265 * 180.0;
}



inline double calcMagV3(const glm::dvec3& A)
{
  return sqrt(glm::dot(A, A));
}



inline bool determineIfVec3MakesLineWithTwoVec3s(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C)
{
  const glm::dvec3 a_to_c_direction = glm::normalize(C - A);
  const glm::dvec3 a_to_b_direction = glm::normalize(B - A);
  
  if (a_to_c_direction == a_to_b_direction || a_to_c_direction == -a_to_b_direction)
  {
    return true;
  }
  else 
  {
    return false;
  }
}



glm::dvec3 findClosestPointOnVec3LineToVec3(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C)
{
  const glm::dvec3 BC = (C-B);
  const glm::dvec3 BA = (A-B);
  const double mag_BC = calcMagV3(BC);
  const double angle = calculateAngleDegreesBetweenVectors(BC, BA);
  const double w = cos(angle / 180.0 * PI) * mag_BC;
  
  return B + glm::normalize(BA) * w;
}



}
}



#endif /* vector_functions_cpp */
