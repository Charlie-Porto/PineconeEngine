#ifndef plane_functions_cpp
#define plane_functions_cpp

#include "../plane_functions.hpp"

namespace pce3d {
namespace maths {


plane calculatePlaneGiven3Points(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C) {
  const glm::dvec3 X = glm::dvec3(A - C);
  const glm::dvec3 Y = glm::dvec3(B - C);
  const glm::dvec3 normal_vec = glm::cross(X, Y);
  const double plane_d_value = normal_vec.x * -A.x + normal_vec.y * -A.y + normal_vec.z * -A.z;

  return plane {
    .a = normal_vec.x,
    .b = normal_vec.y,
    .c = normal_vec.z,
    .d = plane_d_value 
  };
}


bool checkIfPointInPlane(const glm::dvec3& A, const glm::dvec3& B,
                         const glm::dvec3& C, const glm::dvec3& testing_point) {

  plane mplane = calculatePlaneGiven3Points(A, B, C);

  double plane_reduced = mplane.a * testing_point.x + mplane.b * testing_point.y + mplane.c * testing_point.z + mplane.d;

  if (abs(plane_reduced) < .0001) {
    return true;
  }
  return false;
}


double calculateDistanceBetweenPointAndPlane(const plane& mplane, const glm::dvec3& point) {
  return ((mplane.a * point.x + mplane.b * point.y + mplane.c * point.z + mplane.d)
         / sqrt(pow(mplane.a, 2.0) + pow(mplane.b, 2.0) + pow(mplane.c, 2.0)));
}


glm::dvec3 calculateNormalVectorInDirectionOfPoint(const glm::dvec3& A, const glm::dvec3& B,
                                                   const glm::dvec3& C, const glm::dvec3& point) {
  const glm::dvec3 X = glm::dvec3(A - C);
  const glm::dvec3 Y = glm::dvec3(B - C);
  const glm::dvec3 normal_vec = glm::normalize(glm::cross(X, Y));
  const glm::dvec3 alt_normal_vec = -normal_vec;

  const double distance = pce3d::maths::calculateDistanceBetweenVectors(A + normal_vec, point);
  const double alt_distance = pce3d::maths::calculateDistanceBetweenVectors(A + alt_normal_vec, point);

  return distance < alt_distance ? normal_vec : alt_normal_vec;
}


glm::dvec3 calculateClosestPointInPlaneToPoint(const glm::dvec3& A, const glm::dvec3& B,
                                               const glm::dvec3& C, const glm::dvec3& point) {
  const plane mplane = calculatePlaneGiven3Points(A, B, C);
  const double distance = abs(calculateDistanceBetweenPointAndPlane(mplane, point));
  const glm::dvec3 plane_normal_direction_vector = calculateNormalVectorInDirectionOfPoint(A, B, C, point);
  return glm::dvec3(point + (-plane_normal_direction_vector * distance));
}

}
}

#endif /* plane_functions_cpp */
