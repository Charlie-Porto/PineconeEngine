#ifndef vector_functions_hpp
#define vector_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for vectors
-----------------------------------------------------------------*/

#include <utility>
#include <cmath>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "pce_psuedo_randomness.hpp"

namespace pce3d {
namespace maths {

template <typename T> double calculateDistanceBetweenVectors(const T& A, const T& B);

template <typename T> double calculateAngleDegreesBetweenVectors(const T& A, const T& B);

inline double calcMagV3(const glm::dvec3& A);

inline double calcMagV2(const glm::dvec2& A);

inline bool determineIfVec3MakesLineWithTwoVec3s(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

inline glm::dvec3 findClosestPointOnVec3LineToVec3(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

inline double calculateDistanceBetweenPointAndLine(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

bool determineIfLineVectorsWithinDistanceAndAtWhichPoint(
  const glm::dvec3& A, const glm::dvec3& B, 
  const glm::dvec3& C, const glm::dvec3& D,
  const double distance_threshold
);

std::pair<double, glm::dvec3> estimateDistanceAndMidPointOfClosestPointsOnLines(
  const glm::dvec3& A, const glm::dvec3& B, 
  const glm::dvec3& C, const glm::dvec3& D
);


}
}





#endif /* vector_functions_hpp */
