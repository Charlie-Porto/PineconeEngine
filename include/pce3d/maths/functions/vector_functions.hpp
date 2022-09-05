#ifndef vector_functions_hpp
#define vector_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for vectors
-----------------------------------------------------------------*/

#include <cmath>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

namespace pce3d {
namespace maths {

template <typename T> double calculateDistanceBetweenVectors(const T& A, const T& B);

template <typename T> double calculateAngleDegreesBetweenVectors(const T& A, const T& B);

inline double calcMagV3(const glm::dvec3& A);

inline bool determineIfVec3MakesLineWithTwoVec3s(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

glm::dvec3 findClosestPointOnVec3LineToVec3(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

}
}





#endif /* vector_functions_hpp */
