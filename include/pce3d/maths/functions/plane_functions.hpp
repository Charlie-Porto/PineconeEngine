#ifndef plane_functions_hpp
#define plane_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for planes
-----------------------------------------------------------------*/
#include <iostream>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "../objects/PlaneCartesianForm.hpp"
#include "vector_functions.hpp"

namespace pce3d {
namespace maths {

using plane = PlaneCartesianForm;

plane calculatePlaneGiven3Points(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

bool checkIfPointInPlane(const glm::dvec3& A, const glm::dvec3& B,
                         const glm::dvec3& C, const glm::dvec3& testing_point);

double calculateDistanceBetweenPointAndPlane(const plane& mplane, const glm::dvec3& point);

glm::dvec3 calculateNormalVectorInDirectionOfPoint(const glm::dvec3& A, const glm::dvec3& B,
                                                   const glm::dvec3& C, const glm::dvec3& point);

glm::dvec3 calculateClosestPointInPlaneToPoint(const glm::dvec3& A, const glm::dvec3& B,
                                               const glm::dvec3& C, const glm::dvec3& point);
}
}




#endif /* plane_functions_hpp */
