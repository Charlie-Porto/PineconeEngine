#ifndef plane_functions_hpp
#define plane_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for planes
-----------------------------------------------------------------*/
#include <glm/vec3.hpp>
#include "../objects/PlaneCartesianForm.hpp"

namespace pce3d {
namespace maths {

using plane = PlaneCartesianForm;

plane calculatePlaneGiven3Points(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C);

bool checkIfPointInPlane(const glm::dvec3& A, const glm::dvec3& B,
                         const glm::dvec3& C, const glm::dvec3& testing_point);

}
}




#endif /* plane_functions_hpp */
