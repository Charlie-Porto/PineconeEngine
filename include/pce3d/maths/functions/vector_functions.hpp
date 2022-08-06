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

}
}





#endif /* vector_functions_hpp */
