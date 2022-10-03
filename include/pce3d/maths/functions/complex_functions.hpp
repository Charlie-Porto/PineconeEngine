#ifndef complex_functions_hpp
#define complex_functions_hpp

#include <glm/vec2.hpp>
#include <cmath>
#include <complex>

namespace pce3d {
namespace maths {

std::complex<double> convertVec2ToComplexNumber(const glm::dvec2& v);

glm::dvec2 convertComplexNumberToVec2(const std::complex<double>& c);

std::complex<double> calculateRotationComplexNumGivenVec2s(
    const glm::dvec2& A
  , const glm::dvec2& B
);

glm::dvec2 rotateVec2ByComplexNumber(
    const glm::dvec2& v
  , const std::complex<double> c
);


}
}




#endif /* complex_functions_hpp */
