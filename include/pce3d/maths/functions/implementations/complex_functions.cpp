#ifndef complex_functions_cpp
#define complex_functions_cpp

#include "../complex_functions.hpp"


namespace pce3d {
namespace maths {

std::complex<double> convertVec2ToComplexNumber(const glm::dvec2& v)
{
  const std::complex<double> c = std::complex<double>(v.x, v.y);
  return c;
}



glm::dvec2 convertComplexNumberToVec2(const std::complex<double>& c)
{
  const glm::dvec2 v = glm::dvec2(c.real(), c.imag());
  return v;
}




std::complex<double> calculateRotationComplexNumGivenVec2s(
    const glm::dvec2& A
  , const glm::dvec2& B
)
{
  const std::complex<double> A_complex = convertVec2ToComplexNumber(A);
  const std::complex<double> B_complex = convertVec2ToComplexNumber(B);

  // A * x = B so x = B/A
  const std::complex<double> x_complex = B_complex / A_complex;
  return x_complex;
}



glm::dvec2 rotateVec2ByComplexNumber(
    const glm::dvec2& v
  , const std::complex<double> c
)
{
  const std::complex<double> v_complex = convertVec2ToComplexNumber(v);
  const std::complex<double> rotated_x_complex = v_complex * c;
  const glm::dvec2 rotated_x = convertComplexNumberToVec2(rotated_x_complex);
  return rotated_x;
}



}
}




#endif /* complex_functions_cpp */
