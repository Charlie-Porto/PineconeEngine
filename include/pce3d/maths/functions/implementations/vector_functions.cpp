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

inline double calcMagV2(const glm::dvec2& A)
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



inline glm::dvec3 findClosestPointOnVec3LineToVec3(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C)
{
  const glm::dvec3 BC = (C-B);
  const glm::dvec3 BA = (A-B);
  const double mag_BC = calcMagV3(BC);
  const double angle = calculateAngleDegreesBetweenVectors(BC, BA);
  const double w = cos(angle / 180.0 * PI) * mag_BC;
  
  return B + glm::normalize(BA) * w;
}



inline double calculateDistanceBetweenPointAndLine(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C)
{
  const glm::dvec3 closest_point = findClosestPointOnVec3LineToVec3(A, B, C);
  return calculateDistanceBetweenVectors(closest_point, C);
}



bool determineIfLineVectorsWithinDistance(
  const glm::dvec3& A, const glm::dvec3& B, 
  const glm::dvec3& C, const glm::dvec3& D,
  const double distance_threshold
)
{
  const glm::dvec3 AB = B - A; 
  const glm::dvec3 CD = D - C;

  const glm::dvec3 orthogonal_line = glm::normalize(glm::cross(AB, CD));
  const glm::dvec3 CA = A - C;
  const double orth_line_magnitude = calcMagV3(orthogonal_line);
  const double numerator = calcMagV3(glm::cross(orthogonal_line, CA));
  const double distance = numerator / orth_line_magnitude;

  if (distance < distance_threshold)
  {
    return true;
  }

  return false;
}



std::pair<double, glm::dvec3> estimateDistanceAndMidPointOfClosestPointsOnLines(
  const glm::dvec3& A, const glm::dvec3& B, 
  const glm::dvec3& C, const glm::dvec3& D
)
{
  // const double ab_length = calculateDistanceBetweenVectors(A, B);
  // const double cd_length = calculateDistanceBetweenVectors(C, D);
  // const double a_rope_length = calculateDistanceBetweenVectors(A, C) 
  //                            + calculateDistanceBetweenVectors(A, D);
  // const double b_rope_length = calculateDistanceBetweenVectors(B, C) 
  //                            + calculateDistanceBetweenVectors(B, D);
  // const double c_rope_length = calculateDistanceBetweenVectors(C, B) 
  //                            + calculateDistanceBetweenVectors(C, A);
  // const double d_rope_length = calculateDistanceBetweenVectors(D, B) 
  //                            + calculateDistanceBetweenVectors(D, A);
  // const double ab_rope_length = a_rope_length + b_rope_length;  
  // const double cd_rope_length = c_rope_length + d_rope_length;  

  // const double a_rope_percentage = a_rope_length / ab_rope_length;
  // const double c_rope_percentage = c_rope_length / cd_rope_length;

  // const glm::dvec3 ab_closest_point = A + ((B - A) * a_rope_percentage);
  // const glm::dvec3 cd_closest_point = C + ((D - C) * c_rope_percentage);
  
  // const glm::dvec3 midpoint = glm::dvec3((ab_closest_point + cd_closest_point) / 2.0);
  // const double distance = calculateDistanceBetweenVectors(ab_closest_point, cd_closest_point);

  // return std::make_pair(distance, midpoint);

  /* attempt #2 */
  const glm::dvec3 AB_c_point = findClosestPointOnVec3LineToVec3(A, B, C);
  const glm::dvec3 AB_d_point = findClosestPointOnVec3LineToVec3(A, B, D);
  const double AB_c_point_weight = calcMagV3(AB_c_point - C);
  const double AB_d_point_weight = calcMagV3(AB_d_point - D);
  const double total_weight = AB_c_point_weight + AB_d_point_weight;
  const glm::dvec3 normed_AB_c_point = AB_c_point - AB_d_point;

  const glm::dvec3 scaled_normed_AB_d_point = -normed_AB_c_point / AB_d_point_weight;
  const glm::dvec3 scaled_normed_AB_c_point = normed_AB_c_point 
                                            + normed_AB_c_point / AB_d_point_weight;
  const glm::dvec3 scaled_normed_AB_midpoint = (scaled_normed_AB_c_point + scaled_normed_AB_d_point) / 2.0;
  const glm::dvec3 scaled_AB_midpoint = scaled_normed_AB_midpoint + AB_d_point;

  const glm::dvec3 scaled_CD_midpoint = findClosestPointOnVec3LineToVec3(C, D, scaled_AB_midpoint);
  const glm::dvec3 re_scaled_AB_midpoint = findClosestPointOnVec3LineToVec3(A, B, scaled_CD_midpoint);

  const glm::dvec3 midpoint = (re_scaled_AB_midpoint + scaled_CD_midpoint) / 2.0;
  const double distance = calcMagV3(re_scaled_AB_midpoint - scaled_CD_midpoint);
  
  // if (distance < 5.0)
  // {
  // const std::vector<int> red = {255, 255, 255, 255};
  // const std::vector<int> green = {255, 25, 55, 255};
  // dev_render_system.AddUnRotatedPointToPointColorMap(scaled_AB_midpoint, red, 10.0);
  // dev_render_system.AddUnRotatedPointToPointColorMap(scaled_CD_midpoint, green, 10.0);
  // }

  return std::make_pair(distance, midpoint);
}



}
}



#endif /* vector_functions_cpp */
