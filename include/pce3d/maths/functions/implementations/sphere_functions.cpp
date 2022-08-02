#ifndef sphere_functions_cpp
#define sphere_functions_cpp

#include "../sphere_functions.hpp"




namespace pce {
namespace maths {

glm::dvec3 calculateClosestPointVectorIntersectsSphere(
                            const glm::dvec3& v_origin,
                            const glm::dvec3& v_direction,
                            const glm::dvec3& sphere_center,
                            const double sphere_radius) {
  // std::cout << "-------" << '\n';
  // std::cout << "v_origin: " << v_origin.x << ", " << v_origin.y << ", " << v_origin.z << '\n';
  // std::cout << "v_direction: " << v_direction.x << ", " << v_direction.y << ", " << v_direction.z << '\n';
  // std::cout << "sphere center: " << sphere_center.x << ", " << sphere_center.y << ", " << sphere_center.z << '\n';
  // std::cout << "radius: " << sphere_radius << '\n';
  const glm::dvec3 o_less_c = v_origin - sphere_center;
  double a = dot(v_direction, v_direction);
  double b = 2.0 * dot(v_direction, (o_less_c));
  double c = dot(o_less_c, o_less_c) - pow(sphere_radius, 2.0);
  double discriminant = pow(b, 2.0) - 4 * a * c; 
  if (discriminant < 0) {
    throw (discriminant);
  }
  const double distance_a = (-b - sqrt(discriminant)) / (2 * a);
  const double distance_b = (-b + sqrt(discriminant)) / (2 * a);  

  const double closest_intersection_distance = std::min(distance_a, distance_b);
  const glm::dvec3 travel_vector = closest_intersection_distance * v_direction;
  const glm::dvec3 intersection_point = v_origin + travel_vector;
  
  // std::cout << "entity sphere intersection: " << intersection_point.x << ", " << intersection_point.y << ", " << intersection_point.z << '\n';
  return intersection_point; 
}
 

}
}


#endif /* sphere_functions_cpp */
