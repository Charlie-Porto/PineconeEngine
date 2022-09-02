#ifndef shadeFunctions_cpp
#define shadeFunctions_cpp

#include "../shadeFunctions.hpp"

namespace pce3d {
namespace shade {

const double PI = 3.14159265;

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec) {
/* returns a double between 0 and 1 */
/* 0 = black */
/* 1 = pure natural color */

  double angle_light_hits_face = acos(glm::dot(light_direction, plane_normal_vec) 
                                    / (sqrt(glm::dot(light_direction, light_direction))
                                       * sqrt(glm::dot(plane_normal_vec, plane_normal_vec))));
  
  // std::cout << "angle light hits face: " << (angle_light_hits_face / PI * 180.0) << '\n';

  return (angle_light_hits_face/PI);
}



void mapSpherePixelsToBrightnessZones(
    pce::PixelShadeMap& pixel_shade_map
  , const PixelMap& outline_pixels
  , const SurfaceZoneBrightnessMap& camera_transformed_surface_zone_brightness_map
  , const glm::dvec3& light_direction
  , const glm::dvec3& sphere_center
  , const double sphere_radius
  , const double zone_granularity
)
{
  for (auto const& [A, B] : outline_pixels) 
  {
    for (double i_x = A.x; i_x <= B.x; ++i_x) 
    {
      const glm::dvec2 pixel = glm::dvec2(i_x, A.y);

      const glm::dvec3 viewsphere_point = glm::normalize(
        radar::convertPixelToPointOnViewSphere(pixel / pce3d::Core3D::ORDINARY_ZOOM_INDEX_)
      );

      glm::dvec3 entity_sphere_point;

      try 
      {
        entity_sphere_point = pce::maths::calculateClosestPointVectorIntersectsSphere(
                                                  glm::dvec3(0, 0, 0),
                                                  viewsphere_point,
                                                  sphere_center,
                                                  sphere_radius);
      } catch (double discriminant) {}

      const glm::dvec3 rounded_entity_sphere_point = pce3d::round::roundVec3ComponentsToNearestInterval(
        zone_granularity, entity_sphere_point
      );

      if (camera_transformed_surface_zone_brightness_map.find(rounded_entity_sphere_point)
       != camera_transformed_surface_zone_brightness_map.end())
      {
        pixel_shade_map[pixel] = camera_transformed_surface_zone_brightness_map.at(rounded_entity_sphere_point);
      }
      else
      {
        // std::cout << "NOT FOUND IN MAP" << '\n';
        pixel_shade_map[pixel] = 1.0;
      }
      // std::cout << "pixel: " << p.x << ", " << p.y << ", " << "shading: " << pixel_shades.at(p) << '\n';
    }
  }
}



void calculateFaceBrightnessForSpherePixels(const glm::dvec3& light_direction,
                                            const glm::dvec3& sphere_center,
                                            const double sphere_radius,
                                            const PixelMap& outline_pixels,
                                            pce::PixelShadeMap& pixel_shades) {
  pixel_shades.clear();
  for (auto const& [A, B] : outline_pixels) {
    for (double i_x = A.x; i_x <= B.x; ++i_x) {
      const glm::dvec2 p = glm::dvec2(i_x, A.y);

      const glm::dvec3 viewsphere_point = glm::normalize(radar::convertPixelToPointOnViewSphere(p / pce3d::Core3D::ORDINARY_ZOOM_INDEX_));
      glm::dvec3 entity_sphere_point;

      try {
        entity_sphere_point = pce::maths::calculateClosestPointVectorIntersectsSphere(
                                                  glm::dvec3(0, 0, 0),
                                                  viewsphere_point,
                                                  sphere_center,
                                                  sphere_radius);
      } catch (double discriminant) {}
      const glm::dvec3 normal_vect = entity_sphere_point - sphere_center;  
      pixel_shades[p] = calculateFaceBrightness(light_direction, normal_vect);
      // std::cout << "pixel: " << p.x << ", " << p.y << ", " << "shading: " << pixel_shades.at(p) << '\n';
    }
  }
}



std::vector<glm::dvec3> getSurfaceZonesAtSpherePoint(
    const glm::dvec3& surface_point
  , const glm::dvec3& sphere_center
  , const double radius
  , const double zone_granularity_constant
)
{
  std::vector<glm::dvec3> zones{};
  int y = int(radius / zone_granularity_constant);

  std::vector<glm::dvec3> points{};
  points.push_back(glm::dvec3(sphere_center.x, surface_point.y, sphere_center.z+y));
  points.push_back(glm::dvec3(sphere_center.x, surface_point.y, sphere_center.z+y));
  points.push_back(glm::dvec3(sphere_center.x, surface_point.y, sphere_center.z-y));
  points.push_back(glm::dvec3(sphere_center.x, surface_point.y, sphere_center.z-y));
  points.push_back(glm::dvec3(sphere_center.x+y, surface_point.y, sphere_center.z));
  points.push_back(glm::dvec3(sphere_center.x-y, surface_point.y, sphere_center.z));
  points.push_back(glm::dvec3(sphere_center.x+y, surface_point.y, sphere_center.z));
  points.push_back(glm::dvec3(sphere_center.x-y, surface_point.y, sphere_center.z));
  
  int x = 0;
  y = int(radius);
  // std::cout << "rounded circle radius: " << y << '\n';
  int d = 3 - 2 * radius;
  while (y >= x) {
    x++;
    if (d > 0) {
      y--;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    std::vector<glm::dvec3> new_points = {
      glm::dvec3(sphere_center.x+x, surface_point.y, sphere_center.z+y),
      glm::dvec3(sphere_center.x-x, surface_point.y, sphere_center.z+y),
      glm::dvec3(sphere_center.x+x, surface_point.y, sphere_center.z-y),
      glm::dvec3(sphere_center.x-x, surface_point.y, sphere_center.z-y),
      glm::dvec3(sphere_center.x+y, surface_point.y, sphere_center.z+x),
      glm::dvec3(sphere_center.x-y, surface_point.y, sphere_center.z+x),
      glm::dvec3(sphere_center.x+y, surface_point.y, sphere_center.z-x),
      glm::dvec3(sphere_center.x-y, surface_point.y, sphere_center.z-x)
    };
    points.insert(points.end(), new_points.begin(), new_points.end());
  }

  for (auto& point : points) 
  {
    point = point * zone_granularity_constant;
    // std::cout << "point: "
              // << point.x << ", "
              // << point.y << ", "
              // << point.z << '\n';
    // dev_render_system.AddPointToPointColorMap(point, {255, 0, 200, 255}, 1.0);
  }

  return points;
}



void doSphereLightScan(
    const glm::dvec3& sphere_center
  , const double radius
  , SurfaceZoneBrightnessMap& surface_zone_brightness_map 
  , const glm::dvec3& light_direction
  , const double zone_granularity
)
{
  double current_angle = -90.0;
  double angle_increment = asin((zone_granularity / 2.0) / radius) / PI * 180.0;
  std::cout << "angle_increment: " << angle_increment << '\n';
  glm::dvec3 current_surface_point = glm::dvec3(0, cos(current_angle), sin(current_angle)) * radius;
  
  while (current_angle <= 90.0)
  {
    const double circle_radius = abs(cos(current_angle) * radius);
    std::cout << "current angle: "  << current_angle << '\n';
    // std::cout << "original_radius: " << radius << '\n';
    // std::cout << "circle_radius: " << circle_radius << '\n';
    current_surface_point = glm::normalize(glm::dvec3(0, sin(current_angle), cos(current_angle))) * radius;
    auto points = pce3d::shade::getSurfaceZonesAtSpherePoint(
      sphere_center + current_surface_point,
      sphere_center,
      circle_radius,
      1.0
    );
    current_angle += angle_increment;
    for (auto const& point : points)
    {
      const glm::dvec3 normal_vect = glm::normalize(point - sphere_center);
      surface_zone_brightness_map[point] = calculateFaceBrightness(light_direction, normal_vect);
    }
  }
}


}
}


#endif /* shadeFunctions_cpp */
