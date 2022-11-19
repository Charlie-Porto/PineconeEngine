#ifndef polyhedron_render_functions_cpp
#define polyhedron_render_functions_cpp

#include "../polyhedron_render_functions.hpp"

namespace pce3d {
namespace render {

const double RADIUS_SCALER = 200.0;

void renderTransparentCylinder(
    const glm::dvec3& A
  , const glm::dvec3& B
  , const double radius
  , const std::vector<int> color
)
{
  // calculate scaled radii
  const double a_dist = maths::calcMagV3(A);
  const double b_dist = maths::calcMagV3(B);
  const double a_scaled_radius = radius * RADIUS_SCALER / a_dist;
  const double b_scaled_radius = radius * RADIUS_SCALER / b_dist;

  // calculate face A foci line 
  const glm::dvec3 BA = A - B;
  const glm::dvec3 Acam = -A;
  const glm::dvec3 a_foci_line = glm::normalize(glm::cross(BA, Acam));

  // calculate angle with face A
  const double a_face_angle = std::min(maths::calculateAngleDegreesBetweenVectors(BA, Acam), maths::calculateAngleDegreesBetweenVectors(-BA, Acam));
  
  // calculate foci distance from center
  const double a_foci_dist = a_scaled_radius - a_scaled_radius * pow(((90.0 - a_face_angle) / 90.0), 3.0);

  // calculate foci positions and edges
  const glm::dvec3 a_a_focus = A + (a_foci_line * a_foci_dist);
  const glm::dvec3 a_b_focus = A - (a_foci_line * a_foci_dist);
  const glm::dvec3 a_a_edge = A + (a_foci_line * a_scaled_radius);
  const glm::dvec3 a_b_edge = A - (a_foci_line * a_scaled_radius);

  // calculate face B foci line 
  const glm::dvec3 AB = B - A;
  const glm::dvec3 Bcam = -B;
  const glm::dvec3 b_foci_line = glm::normalize(glm::cross(AB, Bcam));

  // calculate angle with face B
  const double b_face_angle = std::min(maths::calculateAngleDegreesBetweenVectors(AB, Bcam),maths::calculateAngleDegreesBetweenVectors(-AB, Bcam));
  std::cout << "a_actual_face_angle: " << maths::calculateAngleDegreesBetweenVectors(BA, Acam) << '\n';
  std::cout << "b_actual_face_angle: " << maths::calculateAngleDegreesBetweenVectors(AB, Bcam) << '\n';
  std::cout << "a_face_angle: " << a_face_angle << '\n';
  std::cout << "b_face_angle: " << b_face_angle << '\n';
  
  // calculate foci distance from center
  const double b_foci_dist = b_scaled_radius - b_scaled_radius * pow(((90.0 - b_face_angle) / 90.0), 3.0);

  // calculate foci positions and edges
  const glm::dvec3 b_a_focus = B + (b_foci_line * b_foci_dist);
  const glm::dvec3 b_b_focus = B - (b_foci_line * b_foci_dist);
  const glm::dvec3 b_a_edge = B + (b_foci_line * b_scaled_radius);
  const glm::dvec3 b_b_edge = B - (b_foci_line * b_scaled_radius);

  // get pixels corresponding to all relevant points
  const glm::dvec2 pa_a_focus = radar::convertPointOnViewSphereToPixel(glm::normalize(a_a_focus), false, true) * 15.0;
  const glm::dvec2 pa_b_focus = radar::convertPointOnViewSphereToPixel(glm::normalize(a_b_focus), false, true) * 15.0;
  const glm::dvec2 pa_a_edge = radar::convertPointOnViewSphereToPixel(glm::normalize(a_a_edge), false, true) * 15.0;
  const glm::dvec2 pa_b_edge = radar::convertPointOnViewSphereToPixel(glm::normalize(a_b_edge), false, true) * 15.0;
  const glm::dvec2 pb_a_focus = radar::convertPointOnViewSphereToPixel(glm::normalize(b_a_focus), false, true) * 15.0;
  const glm::dvec2 pb_b_focus = radar::convertPointOnViewSphereToPixel(glm::normalize(b_b_focus), false, true) * 15.0;
  const glm::dvec2 pb_a_edge = radar::convertPointOnViewSphereToPixel(glm::normalize(b_a_edge), false, true) * 15.0;
  const glm::dvec2 pb_b_edge = radar::convertPointOnViewSphereToPixel(glm::normalize(b_b_edge), false, true) * 15.0;

  const double a_radius = maths::calculateDistanceBetweenVectors(pa_a_edge, pa_b_edge)/2.0;
  const double b_radius = maths::calculateDistanceBetweenVectors(pb_a_edge, pb_b_edge)/2.0;

  pce::render::renderEllipse(pa_a_focus, pa_b_focus, a_radius, 30, color);

  // const glm::dvec2 A_conv = pce::convert::convertCartesianCoordinatesToSDL(pa_a_focus);
  // const glm::dvec2 B_conv = pce::convert::convertCartesianCoordinatesToSDL(pa_b_focus);
  // const glm::dvec2 Ae_conv = pce::convert::convertCartesianCoordinatesToSDL(pa_a_edge);
  // const glm::dvec2 Be_conv = pce::convert::convertCartesianCoordinatesToSDL(pa_b_edge);
  // const glm::dvec2 center_conv = pce::convert::convertCartesianCoordinatesToSDL((pa_a_focus + pa_b_focus)/2.0);

  // pce::render::renderCircle(A_conv.x, A_conv.y, 2, {255, 30, 150, 255});
  // pce::render::renderCircle(B_conv.x, B_conv.y, 2, {255, 30, 150, 255});
  // pce::render::renderCircle(Ae_conv.x, Ae_conv.y, 2, {255, 30, 0, 255});
  // pce::render::renderCircle(Be_conv.x, Be_conv.y, 2, {255, 30, 0, 255});
  // pce::render::renderCircle(center_conv.x, center_conv.y, 2, {0, 30, 150, 255});

  // const glm::dvec2 Ab_conv = pce::convert::convertCartesianCoordinatesToSDL(pb_a_focus);
  // const glm::dvec2 Bb_conv = pce::convert::convertCartesianCoordinatesToSDL(pb_b_focus);
  // const glm::dvec2 Abe_conv = pce::convert::convertCartesianCoordinatesToSDL(pb_a_edge);
  // const glm::dvec2 Bbe_conv = pce::convert::convertCartesianCoordinatesToSDL(pb_b_edge);
  // const glm::dvec2 center_b_conv = pce::convert::convertCartesianCoordinatesToSDL((pb_a_focus + pb_b_focus)/2.0);

  // pce::render::renderCircle(Ab_conv.x, Ab_conv.y, 2, {255, 30, 150, 255});
  // pce::render::renderCircle(Bb_conv.x, Bb_conv.y, 2, {255, 30, 150, 255});
  // pce::render::renderCircle(Abe_conv.x, Abe_conv.y, 2, {255, 30, 0, 255});
  // pce::render::renderCircle(Bbe_conv.x, Bbe_conv.y, 2, {255, 30, 0, 255});
  // pce::render::renderCircle(center_b_conv.x, center_b_conv.y, 2, {0, 30, 150, 255});


  pce::render::renderEllipse(pb_a_focus, pb_b_focus, b_radius, 30, color);
  pce::render::renderLine(pa_a_edge, pb_b_edge, color);
  pce::render::renderLine(pa_b_edge, pb_a_edge, color);
}

}
}


#endif /* polyhedron_render_functions_cpp */
