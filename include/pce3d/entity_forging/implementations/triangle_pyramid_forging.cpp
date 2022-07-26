#ifndef triangle_pyramid_forging_cpp
#define triangle_pyramid_forging_cpp

#include "../triangle_pyramid_forging.hpp"



namespace pce3d {
namespace forge {



Entity forgeTrianglePyramidEntity(const double h, const double base_side_length, 
                                  const glm::dvec3& center, const glm::dquat& local_rotation,
                                  const std::vector<int>& color) {
  const double base_height = base_side_length * sin(60.0 * PI / 180.0);
  VertexMap vertices = {
    {1, glm::dvec3(center.x, center.y - h/2.0, center.z - base_height/2.0)},
    {2, glm::dvec3(center.x - base_side_length/2.0, center.y - h/2.0, center.z + base_height/2.0)},
    {3, glm::dvec3(center.x + base_side_length/2.0, center.y - h/2.0, center.z + base_height/2.0)},
    {4, glm::dvec3(center.x, center.y + h/2.0, center.z)}
  };

  EdgeMap edge_map = {
    std::make_pair(1, 2),
    std::make_pair(2, 3),
    std::make_pair(3, 1),
    std::make_pair(1, 4),
    std::make_pair(2, 4),
    std::make_pair(3, 4)
  };
  
  FaceVertexMap face_vertex_map = {
    {1, {1, 4, 2}},
    {2, {3, 2, 1}},
    {3, {3, 4, 2}},
    {4, {1, 4, 3}},
  };

  Entity new_entity = control.CreateEntity();
  control.AddComponent(new_entity, pce::Position{.actual_center_of_mass = center});
  control.AddComponent(new_entity, pce::LocalRotation{.versor = local_rotation});
  control.AddComponent(new_entity, pce::Surface{.color = color});
  control.AddComponent(new_entity, pce::FaceShade{});
  control.AddComponent(new_entity, pce::RigidObject{
    .radius = 0,
    .vertices = vertices,
    .edges = edge_map,
    .face_vertex_map = face_vertex_map
  });

  return new_entity;

}


}
}





#endif /* triangle_pyramid_forging_cpp */
