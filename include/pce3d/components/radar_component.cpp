#ifndef radar_component_cpp
#define radar_component_cpp



namespace pce {

struct Radar {
  uint32_t closest_vertex_id;
  double closest_vertex_distance;
  uint32_t farthest_vertex_id;
  double farthest_vertex_distance;
};

}

#endif /* radar_component_cpp */
