#ifndef organizeRigidObjectFaces_cpp
#define organizeRigidObjectFaces_cpp

#include <iostream>
#include "../organizeRigidObjectFaces.hpp"
#include "../../../maths/objects/PlaneCartesianForm.hpp"
#include "../../../maths/functions/plane_functions.hpp"

namespace pce3d {

using Plane = pce3d::maths::PlaneCartesianForm;


VertexEdgeMap defineVertexEdgeMap(const VertexMap& vertex_map, const EdgeMap& edge_map) {
  /*
  creates a map mapping each vertex to the edges it participates in
  */
  VertexEdgeMap vertex_edge_map;
  for (auto const& [key, vertex] : vertex_map) {
    vertex_edge_map[key] = {};
  }

  for (auto const& [key, edge] : edge_map) {
    vertex_edge_map[edge.first].push_back(key);
    vertex_edge_map[edge.second].push_back(key);
  }

  /* TEMPORARY: print */
  // for (auto const& [key, edges] : vertex_edge_map) {
    // for (auto const& edge : edges) {
      // std::cout << key << ": " << edge << '\n';
    // }
  // }

  return vertex_edge_map;
}


void groupRigidBodyEdgesIntoFaces(const VertexMap& vertex_map, 
                                  const EdgeMap& edge_map, FaceMap& face_map) {

  /* create participation_map */
  std::unordered_map<uint32_t, int> edge_participation_map;
  for (auto const& [key, edge] : edge_map) {
    edge_participation_map[key] = 0;
  }
  
  VertexEdgeMap vertex_edge_map = defineVertexEdgeMap(vertex_map, edge_map);

  /*----------------- LOOP 1: loop over each edge --------------- */
  int count = -1;
  for (auto const& [key, edge] : edge_map) {
    /* TEMPORARY if statment below is for deving only */
    ++count;
    if (count > 0) { break; }

    if (edge_participation_map.at(key) == 2) { continue; }

    /* iterate through edges connected to the tail vertex of the selected edge */
    const size_t connected_edges = vertex_edge_map.at(edge.second).size();
    std::cout << "count of edges connected to main edge tail: " << connected_edges << '\n';
    size_t i = 0; 
    size_t count = 0;
    auto mplane = Plane{.a=0.0, .b=0.0, .c=0.0, .d=0.0};

    /*--------------- LOOP 2: loop over each edge connected to the primary edge ---------------*/
    /* FOR EACH CONNECTED EDGE (SEPARATE FACES BEING PRODUCED) */
    /* TO START: JUST DRAW 1 OR TWO FACES */
    // while (i < connected_edges) {
    while (count < 1) {

      /* make sure we aren't comparing the edge tail to the head vertex */
      if (key == vertex_edge_map.at(edge.second)[i]) { ++i; continue; }
      
      /* get crawl edge and crawl vertex */
      uint32_t crawl_edge = vertex_edge_map.at(edge.second)[i];
      std::pair<uint32_t, uint32_t> crawl_edge_vertices = edge_map.at(crawl_edge);
      uint32_t crawl_vertex = crawl_edge_vertices.first;
      if (crawl_vertex == edge.second) { crawl_vertex = crawl_edge_vertices.second; }

      std::vector<std::pair<uint32_t, std::pair<uint32_t, uint32_t>>> face_edges = {
        {key, edge},
        {crawl_edge, edge_map.at(crawl_edge)}
      };

      /* TODO: establish the face's plane */
      mplane = pce3d::maths::calculatePlaneGiven3Points(
          vertex_map.at(edge.first), vertex_map.at(edge.second), vertex_map.at(crawl_vertex));
      
      std::cout<< "mplane: " << mplane.a << ", " << mplane.b << ", " << mplane.c << ", " << mplane.d<< '\n';

      /*--------------- LOOP 3: loop through next edges until arrive back at primary edge ---------------*/
      uint32_t previous_vertex = crawl_vertex;
      while (previous_vertex != edge.second) {

        /* pick next edge (one in the same plane as both those already chosen) */
        uint32_t next_edge;
        std::vector<uint32_t> sub_edges = vertex_edge_map.at(previous_vertex);
          
      }

      ++i; 
      ++count;
    }
  }

}

}



#endif /* organizeRigidObjectFaces_cpp */
