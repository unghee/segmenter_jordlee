/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file Graph.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from realtions.
 */

#include "Graph.h"

#ifndef GC_DEBUG
#define GC_DEBUG false
#endif

namespace gc
{

/**
 * @brief Constructor of Graph
 */
Graph::Graph(unsigned nrNodes, std::vector<surface::Relation> &rel)
{
  nodes = nrNodes;
  relations = rel;
}


/**
 * @brief Destructor of GraphCut
 */
Graph::~Graph()
{}


/**
 * @brief Build graph from results of the SVM-Predictor.
 * @param e Vector of edges
 * @param num_edges Number of created edges.
 */
void Graph::BuildFromSVM(std::vector<gc::Edge> &e, unsigned &num_edges)
{
  if(GC_DEBUG) {
    printf("[Graph::BuildFromSVM] Number of nodes: %u\n", nodes);
    for(unsigned i=0; i<relations.size(); i++)
      printf("[Graph::BuildFromSVM] Relation %u: %u-%u\n", i, relations[i].id_0, relations[i].id_1);
  }
      
  // Connectivity check for graph
  for(unsigned i=1; i<nodes; i++) {
    bool node_found = false;
    for(unsigned j=0; j<relations.size(); j++)
      if(relations[j].id_0 == 0 && relations[j].id_1 == i)
        node_found = true;
    if(!node_found) {
#ifdef DEBUG
      printf("[Graph::BuildFromSVM] Warning: Node without relation: Add relation: %u-%u.\n", 0, i);
#endif
      surface::Relation r;
      r.id_0 = 0;
      r.id_1 = i;
      r.groundTruth = -1;
      r.type = 1;
      r.rel_probability.push_back(1.0);
      r.rel_probability.push_back(0.0);
      relations.push_back(r);
    }
  }
    
  if(GC_DEBUG) {
    for(unsigned i=0; i<relations.size(); i++)
      printf("[Graph::BuildFromSVM] Relation %u: %u-%u\n", i, relations[i].id_0, relations[i].id_1);
  }    

  for(unsigned i=0; i< relations.size(); i++) {
    gc::Edge e;
    e.a = relations[i].id_0;
    e.b = relations[i].id_1;
    e.type = 1;
    e.w = 1- relations[i].rel_probability[1];      // TODO ??? it's the weight for joining elements!!! (1-p(x))
    if(GC_DEBUG)
      printf("[Graph::BuildFromSVM] New edge (type: %i): %i-%i: %8.8f\n", e.type, e.a, e.b, e.w);
    edges.push_back(e);
  }
    
  num_edges = edges.size();
  e = edges;
  
  if(GC_DEBUG)
    printf("[Graph::BuildFromSVM] Created %lu edges from %lu relations\n", edges.size(), relations.size());

}

} 











