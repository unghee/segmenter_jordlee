/**
 * @file AJunction.cc
 * @author Zillich, Richtsfeld
 * @date 2009, May 2010
 * @version 0.1
 * @brief Gestalt class arc-junction.
 **/

#include <stdio.h>
#include <cstdio>
#include "AJunction.hh"

namespace Z
{

AJunction::AJunction(VisionCore *vc, Arc *arc_i, Arc *arc_j, int end_i, int end_j) : Gestalt(vc, A_JUNCTION)
{
  // if the END of arc i is conneced to the START of arc j, then this junction
  // (being in the middle) has i as START, j as END
  if(end_i == END)
  {
    arc[START] = arc_i;
    arc[END] = arc_j;
  }
  else
  {
    arc[START] = arc_j;
    arc[END] = arc_i;
  }
  // the START arc is connected at its END and vice versa
  for(int end = START; end <= END; end++)
    arc[end]->AddJunction(OtherEnd(end), this);
}

void AJunction::Draw(int detail)
{
  printf("AJunction::Draw: Draw arcs: %u-%u\n", arc[START]->ID(), arc[END]->ID());
  arc[START]->Draw(detail);
  arc[END]->Draw(detail);
}

void AJunction::DrawInfo()
{
}

const char* AJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%s  arcs @start: %u @end: %u",
      Gestalt::GetInfo(), arc[START]->ID(), arc[END]->ID());
  return info_text;
}

bool AJunction::IsAtPosition(int x, int y)
{
  return arc[START]->IsAtPosition(x, y) || arc[END]->IsAtPosition(x, y);
}

}

