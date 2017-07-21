/**
 * @file Flap.hh
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt class for flaps.
 * Class for generalized flaps, built up from closures (polygons).
 */

#include <assert.h>
#include "Closure.hh"
#include "Flap.hh"
#include <cstdio>

namespace Z
{

/**
 * @brief Constructor of class Flap.
 * @param c Vision core
 * @param c0 Closure
 * @param c1 Closure
 */
Flap::Flap(VisionCore *vc, Closure *c0, Closure *c1) : Gestalt(vc, FLAP)
{
  assert(c0 != 0 && c1 != 0);
  clos[0] = c0;
  clos[1] = c1;
}

/**
 * @brief Draw Gestalt.
 * @param detail Degree of details.
 */
void Flap::Draw(int detail)
{
  clos[0]->Draw(detail);
  clos[1]->Draw(detail);
}


/**
 * @brief Get infos about the flap as text.
 */
const char* Flap::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%s  closure: %u %u\n",
      Gestalt::GetInfo(), clos[0]->ID(), clos[1]->ID());
  return info_text;
}


/**
 * @brief Check, if flap is at position x,y.
 * @param x x-Coordinate
 * @param y y-Coordinate
 */
bool Flap::IsAtPosition(int x, int y)
{
  return clos[0]->IsAtPosition(x, y) || clos[1]->IsAtPosition(x, y);
}

}

