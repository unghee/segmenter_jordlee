/**
 * @file Segment.cc
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Gestalt class segment.
 **/

#include <algorithm>
#include <cstdio>
#include "Draw.hh"
#include "VisionCore.hh"
#include "Segment.hh"
#include "FormSegments.hh"

namespace Z
{

/**
 * @brief Construct a segment with a given number of edgels.
 * @param vc Vision core
 */
Segment::Segment(VisionCore *vc, const Array<Edgel> &arr, int _b_id) : Gestalt(vc, SEGMENT), edgels(arr.Size())
{
  edgels = arr;
  b_id = _b_id;
  CalculateSignificance();
}

/**
 * @brief Draw the segment.
 * @param detail The detail has no effect here.
 */
void Segment::Draw(int detail)
{
  for(unsigned i = 0; i < edgels.Size(); i++) {
    int g=255-(i/10.);
    if(g<0) g=0;
    DrawPoint2D(edgels[i].p.x, edgels[i].p.y, /*RGBColor::green*/RGBColor(255,255,g));
  }
}

/**
 * @brief Get info as string.
 * @return Returns array of characters.
 */
const char* Segment::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%sedgels: %d\n", Gestalt::GetInfo(),
      edgels.Size());
  return info_text;
}

/**
 * @brief Returns true if any part of the Gestalt is at pixel position (x,y).
 * @return Returns true if any part of the Gestalt is at pixel position (x,y).
 */
bool Segment::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = 0; i < edgels.Size(); i++)
    if(IsEqual(xd, edgels[i].p.x) && IsEqual(yd, edgels[i].p.y))
      return true;
  return false;
}

/**
 * @brief Calculate significance\n
 * The significance is proportional to the edgel size.
 */
void Segment::CalculateSignificance()
{
  /*if(edgels.Size() <= 1)
    sig = 0.;
  else
  {
    // gap missing for closed contour
    int g = (int)floor(Distance(edgels[0].p, edgels[edgels.Size()-1].p) + 0.5);
    sig = Significance(0, edgels.Size(), edgels.Size() + g, VisionCore::p_e);
  }*/
  //!!! HACK
  sig = edgels.Size();
}

/**
 * @brief Returns the normalised tangent direction at the specified pixel.\n
 * Range of points over which tangent is estimated is adapted automatically.\n
 * Starting with points i-1 to i+1, extends range until line through first and\n
 * last point no longer fits the points between.\n
 * Note: direction points "forward" in terms of counting pixels on the edge.\n
 * @param i Tangent on this place of the array.
 * @param l TODO
 * @param u TODO
 */
Vector2 Segment::Tangent(int i, int l, int u)
{
  const double MAX_DIST = 1.; // TODO: nasty threshold
  if(u == (int)UNDEF_ID)
    u = edgels.Size() - 1;
  int j, start = max(l, i - 1);
  int end = min(i + 1, u);
  bool on_line = true;
  Vector2 d, d_out;
  d_out = Normalise(edgels[end].p - edgels[start].p);
  while(on_line && (start >= l && end <= u))
  {
    d = Normalise(edgels[end].p - edgels[start].p);
    for(j = start + 1; j <= end - 1 && on_line; j++)
      if(AbsDistPointToLine(edgels[j].p, edgels[start].p, d) > MAX_DIST)
        on_line = false;
    if(on_line)
    {
      d_out = d;
      start--;
      end++;
    }
  }
  return d_out;
}

}

