/**
 * $Id: ParallelLineGroup.cc,v 1.7 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include "Line.hh"
#include "ParallelLineGroup.hh"
#include "FormSegments.hh"

namespace Z
{

ParallelLineGroup::ParallelLineGroup(const Array<unsigned> &ls, unsigned seed)
  : Gestalt(PARALLEL_LINE_GROUP)
{
  lines = ls;
  seed_line = seed;
  CalculateSignificance();
}

void ParallelLineGroup::Draw(int detail)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    Lines(lines[i])->Draw(detail);
  Lines(seed_line)->Draw(detail+1);
}

const char* ParallelLineGroup::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%slines: %u seed: %u\nphi: %f..%f dphi: %f\n",
      Gestalt::GetInfo(), lines.Size(), seed_line,
      ScaleAngle_0_pi(Lines(lines.First())->phi),
      ScaleAngle_0_pi(Lines(lines.Last())->phi),
      AngleBetweenLines(Lines(lines.Last())->phi, Lines(lines.First())->phi));
  return info_text;
}

bool ParallelLineGroup::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    if(Lines(lines[i])->IsAtPosition(x, y))
      return true;
  return false;
}

void ParallelLineGroup::CalculateSignificance()
{
//  int l = (int)sum_length;
//  int k = (int)sum_length;
//  sig = Significance(2, k, l, FormSegments::p_edgel);
  // -log[ (d_phi/pi)^k ]
  sig = -(double)lines.Size()*
    log(AngleBetweenLines(Lines(lines.Last())->phi, Lines(lines.First())->phi)/M_PI);
//  sig = -LogBinDist(NumLines(), lines.Size(),
//   AngleBetweenLines(Lines(lines.Last())->phi, Lines(lines.First())->phi)/M_PI);
}

}

