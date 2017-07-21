/**
 * $Id: TJunction.cc,v 1.10 2007/03/25 21:35:57 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "LJunction.hh" 
#include "TJunction.hh"
#include <cstdio>

namespace Z
{

/**
 * TODO: add setting of coll to constructor rather than outside in FormJunctions
 */
TJunction::TJunction(VisionCore *c, Line *pole, Line *left, Line *right,
    int end_p, int end_l, int end_r,
    Collinearity *co, LJunction *ljct_l, LJunction *ljct_r,
    double g, const Vector2 &inter)
  : Gestalt(c, T_JUNCTION)
{
  line[LEFT] = left;
  line[RIGHT] = right;
  line[POLE] = pole;
  lend[LEFT][INNER] = end_l;
  lend[RIGHT][INNER] = end_r;
  lend[POLE][INNER] = end_p;
  for(int i = 0; i < 3; i++)
    lend[i][OUTER] = Other(lend[i][INNER]);
  coll = co;
  ljct[LEFT] = ljct_l;
  ljct[RIGHT] = ljct_r;
  isct = inter;
  dir = (lend[POLE][START] == START ? line[POLE]->dir : -line[POLE]->dir);
  gap = max(g, 1.);
  /* this would take the size of the collinear gap into account:
  double g1 = Distance(pole->point[end_p], left->point[end_l]);
  double g2 = Distance(pole->point[end_p], right->point[end_r]); 
  gap = max(1., max(g1, g2));*/

  pole->t_jct[end_p] = this;
  left->AddPassiveTJunction(end_l, LEFT, this);
  right->AddPassiveTJunction(end_r, RIGHT, this);

  // collinearities at pole end can now reduce their apparent gap
  for(unsigned i = 0; i < pole->coll[end_p].Size(); i++)
    pole->coll[end_p][i]->CorrectGap();

  CalculateSignificance();
}

void TJunction::Recalc()
{
  // nothing to recalculate right now
  // TODO: check this properly
}

void TJunction::Draw(int detail)
{
  if(detail >= 1)
    for(int i = 0; i < 3; i++)
      line[i]->Draw(detail - 1);
  if(detail >= 2)
  {
    char id_str[20];
    Vector2 v = isct - 10.*dir;
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, v.x, v.y, RGBColor::blue);
  }
  for(int i = 0; i < 3; i++)
    DrawLine2D(isct.x, isct.y,
      line[i]->point[lend[i][INNER]].x,
      line[i]->point[lend[i][INNER]].y,
      RGBColor::magenta);
  DrawPoint2D(isct.x, isct.y, RGBColor::blue);
}

const char* TJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%spole: %u left: %u right: %d\ngap: %f\n"
      "coll: %u jct_l %u jct_r: %u\n",
      Gestalt::GetInfo(), line[POLE]->ID(), line[LEFT]->ID(),
      line[RIGHT]->ID(), gap, coll->ID(), ljct[LEFT]->ID(), ljct[RIGHT]->ID());
  return info_text;
}

bool TJunction::IsAtPosition(int x, int y)
{
  for(int i = 0; i < 3; i++)
    if(line[i]->IsAtPosition(x, y))
      return true;
  return false;
}

/**
 * Calculate accidentalness and significance.
 * Assume edgels are distributed following a Poisson process with parameter
 * p_e in space R2 (p_e = prob. of edgels). Note that this is strictly speaking
 * not true: edgels are not independent. But for now this crude approximaation
 * suffices.
 * Be r the distance between endpoint of pole and bar.
 * Calculate the probability that there are one or more edgels in area A =
 * pi*r^2/2. Which is the same as 1 - probability that there is no endpoint in
 * area A.
 * We chose a half circle because we are only looking for points "in front of"
 * the pole.
 */
void TJunction::CalculateSignificance()
{
  acc = 1. - exp(-core->p_e*M_PI*gap*gap/2.);
  sig = -log(acc);
}

int TJunction::WhichLineIs(Line *l) throw(std::runtime_error)
{
  for(int i = LEFT; i <= POLE; i++)
    if(l == line[i])
      return i;
  char buffer [100];
  sprintf(buffer, "TJunction::WhichLineIs: line %u is not part of T-jct %u", l, id);
  throw std::runtime_error(buffer);
}

}

