/**
 * @file Collinearity.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief The vote image controls the extension of search lines.
 */

#include "Draw.hh"
#include "Line.hh"
#include "TJunction.hh"
#include "Collinearity.hh"
#include <cstdio>

namespace Z
{

/**
 * @brief Constructor of Gestalt class Collinearity
 * @param vc Vision core
 * @param line_i First line of collinearity
 * @param line_j Second line of collinearity
 * @param end_i Line end of first line
 * @param end_j Line end of second line
 */
Collinearity::Collinearity(VisionCore *vc, Line *line_i, Line *line_j, int end_i, int end_j)
  : Gestalt(vc, COLLINEARITY)
{
	
  // line 0 is to be the longer, line 1 the shorter line
  if(line_i->Length() >= line_j->Length())
  {
    line[0] = line_i;
    line[1] = line_j;
    near_point[0] = end_i;
    near_point[1] = end_j;
  }
  else
  {
    line[0] = line_j;
    line[1] = line_i;
    near_point[0] = end_j;
    near_point[1] = end_i;
  }
  line[0]->AddCollinearity(near_point[0], this);											// After recalculation is this value maybe false!
  line[1]->AddCollinearity(near_point[1], this);
	
	Recalc();
//   gap = max(1., Distance(line[0]->point[near_point[0]], line[1]->point[near_point[1]]));
//   vertex = (line[0]->point[near_point[0]] + line[1]->point[near_point[1]])/2.;
//   CorrectGap();
//   CalculateColors();
//   CalculateSignificance();
}

/**
 * @brief Recalculate the col-properties after line splitting.
 */
void Collinearity::Recalc()
{
  // line 0 is to be the longer, line 1 the shorter line
  if(line[0]->Length() < line[1]->Length())
  {
    Swap(line[0], line[1]);
    Swap(near_point[0], near_point[1]);
  }
  gap = max(1., Distance(line[0]->point[near_point[0]], line[1]->point[near_point[1]]));
  vertex = (line[0]->point[near_point[0]] + line[1]->point[near_point[1]])/2.;
  CorrectGap();
  CalculateColors();
  CalculateSignificance();
}

/**
 * @brief Draw Gestalt.
 * @param detail Degree of detail.
 */
void Collinearity::Draw(int detail)
{
  Vector2 p[2];
  if(detail >= 1)
    for(unsigned i = 0; i <= 1; i++)
      line[i]->Draw(detail - 1);
  if(detail >= 2)
  {
    char id_str[20];
    Vector2 v = vertex + Vector2(10., 10.);
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, v.x, v.y, RGBColor::dark_yellow);
  }
  for(unsigned i = 0; i <= 1; i++)
    p[i] = line[i]->point[near_point[i]];
	
  DrawLine2D(p[0].x, p[0].y, p[1].x, p[1].y);
  DrawPoint2D(vertex.x, vertex.y);
}

/**
 * @brief Draw some information about the Gestalt.
 */
void Collinearity::DrawInfo()
{
  char str[100];
  // longer line 0 defines what is left and right
  // if line 0 and 1 point the same way, left 0 = left, else left 0 = right
  unsigned left_1 = (near_point[0] != near_point[1] ? LEFT : RIGHT);
  RGBColor col[2][2];  // line 0/1 and left/right
  col[0][LEFT] = line[0]->MeanCol(LEFT);
  col[0][RIGHT] = line[0]->MeanCol(RIGHT);
  col[1][LEFT] = line[1]->MeanCol(left_1);
  col[1][RIGHT] = line[1]->MeanCol(Other(left_1));
    
  FillRect2D(0., 0., 0.5, 0.5, col[0][LEFT]);
  snprintf(str, 100, "0 left: %d %d %d", col[0][LEFT].r, col[0][LEFT].g,
      col[0][LEFT].b);
  DrawText2D(str, 0.05, 0.1, RGBColor::green);

  FillRect2D(0., 0.5, 0.5, 1., col[1][LEFT]);
  snprintf(str, 100, "1 left: %d %d %d", col[1][LEFT].r, col[1][LEFT].g,
      col[1][LEFT].b);
  DrawText2D(str, 0.05, 0.6, RGBColor::green);

  FillRect2D(0.5, 0., 1., 0.5, col[0][RIGHT]);
  snprintf(str, 100, "0 right: %d %d %d", col[0][RIGHT].r, col[0][RIGHT].g,
      col[0][RIGHT].b);
  DrawText2D(str, 0.55, 0.1, RGBColor::green);

  FillRect2D(0.5, 0.5, 1., 1., col[1][RIGHT]);
  snprintf(str, 100, "1 right: %d %d %d", col[1][RIGHT].r, col[1][RIGHT].g,
      col[1][RIGHT].b);
  DrawText2D(str, 0.55, 0.6, RGBColor::green);
}

/**
 * @brief Get information about the Gestalt as string.
 * @return Returns information about the Gestalt as string.
 */
const char* Collinearity::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%s  lines: %u %u\n  gap: %f (corrected %f)\n  col dist: %f",
      Gestalt::GetInfo(), line[0]->ID(), line[1]->ID(), gap, gap_cor, col_dist);
  return info_text;
}

/**
 * @brief Checks, if Gestalt is at this position.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if Gestalt is at this position.
 */
bool Collinearity::IsAtPosition(int x, int y)
{
  return line[0]->IsAtPosition(x, y) ||
    line[1]->IsAtPosition(x, y);
}

/**
 * @brief Calculate color difference.
 */
void Collinearity::CalculateColors()
{
  double dist[2];
  // longer line 0 defines what is left and right
  // if line 0 and 1 point the same way, left 0 = left, else left 0 = right
  unsigned left_1 = (near_point[0] != near_point[1] ? LEFT : RIGHT);
  dist[LEFT] = Dist(line[0]->MeanCol(LEFT), line[1]->MeanCol(left_1));
  dist[RIGHT] = Dist(line[0]->MeanCol(RIGHT), line[1]->MeanCol(Other(left_1)));
  // take the smaller of the two color distances
  // note: colors falling into the same bin could in the worst case still be
  // separated by bin diameter -> assume bin diameter as min distance
  col_dist = max(sqrt(3.), min(dist[LEFT], dist[RIGHT]));
}

/**
 * @brief Calculate accidentalness and significance.
 * Assume endpoints are distributed following a Poisson process with parameter
 * p_lep in space R2 (p_lep = prob. of line endpoints).
 * Be g the vector from the endpoint of the longer line 0 to the endpoint of the
 * shorter line 1, r its length and theta its angle with line 1.
 * Calculate the probability that there are one or more endpoints in area A =
 * 2*theta*r^2. Which is the same as 1 - probability that there is no endpoint
 * in area A.
 * We chose the wedge with width 2*theta instead of the full circle because we
 * are also interested in the direction of the endpoint, not just its distance.
 * Be dphi the angular difference between line 0 and line 1.
 * Then multiply with the probability that line 0 falls within +/-dphi.
 * P = P(endpoint falls in area)*P(angle falls in interval)
 * Note that the lengths of the lines play no role.
 * TODO: check and fix the above descriptive text to match the actual and
 * correct code below.
 */
void Collinearity::CalculateSignificance()
{
  double n = 2.*(double)NumLines(core);
  double r = gap_cor;  // gap corrected for occlusion
  double alpha, beta;
  double f_prox;    // proximity factor
  double f_ori;     // orientation factor
  double f_col;     // color factor
  // gap vector pointing from line 0 to 1
  Vector2 g = line[1]->point[near_point[1]] - line[0]->point[near_point[0]];
  // vector along line 0 pointing towards gap
  Vector2 a = Normalise(line[0]->point[near_point[0]] -
      line[0]->point[Other(near_point[0])]);
  // vector along line 1 pointing towards gap
  Vector2 b = Normalise(line[1]->point[near_point[1]] -
      line[1]->point[Other(near_point[1])]);

  // To avoid 0 angles and thus acc = 0 (sig -> inf) we always assume an angular
  // uncertainty of 1 pixel over line length
  // If there is a gap measure angles of between lines and gap
  if(g.Length() > 1.)
  {
    g.Normalise();
    alpha = fmax(atan(1./line[0]->Length()), acos(Dot(a, g)));
    beta = fmax(atan(1./line[1]->Length()), acos(Dot(b, -g)));
  }
  // else measure angles between lines themselves
  else
  {
    alpha = fmax(atan(1./line[0]->Length()), acos(Dot(a, -b)));
    beta = fmax(atan(1./line[1]->Length()), acos(Dot(b, -a)));
  }

  // proximity factor
  f_prox = alpha*r*r/(double)core->ImageArea();
  // orientation factor
  f_ori = beta/M_PI;
  // color factor
#ifdef USE_COLOR
  f_col = SphereVolume(col_dist)/pow(256., 3.);
#else
  f_col = 1.;
#endif

  acc = 1. - exp(-n*f_prox*f_ori*f_col);
  acc = min(acc, 1.);  // TODO: what's that for??
  sig = -log(acc);
  sig = max(sig, 1e-6);  // TODO: properly
}

/**
 * @brief Correct the gap, if at both ends is a T-junction.
 */
void Collinearity::CorrectGap()
{
  TJunction *t[2];
  t[0] = line[0]->t_jct[near_point[0]];
  t[1] = line[1]->t_jct[near_point[1]];
  // if there are T-junctions at both ends of the collinearity reduce the actual
  // gap by the distance between T-junctions
  if(t[0] != 0 && t[1] != 0)
  {
    double d = Distance(t[0]->isct, t[1]->isct);
    gap_cor = max(1., gap - d);
  }
  else
    gap_cor = gap;
}

/**
 * @brief Check which line is at which position.
 * @return Returns the number in the line array [0/1]
 */
int Collinearity::WhichLineIs(Line *l) throw(std::runtime_error)
{
  if(l == line[0])
    return 0;
  else if(l == line[1])
    return 1;
  else
	{
		char buffer [100];
		sprintf(buffer, "Collinearity::WhichLineIs: line %u is not part of collinearity %u.", l->ID(), ID());
    throw std::runtime_error(buffer);
	}
}

/**
 * @brief Get the other line of the collinearity.
 * @param l Known line
 * @return Returns the other line of the collinearity
 */
Line* Collinearity::OtherLine(Line *l)
{
  return line[Other(WhichLineIs(l))];
}

/**
 * @brief Checks, at which line end the collinearity is.
 * @param l Line
 * @return Returns the line end, on which the collinearity is [START/END]
 */
int Collinearity::WhichEndIs(Line *l)
{
  return near_point[WhichLineIs(l)];
}

}

