/**
 * @file Arc.cc
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Class file of Gestalt arc.
 */

#include <stdio.h>
#include <algorithm>
// extern "C"
// {
// #include "fitcircle.h"
// }
#include "Math.hh"
#include "Draw.hh"
#include "Arc.hh"
#include "FormSegments.hh"
#include "FormArcJunctions.hh"
#include "FormConvexArcGroups.hh"

namespace Z
{

int Arc::STRICT_CONVEXITY = 0;
 
/**
 * @brief Constructor of class Arc.
 * @param vc Vision core
 * @param s Edge segment
 * @param i
 * @param j
 * @param k
 * @param cent
 * @param rad
 */
Arc::Arc(VisionCore  *vc, Segment *s, unsigned i, unsigned j, unsigned k,
    const Vector2 &cent, double rad) : Gestalt(vc, ARC)
{
//   printf("Arc::Arc: start\n");
  seg = s;
  idx[START] = i;
  idx[MID] = j;
  idx[END] = k;
  idx[ONE_THIRD] = i + (k-i)/3;
  idx[TWO_THIRD] = i + (2*(k-i))/3;
  point[START] = seg->edgels[idx[START]].p;
  point[MID] = seg->edgels[idx[MID]].p;
  point[END] = seg->edgels[idx[END]].p;
  point[ONE_THIRD] = seg->edgels[idx[ONE_THIRD]].p;
  point[TWO_THIRD] = seg->edgels[idx[TWO_THIRD]].p;
  center = cent;
  radius = rad;
  // TODO: this sometimes returns odd results, disable for now
  // CalculateParams();
// printf("Arc::Arc: vor normalise: %f - %f - %f - %f\n", center.x, center.y, point[START].x, point[START].y);
  if((center-point[START]).x != 0.0f)
    norm[START] = Normalise(center - point[START]);
// printf("Arc::Arc: vor normalise: %f - %f - %f - %f\n", center.x, center.y, point[MID].x, point[MID].y);
  if((center-point[MID]).x != 0.0f)
    norm[MID] = Normalise(center - point[MID]);
// printf("Arc::Arc: vor normalise: %f - %f - %f - %f\n", center.x, center.y, point[END].x, point[END].y);
  if((center-point[END]).x != 0.0f)
    norm[END] = Normalise(center - point[END]);
// printf("Arc::Arc: nach normalise\n");
  CalculateTangents();
  // Note: START and END points might be swapped according to angles!
  CalculateAngles();
  CalculateColors();
  CalculateSignificance();
  
  //printf("NEW ARC: start,mid,end,center,radius: %1.0f-%1.0f / %1.0f-%1.0f / %1.0f-%1.0f / %1.0f-%1.0f / %1.0f\n", point[START].x, point[START].y, point[MID].x, point[MID].y, point[END].x, point[END].y, center.x, center.y, radius);
//   printf("Arc::Arc: end\n");
}

extern unsigned peek;																																	/// TODO TODO TODO was ist das?

/**
 * @brief Draw the arc.
 * Details are:
 * 1: draw translucent "pie"
 * 2: draw edgels of arc and tangents
 * 3: draw arc id
 * 4: draw the complete originating segment
 */
void Arc::Draw(int detail)
{
  char id_str[20];

	if(detail == 1)
    TransparentArc2D(center.x, center.y, radius, start_angle, angular_span, RGBColor::red);
  
  if(detail == 2)
  {
    for(unsigned i = idx[START]; i <= idx[END]; i++)
      DrawPoint2D(seg->edgels[i].p.x,
          seg->edgels[i].p.y, RGBColor::green);
  }
  
	DrawArc2D(center.x, center.y, radius, start_angle, angular_span);
 
	if(detail == 3) // Normal
  {
    Vector2 p = point[MID];
    Vector2 d = norm[MID].NormalClockwise();
    DrawLine2D(p.x - 100*d.x, p.y - 100*d.y, p.x + 100*d.x, p.y + 100*d.y, RGBColor::yellow);
  }

	if(detail >= 4)	// Segment and search lines
	{
		seg->Draw();
    DrawSearchLines();
	}
  if(detail >= 5)
  {
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, point[MID].x, point[MID].y, RGBColor::red);
    DrawText2D("START", point[START].x, point[START].y, RGBColor::red);
    DrawText2D("END", point[END].x, point[END].y, RGBColor::red);
  }
  if(detail >= 6)
  {
    //!!! temp: draw color histogram sampling
    const unsigned NUM_SAMPLES = 16;
    unsigned i, j;
    int x1, y1, x2, y2;
    RGBColor col;
    // sample the arc at regular intervals
    for(j = 0; j < NUM_SAMPLES; j++)
    {
      i = idx[START] + ((idx[END] - idx[START])*j)/(NUM_SAMPLES - 1);
      // find the point halfway between arc and next edge
      x1 = (int)seg->edgels[i].p.x;
      y1 = (int)seg->edgels[i].p.y;
      if(FormSegments::edge_img->FindLineEnd(x1, y1, (int)center.x, (int)center.y, &x2, &y2))
        DrawPoint2D(x2, y2, RGBColor::blue);
    }
    for(j = 0; j < NUM_SAMPLES; j++)
    {
      i = idx[START] + ((idx[END] - idx[START])*j)/(NUM_SAMPLES - 1);
      Vector2 d = seg->edgels[i].p - center;
      // find the point halfway between arc and next edge
      x1 = (int)seg->edgels[i].p.x;
      y1 = (int)seg->edgels[i].p.y;
      if(FormSegments::edge_img->FindLineEnd(x1, y1, x1 + (int)d.x, y1 + (int)d.y, &x2, &y2))
        DrawPoint2D(x2, y2, RGBColor::blue);
    }
    //!!! end temp
  }
}

/**
 * @brief Draw the search lines from the arc.
 */
void Arc::DrawSearchLines()
{
  VoteImage *vi = core->VI();
  int x, y;
  VoteImage::Elem *el;
  if(vi == 0)
    return;
  for(x = 0; x < vi->width; x++)
    for(y = 0; y < vi->height; y++)
    {
      el = vi->Pixel(x, y);
      while(el != 0)
      {
        if((el->id/vi->GetBaseIndex()-vi->GetArcOffset()) == id)
        {
          unsigned vtype = el->id%vi->GetBaseIndex();
          switch(vtype)
          {
            case VOTE_ATS:
            case VOTE_ANLS:
            case VOTE_ANRS:
              DrawPoint2D(x, y, RGBColor::magenta);
              break;
            case VOTE_ATE:
            case VOTE_ANLE:
            case VOTE_ANRE:
              DrawPoint2D(x, y, RGBColor::cyan);
              break;
            default:
              DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
}

/**
 * @brief Draw arc info.
 */
void Arc::DrawInfo()
{
	printf("Arc::DrawInfo: Not yet implemented!\n");
//   //col_hist_inside.Draw(0., 0., 1., 1.);
//   FillRect2D(0., 0., 0.5, 1., mean_col_inside);
//   DrawText2D("inside", 0.1, 0.8, RGBColor::green);
//   FillRect2D(0.5, 0., 1., 1., mean_col_outside);
//   DrawText2D("outside", 0.6, 0.8, RGBColor::green);
}

/**
 * @brief Get arc informations.
 * @return Returns the information about the arc as string.
 */
const char* Arc::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%s  edgels: %u\n  span: %f\n  length: %f\n  radius: %f\n  seg: %u %u-%u",
      Gestalt::GetInfo(), NumEdgels(), angular_span, ArcLength(), Radius(),
      seg->ID(), idx[START], idx[END]);
  return info_text;
}

/**
 * @brief Returns true, if arc is at this possition.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if arc is at this possition.
 */
bool Arc::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = idx[START]; i <= idx[END]; i++)
    if(IsEqual(xd, seg->edgels[i].p.x) && IsEqual(yd, seg->edgels[i].p.y))
      return true;
  return false;
}


/**
 * @brief Calculate parameters.
 */
void Arc::CalculateParams()
{
printf("Arc::CalculateParams: Not yet implemented!\n");
//   double *x = new double[NumEdgels()];
//   double *y = new double[NumEdgels()];
//   for(unsigned i = idx[START], j = 0; i <= idx[END]; i++, j++)
//   {
//     x[j] = seg->edgels[i].p.x;
//     y[j] = seg->edgels[i].p.y;
//   }
//   fit_circle(x, y, NumEdgels(), &center.x, &center.y, &radius);
//   delete[] x;
//   delete[] y;
}

/**
 * @brief Order the given angles such that a,b,c are in counter clockwise order (i.e.
 * ascending in mathematical sense). The angles are either clockwise already
 * (then b - a > 0) or counterclockwise (then b - a < 0). If so, swap a and c.
 * Returns true if angles were already clockwise, false otherwise.
 * Note that since the sum of all angles is <= 2 pi and b is in the middle of
 * a and c, differences b - a and c - b can only be <= pi -> differences are
 * unique.
 * @param a
 * @param b
 * @param c
 * @return
 */
bool Arc::OrderAnglesCounterClockwise(double &a, double &b, double &c)
{
  if(DiffAngle_mpi_pi(b, a) < 0.)
  {
    swap(a, c);
    return false;
  }
  else
    return true;
}

/**
 * @brief Calculate start, mid and end angle.
 * NOTE: START and END points (and normals etc.) might be swapped according to
 * angles!
 * NOTE: PolarAngle delivers angle clockwise => Reverted to counter-clockwise
 */
void Arc::CalculateAngles()
{
//   printf("  Arc::CalculateAngles: start: start,mid,end,center,radius: %1.0f-%1.0f / %1.0f-%1.0f / %1.0f-%1.0f / %1.0f-%1.0f / %1.0f\n", point[START].x, point[START].y, point[MID].x, point[MID].y, point[END].x, point[END].y, center.x, center.y, radius);
  double mid_angle;
  Vector2 v1(point[START] - center);
  Vector2 v2(point[MID] - center);
  Vector2 v3(point[END] - center);
  //printf("  v1: %4.0f-%4.0f - v2: %4.0f-%4.0f - v3: %4.0f-%4.0f\n", v1.x, v1.y, v2.x, v2.y, v3.x, v3.y);
  start_angle = -PolarAngle(v1);
  mid_angle = -PolarAngle(v2);
  end_angle = -PolarAngle(v3);
  //printf("  before: %4.3f - %4.3f - %4.3f\n", start_angle*180./M_PI, mid_angle*180./M_PI, end_angle*180./M_PI);

if(!OrderAnglesCounterClockwise(start_angle, mid_angle, end_angle))
  {
    // if angles were swapped, also swap the corresponding points
    swap(point[START], point[END]);
    swap(point[ONE_THIRD], point[TWO_THIRD]);
    swap(norm[START], norm[END]);
    swap(tang_pt[0], tang_pt[1]);
    swap(tang[0], tang[1]);
  }
  // using the center angle we get the correct span also for spans > pi
  // note: for arcs spanning the whole circle, mid - start angle might be + or
  // -pi, therefore use absolute value
  angular_span = fabs(DiffAngle_mpi_pi(end_angle, mid_angle)) +
                 fabs(DiffAngle_mpi_pi(mid_angle, start_angle));

//   printf(" Arc::CalculateAngles: after: %4.3f-%4.3f-%4.3f => %4.3f\n", start_angle*180./M_PI, mid_angle*180./M_PI, end_angle*180./M_PI, angular_span*180./M_PI);
}

/**
 * @brief Calculate tangents at two points at 1/3 and 2/3 of arc length.
 */
void Arc::CalculateTangents()
{
  tang_pt[0] = seg->edgels[idx[ONE_THIRD]].p;
  tang[0] = seg->Tangent(idx[ONE_THIRD], idx[START], idx[END]);
  tang_pt[1] = seg->edgels[idx[TWO_THIRD]].p;
  tang[1] = seg->Tangent(idx[TWO_THIRD], idx[START], idx[END]);
}

/**
 * @brief Calculate mean colors inside and outside of arc.
 */
void Arc::CalculateColors()
{
  const unsigned NUM_SAMPLES = 16;
  unsigned i, j;
  int x1, y1, x2, y2;
  int ri = 0, gi = 0, bi = 0;
  int ro = 0, go = 0, bo = 0;
  Vector2 d;
  RGBColor col;
  // sample the arc at regular intervals
  for(j = 0; j < NUM_SAMPLES; j++)
  {
    i = idx[START] + ((idx[END] - idx[START])*j)/(NUM_SAMPLES - 1);
    // find the points halfway between arc and next edge
    // inside
    x1 = (int)seg->edgels[i].p.x;
    y1 = (int)seg->edgels[i].p.y;
    if(FormSegments::edge_img->FindLineEnd(x1, y1, (int)center.x, (int)center.y,
        &x2, &y2))
      col = core->Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    ri += (int)col.r;
    gi += (int)col.g;
    bi += (int)col.b;
    // outside
    d = seg->edgels[i].p - center;
    if(FormSegments::edge_img->FindLineEnd(x1, y1, x1 + (int)d.x, y1 + (int)d.y,
        &x2, &y2))
      col = core->Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    ro += (int)col.r;
    go += (int)col.g;
    bo += (int)col.b;
  }
  mean_col_inside.r = ri/NUM_SAMPLES; 
  mean_col_inside.g = gi/NUM_SAMPLES; 
  mean_col_inside.b = bi/NUM_SAMPLES; 
  mean_col_outside.r = ro/NUM_SAMPLES; 
  mean_col_outside.g = go/NUM_SAMPLES; 
  mean_col_outside.b = bo/NUM_SAMPLES; 
}

/**
 * @brief Returns true if arc b lies "inside" this arc a, false otherwise
 *  b inside a:    b not inside a:
 *
 *   /    \          \   \
 *  |     |          |   |
 *   \   /          /   /
 *  a   b          a    b
 *
 * Calculation: v is a vector from point p on arc a to the circle center. If
 * dotproduct v.(x - p) for every point x on arc b is positive (angle<90) then b
 * lies on the "inside" of a. 
 */
bool Arc::HasInside(Arc *b)
{
  if(STRICT_CONVEXITY)
    return HasInsideStrong(b);
  else
    return HasInsideWeak(b);
}

/**
 * @brief Strong convexity criterion.
 * Checks against half-planes at end points.
 * @param b Arc
 */
bool Arc::HasInsideStrong(Arc *b)
{
  Vector2 b1s = b->point[ONE_THIRD] - point[START];
  Vector2 b1e = b->point[ONE_THIRD] - point[END];
  Vector2 b2s = b->point[TWO_THIRD] - point[START];
  Vector2 b2e = b->point[TWO_THIRD] - point[END];
  // check tangent halfplanes
  if(Dot(norm[START], b1s) < 0.)
    return false;
  if(Dot(norm[END], b1e) < 0.)
    return false;
  if(Dot(norm[START], b2s) < 0.)
    return false;
  if(Dot(norm[END], b2e) < 0.)
    return false;
  // check overlap
  if(Length(b->point[ONE_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b1s) && LeftOf(norm[END], b1e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b1s) || LeftOf(norm[END], b1e))
        return false;
    }
  }
  if(Length(b->point[TWO_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b2s) && LeftOf(norm[END], b2e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b2s) || LeftOf(norm[END], b2e))
        return false;
    }
  }
  return true;
}

/**
 * @brief Weak convexity criterion.
 * Checks against half-plane at midpoint.
 * @param b Arc
 * @return Returns true, if ...?	TODO
 */
bool Arc::HasInsideWeak(Arc *b)
{
  Vector2 b1s = b->point[ONE_THIRD] - point[START];
  Vector2 b1e = b->point[ONE_THIRD] - point[END];
  Vector2 b1m = b->point[ONE_THIRD] - point[MID];
  Vector2 b2s = b->point[TWO_THIRD] - point[START];
  Vector2 b2e = b->point[TWO_THIRD] - point[END];
  Vector2 b2m = b->point[TWO_THIRD] - point[MID];
  /*Vector2 b1s = b->point[START] - point[START];
  Vector2 b1e = b->point[START] - point[END];
  Vector2 b1m = b->point[START] - point[MID];
  Vector2 b2s = b->point[END] - point[START];
  Vector2 b2e = b->point[END] - point[END];
  Vector2 b2m = b->point[END] - point[MID];*/
  // check halfplane
  if(Dot(norm[MID], b1m) < 0.)
    return false;
  if(Dot(norm[MID], b2m) < 0.)
    return false;
  // check overlap
  if(Length(b->point[ONE_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b1s) && LeftOf(norm[END], b1e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b1s) || LeftOf(norm[END], b1e))
        return false;
    }
  }
  if(Length(b->point[TWO_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b2s) && LeftOf(norm[END], b2e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b2s) || LeftOf(norm[END], b2e))
        return false;
    }
  }
  return true;
}

bool Arc::HasInside(Vector2 &p)
{
  if(STRICT_CONVEXITY)
  {
    if(Dot(norm[START], p - point[START]) < 0.)
      return false;
    if(Dot(norm[END], p - point[END]) < 0.)
      return false;
  }
  else
  {
    if(Dot(norm[MID], p - point[MID]) < 0.)
      return false;
  }
  return true;
}

/**
 * @brief Returns true if two arcs are convex (form a convex pair), false otherwise
 *
 * TODO: try using weak convexity for near points, strong convexity for far
 * points.
 *
 *  convex:        not convex:
 *
 *   /    \          \    /
 *  |     |          |   |
 *   \   /          /     \
 *  
 * @param b Arc
 * @return Returns true, when convex.
 */
bool Arc::ConvexWith(Arc *b)
{
  return this->HasInside(b) && b->HasInside(this);
}


/**
 * @brief A "soft" inside criterion.
 * @param Arc
 * @return Returns a (double) significance value for "insideness"
 */
double Arc::Insideness(Arc *b)
{
  int inside = 0;
  for(unsigned i = b->idx[START]; i <= b->idx[END]; i++)
    if(Dot(norm[START], b->seg->edgels[i].p - point[START]) > 0. &&
        Dot(norm[END], b->seg->edgels[i].p - point[END]) > 0.)
      inside++;
  return (double)inside/(double)b->NumEdgels();
}

/**
 * @brief A "soft" convexity criterion.
 * @param b Arc
 * @return Returns true, when ...? TODO
 */
double Arc::Convexity(Arc *b)
{
  return fmin(this->Insideness(b), b->Insideness(this));
}

/**
 * @brief Returns true if point p lies inside the "wedge" defined by center and
 * endpoints.
 * @param b Arc
 * @return Returns true, when ...? TODO
 */
bool Arc::HasInsideWedge(Arc *b)
{
  Vector2 v1 = (point[START] - center).NormalAntiClockwise();
  Vector2 v3 = (point[END] - center).NormalClockwise();
  Vector2 s = b->point[START] - center;
  Vector2 c = b->center - center;
  Vector2 e = b->point[END] - center;
  if(s.Norm() <= Radius() && Dot(s, v1) >= 0. && Dot(s, v3) >= 0.)
    return true;
  if(c.Norm() <= Radius() && Dot(c, v1) >= 0. && Dot(c, v3) >= 0.)
    return true;
  if(e.Norm() <= Radius() && Dot(e, v1) >= 0. && Dot(e, v3) >= 0.)
    return true;
  return false;
}

/**
 * @brief	TODO
 * @param b Arc
 * @return Returns true, when ...? TODO
 */
bool Arc::HasInsideBeam(Arc *b)
{
  Vector2 m0 = point[MID] - center;
  m0.Normalise();
  Vector2 s = point[START] - center;
  Vector2 sn = s - m0*Dot(s, m0);
  Vector2 e = point[END] - center;
  Vector2 en = e - m0*Dot(e, m0);
  if(Dot(b->point[START] - point[START], sn) <= 0 &&
     Dot(b->point[START] - point[END], en) <= 0)
    return true;
  if(Dot(b->point[END] - point[START], sn) <= 0 &&
     Dot(b->point[END] - point[END], en) <= 0)
    return true;
  return false;
}

/**
 * @brief Returns true if there is a coaxial overlap between the two arcs.
 * @param b Arc
 * @return Returns true if there is a coaxial overlap between the two arcs.
 */
bool Arc::CoaxialOverlap(Arc *b)
{
  return this->HasInsideWedge(b) || b->HasInsideWedge(this);
  //return this->HasInsideBeam(b) || b->HasInsideBeam(this);
}

/**
 * @brief Calculates significance value for the Gestalt.
 */
void Arc::CalculateSignificance()
{
  //int l = NumEdgels();//(int)(2.*M_PI*Radius());
  //sig = Significance(3, NumEdgels(), l, core->p_e);
	sig = -log(pow(core->p_ee, (double)NumEdgels()));
}

/**
 * @brief Add a junction (AJunction) to the arc (as additional information).
 * @param end Which arc end [START/END]
 * @param j AJunction
 */
void Arc::AddJunction(int end, AJunction *j)
{
  jct[end].PushBack(j);
}

}

