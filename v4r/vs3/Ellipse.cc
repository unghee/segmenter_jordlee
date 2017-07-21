/**
 * @file Ellipse.cc
 * @author Zillich, Richtsfeld
 * @date 2006, Dec. 2009, 2010
 * @version 0.1
 * @brief Gestalt feature Ellipse
 */

#include <math.h>
#include <assert.h>
#include <algorithm>
#include <cstdio>
#ifdef USE_OPENCV_ELLFIT
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
// extern "C" for FitEllipse_b2ac
// {
// #include "fitellipse.h"
// }
#endif
#include "Draw.hh"
#include "Segment.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "Ellipse.hh"
#include "FormSegments.hh"
#include "Line.hh"
#include "EJunction.hh"

namespace Z
{

// use ellipse curvature for additional weighting in support calculation
// experiments show that this does not improve results, rather the contrary
//#define USE_CURVATURE_WEIGHTING
 
// distance of an edgel from an ellipse to consider it supporting
#define SUPPORT_DISTANCE 1.

#ifdef USE_B2AC_ELLFIT
static bool FitEllipse_b2ac(Array<Arc*> &arcs, unsigned l, unsigned u,
    double &x, double &y, double &a, double &b, double &phi)
{
printf("Ellipse: FitEllipse_b2ac not yet implemented!\n");
//   unsigned n = 0, i, j;
//   double *points_x = 0, *points_y = 0;
// 
//   for(i = l; i <= u; i++)
//     n += arcs[i]->NumEdgels();
//   points_x = new double[n];
//   assert(points_x != 0);
//   points_y = new double[n];
//   assert(points_y != 0);
//   n = 0;
//   for(i = l; i <= u; i++)
//     for(j = arcs[i]->idx[START]; j <= arcs[i]->idx[END]; j++)
//     {
//       points_x[n] = arcs[i]->seg->edgels[j].p.x;
//       points_y[n] = arcs[i]->seg->edgels[j].p.y;
//       n++;
//     }
//   bool fit_ok = fit_ellipse(points_x, points_y, (int)n, &x, &y, &a, &b, &phi);
//   delete[] points_x;
//   delete[] points_y;
//   return fit_ok;
}
#else
static bool FitEllipse_opencv(Array<Arc*> &arcs, unsigned l, unsigned u,
    double &x, double &y, double &a, double &b, double &phi)
{
  unsigned n = 0, i, j;
  CvPoint2D32f *points = 0;
  CvBox2D params;

  for(i = l; i <= u; i++)
    n += arcs[i]->NumEdgels();
  points = new CvPoint2D32f[n];
  assert(points != 0);
  n = 0;
  for(i = l; i <= u; i++)
    for(j = arcs[i]->idx[START]; j <= arcs[i]->idx[END]; j++)
    {
      points[n].x = arcs[i]->seg->edgels[j].p.x;
      points[n].y = arcs[i]->seg->edgels[j].p.y;
      n++;
    }

printf("[Ellips::FitEllipse_opencv] ERROR: Antiquated! Update this function to new opencv version!\n");
//   cvFitEllipse(points, n, &params);
  x = params.center.x;
  y = params.center.y;
  // box size is double the axis lengths
  a = params.size.width/2.;
  b = params.size.height/2.;
  // note: the angle returned is in degrees!
  phi = ScaleAngle_0_2pi(params.angle*M_PI/180.);
//  phi = ScaleAngle_0_pi(params.angle*M_PI/180.);					/// HACK ARI: scale to 0..pi
  // note: for unknown reasons sometimes a < b!
  if(a < b)
  {
    swap(a, b);
    phi = ScaleAngle_0_2pi(phi + M_PI_2);
//    phi = ScaleAngle_0_pi(phi + M_PI_2);									/// HACK ARI: scale to 0..pi
  }
  delete[] points;
  return true;
}
#endif

/**
 * @brief Fit ellipse to arcs, from arcs[l] to arcs[u].
 */
bool FitEllipse(Array<Arc*> &arcs, unsigned l, unsigned u,
    double &x, double &y, double &a, double &b, double &phi)
{
#ifdef USE_B2AC_ELLFIT
  return FitEllipse_b2ac(arcs, l, u, x, y, a, b, phi);
#else
  return FitEllipse_opencv(arcs, l, u, x, y, a, b, phi);
#endif
}

/**
 * @brief Fit ellipse to arcs.
 */
bool FitEllipse(Array<Arc*> &arcs,
    double &x, double &y, double &a, double &b, double &phi)
{
#ifdef USE_B2AC_ELLFIT
  return FitEllipse_b2ac(arcs, 0, arcs.Size() - 1, x, y, a, b, phi);
#else
  return FitEllipse_opencv(arcs, 0, arcs.Size() - 1, x, y, a, b, phi);
#endif
}

/**
 * @brief Approximation to ellipse circumference.
 * (from Bartsch: Mathematische Formeln, Buch- und Zeit-Verlagsgesellschaft
 * Koeln, 1988, p. 221)
 */
double EllipseCircumference(double a, double b)
{
  return M_PI*(1.5*(a + b) - sqrt(a*b));
}

double EllipseArea(double a, double b)
{
  return M_PI*a*b;
}

/**
 * @brief Ellipse curvature at point (x,y) which is given in ellipse coordinates.
 */
double EllipseCurvature(double a, double b, double x, double y)
{
  double t = atan2(a*y, b*x);
  return a*b / pow( Sqr(b)*Sqr(cos(t)) + Sqr(a)*Sqr(sin(t)), 1.5 );
}

/**
 * @brief Calculate the support for an ellipse.
 * Support is defined by a simple distance threshold. Each supporting edgel is
 * additionally weighted by how much of the arc containing that edgel is
 * actually supporting. I.e. if an arc intersects the ellipse at an almost
 * normal angle, the one or two supporting edgels are weighted low. If an arc
 * is really part of an ellipse, most of its edgels are supporting and are
 * therefore weighted higher.
 * This weighting serves to lower the support for weird accidental ellipse
 * hypotheses that cover half the image and intersect dozens of useless arcs.
 * Finally the support is divded by the ellipse circumference to get the
 * relative support.
 * @param arcs
 * @param l
 * @param u
 * @param x
 * @param y
 * @param a
 * @param b
 * @param phi
 * @param fit_error
 * @param abs_sup
 * @return TODO
 */
double EllipseSupport(Array<Arc*> &arcs, unsigned l, unsigned u,
    double x, double y, double a, double b, double phi, double &fit_error,
    double *abs_sup)
{
  double sup = 0., sup_per_arc;
  double co = cos(-phi), si = sin(-phi), x2, y2, a2, b2, a4, b4, n, d, dist;
#ifdef USE_CURVATURE_WEIGHTING
  double kap, kap_max;
#endif
  unsigned i, j, n_pix = 0;
  Vector2 p, q;

  a2 = Sqr(a);
  b2 = Sqr(b);
  a4 = Sqr(a2);
  b4 = Sqr(b2);
#ifdef USE_CURVATURE_WEIGHTING
  // maximum curvature is at point (a,0)
  kap_max = EllipseCurvature(a, b, a, 0.);
#endif
  fit_error = 0.;
  for(i = l; i <= u; i++)
  {
    sup_per_arc = 0.;
    for(j = arcs[i]->idx[START]; j <= arcs[i]->idx[END]; j++)
    {
      // transform point in image coords to ellipse coords
      // Note that this piece of code is called often.
      // Implementing this explicitely here is faster than using
      // TransformToEllipse(), as cos and sin only need to be evaluated once.
      p = arcs[i]->seg->edgels[j].p;
      p.x -= x;
      p.y -= y;
      q.x = co*p.x - si*p.y;
      q.y = si*p.x + co*p.y;
      // calculate absolute distance to ellipse
      if(IsZero(q.x) && IsZero(q.y))
        dist = b;
      else
      {
        x2 = Sqr(q.x);
        y2 = Sqr(q.y);
        n = fabs(x2/a2 + y2/b2 - 1.);
        d = 2.*sqrt(x2/a4 + y2/b4);
        dist = n/d;
      }
      // calculate curvature
      if(dist <= SUPPORT_DISTANCE)
      {
#ifdef USE_CURVATURE_WEIGHTING
        kap = EllipseCurvature(a, b, q.x, q.y);
        sup_per_arc += 1.*kap/kap_max;
#else
        sup_per_arc += 1.;
#endif
      }
      fit_error += dist;
      n_pix++;
    }
    // weight support per arc by percentage of supporting edgels
    sup += sup_per_arc * sup_per_arc/(double)arcs[i]->NumEdgels();
  }
  fit_error /= (double)n_pix;
  if(abs_sup)
    *abs_sup = sup;
  return sup/EllipseCircumference(a, b);
}

/**
 * @brief 
 * @param arcs
 * @param x
 * @param y
 * @param a
 * @param b
 * @param phi
 * @param fit_error
 * @param abs_sup
 * @return TODO
 */
double EllipseSupport(Array<Arc*> &arcs, double x, double y, double a, double b, 
    double phi, double &fit_error, double *abs_sup)
{
  return EllipseSupport(arcs, 0, arcs.Size() - 1, x, y, a, b, phi, fit_error, abs_sup);
}

/**
 * @brief Constructor of class Ellipse.
 * @param vc Vision core
 * @param grp_in ConvexArcGroup
 * @param x_in x-coordinate parameter of ellipse
 * @param y_in y-coordinate parameter of ellipse
 * @param a_in a parameter of ellipse (main axis length)
 * @param b_in b parameter of ellipse (non-main axis length
 * @param phi_in phi parameter of ellipse (angle)
 */
Ellipse::Ellipse(VisionCore *vc, ConvexArcGroup *grp_in, double x_in, double y_in, 
    double a_in, double b_in, double phi_in) : Gestalt(vc, ELLIPSE)
{
  group = grp_in;
  x = x_in;
  y = y_in;
  a = a_in;
  b = b_in;
  phi = phi_in;
	
	center.x = x;
	center.y = y;

  // these are frequently needed by Distance(), so let's cache them
  a2 = Sqr(a);
  a4 = Sqr(a2);
  b2 = Sqr(b);
  b4 = Sqr(b2);

	// vertices of the ellipse normalized to 0-Pi
  int l=0, r=1;					// left/right
  if (phi > M_PI) {l=1; r=0;}	// change left right, if phi > Pi
  vertex[r].x = x - (a*cos(phi));
  vertex[r].y = y - (a*sin(phi));
  vertex[l].x = x + (a*cos(phi));
  vertex[l].y = y + (a*sin(phi));
	
  // normal direction of the ellipse (0-pi)
  dir = Normalise(vertex[RIGHT] - vertex[LEFT]);
	
  support = abs_support = 0.;
  fit_error = HUGE;
  area = 0.;
  sig = -HUGE;
  area = Area();
  support = EllipseSupport(group->arcs, x, y, a, b, phi,
      fit_error, &abs_support);
  CalculateSignificance();
}

/**
 * @brief Calculate circumference of the ellipse.
 * @return Returns the ellipse circumference
 */
double Ellipse::Circumference()
{
  return EllipseCircumference(a, b);
}

/**
 * @brief Calculate area of the ellipse.
 * @return Returns the area of the ellipse.
 */
double Ellipse::Area()
{
  return EllipseArea(a, b);
}

/**
 * @brief Transform a point from image to ellipse co-ordinates.
 * @param p Point to transform.
 * @return Returns the transformed point.
 */
Vector2 Ellipse::TransformToEllipse(const Vector2 &p)
{
  return Rotate(p - Vector2(x, y), -phi);
}

/**
 * @brief Transform a point from ellipse to image co-ordinates.
 * @param p Point to transform.
 * @return Returns the transformed point.
 */
Vector2 Ellipse::TransformFromEllipse(const Vector2 &p)
{
  return Rotate(p, phi) + Vector2(x, y);
}

/**
 * @brief Approximation of the shortest abolute distance of a point to an ellipse.
 * The method used is gradient-weighted algebraic distance, which requires that
 * the ellipse be centered in the origin and the big axis a lies on
 * the x-axis. I.e. the point x,y must first be transformed to ellipse
 * co-ordinates. This is done inside the Distance function. So You don't have
 * to worry about that.
 * @param p Point
 * @return Returns the shortest distance of point to ellipse.
 */
double Ellipse::Distance(const Vector2 &p)
{
  return DistanceCentAxPar(TransformToEllipse(p));
}

/**
 * @brief Approximation of the shortest absolute distance of a point to a centered,
 * axis-parallel ellipse.
 * @param p Point
 * @return Returns the shortest distance of point to a centered axis-parallel ellipse
 */
double Ellipse::DistanceCentAxPar(const Vector2 &p)
{
  double x2, y2, n, d;
  if(IsZero(p.x) && IsZero(p.y))
    return b;
  x2 = Sqr(p.x);
  y2 = Sqr(p.y);
  n = fabs(x2/a2 + y2/b2 - 1.);
  d = 2.*sqrt(x2/a4 + y2/b4);
  return n/d;
}

/**
 * @brief 
 * @param 
 * @return TODO
 */
double Ellipse::Distance(const Edgel &e)
{
  Vector2 p = TransformToEllipse(e.p);
  if(GradientMatchesCentAxPar(p, ScaleAngle_0_2pi(e.dir - phi)))
    return DistanceCentAxPar(p);
  else
    return 10000.;
}

/**
 * @brief 
 * @param 
 * @return TODO
 */
bool Ellipse::GradientMatches(const Edgel &e)
{
  return GradientMatchesCentAxPar(TransformToEllipse(e.p),
      ScaleAngle_0_2pi(e.dir - phi));
}

/**
 * @brief 
 * @param 
 * @return TODO
 */
bool Ellipse::GradientMatches(const Vector2 &p, double dir)
{
  return GradientMatchesCentAxPar(TransformToEllipse(p),
      ScaleAngle_0_2pi(dir - phi));
}

/**
 * @brief 
 * @param 
 * @return TODO
 */
bool Ellipse::GradientMatchesCentAxPar(const Vector2 &p, double dir)
{
  Vector2 n = NormalCentAxPar(p);
  double ell_grad_dir = atan2(n.y, n.x);
  return AngleBetweenLines(ell_grad_dir, dir) <= M_PI_4;
}

/**
 * @brief Tangent vector at given ellipse point.
 * Always points counterclockwise.
 * @param p Point
 * @return Returns the tangent vector at a given ellipse point.
 */
Vector2 Ellipse::Tangent(const Vector2 &p)
{
  return Rotate(TangentCentAxPar(TransformToEllipse(p)), phi);
}

/**
 * @brief Tangent vector at given ellipse point, for centered, axis-parallel ellipse.
 * Always points counterclockwise.
 * @param p Point
 * @return Returns the tangent vector at a given ellipse point, for centered, axis-parallel ellipse.
 */
Vector2 Ellipse::TangentCentAxPar(const Vector2 &p)
{
  double t = atan2(a*p.y, b*p.x);
  return Vector2(-a*sin(t), b*cos(t));
}

/**
 * @brief Normal vector at given ellipse point, for centered, axis-parallel ellipse.
 * Always points outward.
 * @param p Point
 * @return Returns the normal vector at a given ellipse point.
 */
Vector2 Ellipse::Normal(const Vector2 &p)
{
  return Rotate(NormalCentAxPar(TransformToEllipse(p)), phi);
}

/**
 * @brief Normal vector at given ellipse point.
 * Always points outward.
 * @param p Point
 * @return Returns the normal of a given ellipse point.
 */
Vector2 Ellipse::NormalCentAxPar(const Vector2 &p)
{
  return TangentCentAxPar(p).NormalClockwise();
}


/**
 * @brief Draw the Gestalt.
 * @param detail Degree of detail.
 */
void Ellipse::Draw(int detail)
{
	if(detail == 1) DrawVotes();
	if(detail == 2)
	{
		char id_str[20];
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, x, y, RGBColor::red);
		DrawEllipse2D(x, y, a, b, phi, RGBColor::red);
	}
  if(detail >= 3)
    group->Draw(detail-1);
	else	
		DrawEllipse2D(x, y, a, b, phi);
}

/**
 * @brief Draw the votes of the ellipse
 */
void Ellipse::DrawVotes()
{
  VoteImage *vi = core->VI();
  if(vi == 0)
    return;
	unsigned baseIndex = vi->GetBaseIndex();
	unsigned ellOffset = vi->GetEllOffset();

	for(int x = 0; x < vi->width; x++)
	{
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        if((el->id/baseIndex == ID()+ellOffset))
        {
          unsigned vtype = el->id%baseIndex;
          switch(vtype)
          {
						case VOTE_EOTL:
							DrawPoint2D(x, y, RGBColor::magenta);
							break;
						case VOTE_EOTR:
							DrawPoint2D(x, y, RGBColor::magenta);
							break;
						case VOTE_EITL:
							DrawPoint2D(x, y, RGBColor::cyan);
							break;
						case VOTE_EITR:
							DrawPoint2D(x, y, RGBColor::cyan);
							break;
            default: // line itself
							DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
	}
}

/**
 * @brief Get information about the Gestalt.
 * @return Returns the information about the Gestalt as string.
 */
const char* Ellipse::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
	int n=0;
  n += snprintf(info_text, info_size,
		"%s  params: %.2f %.2f  %.2f %.2f  %.4f\n"
		"  support: rel %f  abs: %f\n"
		"  circumference: %.2f area: %.2f\n"
		"  fit_err: %f\n",
    Gestalt::GetInfo(), x, y, a, b, phi, support, abs_support, Circumference(),
    Area(), fit_error);

	n += snprintf(info_text + n, info_size - n, "  ------------\n  %u e-jcts left: ", ejcts[0].Size());
	for (unsigned i=0; i<ejcts[0].Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", ejcts[0][i]->ID());

	n += snprintf(info_text + n, info_size - n, "\n  %u e-jcts right: ", ejcts[1].Size());
	for (unsigned i=0; i<ejcts[1].Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", ejcts[1][i]->ID());
		
	return info_text;
}

/**
 * @brief Checks, if Gestalt is at given position x,y.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if the ellipse is at position x,y.
 */
bool Ellipse::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < group->arcs.Size(); i++)
    if(group->arcs[i]->IsAtPosition(x, y))
      return true;
  return false;
}

/**
 * TODO: why not use LogBinomialCDF_tail??
 * @brief Calculate significance of the Gestalt
 */
void Ellipse::CalculateSignificance()
{
  //sig = Significance(5, nedgels, l, FormSegments::p_edgel);
  // If k < p*l, i.e. if fewer than expected edgels are supporting, this
  // would also be unlikely and therefore significant. But we explicitely
  // reject this case.
  int k = (int)abs_support;
  int l = (int)Circumference();
  // when using contour
  //int k = nedgels;
  // int l = contour.Size();
  k = min(k, l);
  if((double)k < (double)l*core->p_e)
    sig = -1.;
  else
  {
    sig = (double)k*log(core->p_ee) -
          LogBinDist(l, k, core->p_e);
    //sig = LogBinDist(l, k, VisionCore::p_ee) -
    //      LogBinDist(l, k, VisionCore::p_e);
    //sig = -LogBinDist(l, k, VisionCore::p_e);
    //sig = -LogBinomialCDF_tail(l, k, VisionCore::p_e);
  }
  // significance is gap_size/area
  //sig = Area()/(Circumference()*(1. - support));
  // significance is inverse to fit error

  //sig = 1./fit_error;

  //sig = support;

  //sig = abs_support;

  /*if(k > 5)
    sig = -k*log(VisionCore::p_ee) + LogBinCoef(k, 5);
  else
    sig = -k*log(VisionCore::p_ee);*/
}

/** Definition of neigbors */
static int neighbours[8][2] =
  {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

/**
 * @brief Find optimal neighbour of (x,y) in directions from a to b.
 * Result is returned in x_o, y_o. Optimal distance d_o.
 * @param x
 * @param y
 * @param a
 * @param b
 * @param x_o
 * @param y_o
 * @param d_o TODO
 */
void Ellipse::OptimalNeighbour(int x, int y, int a, int b, int &x_o, int &y_o, double &d_o)
{
  int x_n, y_n, i = a;
  double d;
  bool cont = true;
  x_o = INT_MAX;
  y_o = INT_MAX;
  d_o = HUGE;
  while(cont)
  {
    if(i == b)
      cont = false;
    x_n = x + neighbours[i][0];
    y_n = y + neighbours[i][1];
    d = Distance(Vector2(x_n, y_n));
    if(d < d_o)
    {
      d_o = d;
      x_o = x_n;
      y_o = y_n;
    }
    i = ScaleIntAngle_0_8(i + 1);
  }
}

/**
 * TODO: beautify this
 * @brief Build contour string
 */
void Ellipse::BuildContourString()
{
  int x, y, x_s, y_s, x_o, y_o, i, cnt = 0;
  double d, d_o;
  // estimated start point of contour
  Vector2 p = TransformFromEllipse(Vector2(a, 0.));
  x = (int)round(p.x);
  y = (int)round(p.y);
  d = Distance(Vector2(x, y));
  /*contour.Clear();
  contour.PushBack(ContourPoint(x, y, d));
  x_s = x;
  y_s = y;*/
  // check if any neighbour is nearer to ellipse
  OptimalNeighbour(x, y, 0, 7, x_o, y_o, d_o);
  if(d < d_o)
  {
    d_o = d;
    x_o = x;
    y_o = y;
  }
  contour.Clear();
  contour.PushBack(ContourPoint(x_o, y_o, d_o));
  // this is now the real start point
  x_s = x = x_o;
  y_s = y = y_o;
  while(true)
  {
    // TODO: dont need Normalise here
    Vector2 t = Normalise(Tangent(Vector2(x, y)));
    // direction of tangent in [0..8[
    // TODO: shouln't i subtract an offset here of 22.5 deg?
    i = (int)round(ScaleAngle_0_2pi(atan2(t.y, t.x))*4./M_PI);
    // we want next pixel in main direction plus its two neighbours
    OptimalNeighbour(x, y, ScaleIntAngle_0_8(i-1), ScaleIntAngle_0_8(i+1),
        x_o, y_o, d_o);
    if(x_o == x_s && y_o == y_s)
      break;
    if(cnt++ > 1000)
    {
      //printf("*** endless loop!\n");
      break;
    }
    else
    {
      contour.PushBack(ContourPoint(x_o, y_o, d_o));
      x = x_o;
      y = y_o;
    }
  }
}

/**
 * @brief Fill contour string
 */
void Ellipse::FillContourString()
{
  int nedgels = 0;
  for(unsigned i = 0; i < contour.Size(); i++)
  {
    if(FormSegments::edge_img->IsInside(contour[i].x, contour[i].y))
      if(FormSegments::edge_img->Occupied(contour[i].x, contour[i].y))
      {
        contour[i].dir = FormSegments::dir_img[
          FormSegments::edge_img->width*contour[i].y + contour[i].x];
        contour[i].id = FormSegments::edge_img->Pixel(contour[i].x,
            contour[i].y);
        contour[i].supports = true;
        nedgels++;
      }
  }
}

}

