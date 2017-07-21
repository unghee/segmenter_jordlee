/**
 * @file ConvexArcGroup.cc
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Class file of Gestalt ConvexArcGroup.
 */

#include <cstdio>

#include "Draw.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"

namespace Z
{

/**
 * @brief Calculate
 * @param arcs Array of arcs
 * @param l
 * @param u
 * @param p
 * @return Returns true, if ... TODO
 */
static bool Inside(Array<Arc*> &arcs, unsigned l, unsigned u, Vector2 &p)
{
  for(unsigned i = l; i <= u; i++)
    if(!arcs[i]->HasInside(p))
      return false;
  return true;
}

/**
 * @brief Constructor of class ConvexArcGroup.
 * @param c Vision core
 */
ConvexArcGroup::ConvexArcGroup(VisionCore *vc) : Gestalt(vc, CONVEX_ARC_GROUP)
{}

/**
 * @brief Constructor of class ConvexArcGroup.
 * @param c Vision core
 * @param a Arcs array
 * @param l 
 * @param u
 * @param s TODO
 */
ConvexArcGroup::ConvexArcGroup(VisionCore *vc, Array<Arc*> &a, unsigned l, unsigned u, double s)
  : Gestalt(vc, CONVEX_ARC_GROUP)
{
  if(u >= l)
  {
    arcs.Resize(u - l + 1);
    for(unsigned i = 0; l <= u; i++, l++)
      arcs[i] = a[l];
  }
  else
	{
		char buffer [100];
		sprintf(buffer, "ConvexArcGroup::ConvexArcGroup: Invalid array bounds %u-%u", l, u);

    throw(std::runtime_error(buffer));
	}
  ang_cover = ArcGroupAngularCoverage(arcs);
  closedness = ArcGroupClosedness(arcs);
  if(s >= 0.)
    sig = s;
  else
    CalculateSignificance();
}


/**
 * @brief Compare the centers of the convex arc group
 */
static Vector2 cmp_mean;																/// TODO Static variable neccessary?
static int CmpCenters(const void *a, const void *b)
{
  if(Distance(cmp_mean, *((Vector2*)a)) < Distance(cmp_mean, *((Vector2*)b)))
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Draw the convex arc group.
 * If detail >= 1 also draw stuff of the forming gestalt principle.
 */
void ConvexArcGroup::Draw(int detail)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    arcs[i]->Draw(detail - 1);

  if(detail >= 5)
  {
    unsigned i, j, k, n = 2*arcs.Size();
    Vector2 p[n], t[n], c[n];
    Vector2 m1, t1, m2, t2;
    Vector2 mean(0., 0.);
    double weight[n], sum_weight = 0., swd = 0., pp = 1.;
    double point_dens = 2.*NumArcs(core)/(double)core->ImageArea();
    double r, r_max = 0., alpha;
    double prob, s, s_max = -HUGE;
    for(i = 0; i < arcs.Size(); i++)
    {
      p[2*i] = arcs[i]->tang_pt[0];
      t[2*i] = arcs[i]->tang[0];
      p[2*i+1] = arcs[i]->tang_pt[1];
      t[2*i+1] = arcs[i]->tang[1];
    }
    for(i = 0; i < n; i++)
    {
      try
      {
        j = (i + n/3)%n;
        k = (i + (2*n)/3)%n;
        if(i==j || j==k || k==i) break; // not the same lines
        weight[i] = 1.;
        m1 = MidPoint(p[i], p[j]);
        t1 = LineIntersection(p[i], t[i], p[j], t[j]);
        weight[i] *= fabs(Cross(t[i], t[j]));
        //DrawLine2D(p[i].x, p[i].y, p[j].x, p[j].y, RGBColor::white);
        //DrawLine2D(m1.x, m1.y, t1.x, t1.y, RGBColor::yellow);
        t1 = Normalise(t1 - m1);  // now is direction vector
        m2 = MidPoint(p[j], p[k]);
        t2 = LineIntersection(p[j], t[j], p[k], t[k]);
        weight[i] *= fabs(Cross(t[j], t[k]));
        //DrawLine2D(p[j].x, p[j].y, p[k].x, p[k].y, RGBColor::white);
        //DrawLine2D(m2.x, m2.y, t2.x, t2.y, RGBColor::yellow);
        t2 = Normalise(t2 - m2);
        c[i] = LineIntersection(m1, t1, m2, t2);
        weight[i] *= fabs(Cross(t1, t2));
        DrawPoint2D(c[i].x, c[i].y, RGBColor::magenta);
        mean += c[i]*weight[i];
        sum_weight += weight[i];
      }
      catch (exception &e)
      {
//         printf("ConvexArcGroup::Draw: Exception during processing.\n");
//         cout << e.what() << endl;
      }
    }
    if(sum_weight > 0.)
    {
      mean /= sum_weight;
      for(i = 0; i < n; i++)
        swd += Distance(mean, c[i])*weight[i];
      swd /= sum_weight;
      for(i = 0; i < n; i++)
        pp *= (1. - exp(-CircleArea(fmax(Distance(mean, c[i]),1.)*point_dens)));
      // proper Poisson formalism
      // sort points according to distance
      cmp_mean = mean;
      qsort(c, n, sizeof(Vector2), CmpCenters);
      for(i = 0; i < n; i++)
      {
        r = fmax(Distance(mean, c[i]), 1.);
        alpha = CircleArea(r)*point_dens;
        // TODO: shouln't it be PoissonCDF(i-1, alpha)?
        prob = PoissonCDF(i, alpha);
        s = -log(1. - prob);
        if(s > s_max)
        {
          s_max = s;
          r_max = r;
        }
      }
      //FillRect2D(mean.x-2, mean.y-2, mean.x+2, mean.y+2, RGBColor::blue);
      TransparentArc2D(mean.x, mean.y, r_max, 0, 2*M_PI, RGBColor::blue);
    }
  }
}

/**
 * @brief Get information about the convex arc group as string.
 * @return Returns information about the convex arc group as string.
 */
const char* ConvexArcGroup::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
	int n = 0;
	n += snprintf(info_text, info_size,
      "%s  %d arcs: ",
      Gestalt::GetInfo(), arcs.Size());

	for (unsigned i=0; i<arcs.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", arcs[i]->ID());
			
	n += snprintf(info_text + n, info_size - n, "\n  coverage: %f (gap: %f)\n  closedness: %f",
								ang_cover, ArcGroupLargestAngularGap(arcs), closedness);
			
  return info_text;
}

/**
 * @brief Check, if Gestalt is at position x,y
 * @param x x-coordinate in pixel
 * @param y y-coordinate in pixel
 * @return Returns true, if Gestalt is at position x,y
 */
bool ConvexArcGroup::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    if(arcs[i]->IsAtPosition(x, y))
      return true;
  return false;
}

/**
 * @brief Check, if convex arc group already contains the arc.
 * @return Returns true, if ConvexArcGroup contains already the arc.
 */
bool ConvexArcGroup::HasArc(Arc *arc)
{
  return arcs.Contains(arc);
}

/**
 * @brief Returns true if the group is still convex when the given arc is added.
 * TODO: When appending an arc it is sufficient to check for convecity with the
 * first arc of the group. When prepending, with the last arc of the group.
 * @param arc Arc
 */
bool ConvexArcGroup::StillConvex(Arc *arc)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    if(!arcs[i]->ConvexWith(arc))
      return false;
  return true;
}

/**
 * @brief Calculate significance of the Gestalt.
 */
void ConvexArcGroup::CalculateSignificance()
{
  sig = ArcGroupSignificance(core, arcs, 0, arcs.Size() - 1);
}

/**
 * @brief Calculate arc group significance.
 * TODO: significance of a single arc should be > 0. if arc coverage is large
 * enough
 */
double ArcGroupSignificance(VisionCore *core, Array<Arc*> &arcs, unsigned l, unsigned u)
{
  if(u - l > 0)
  {
    unsigned i, j, k, n = 2*(u - l + 1);
    Vector2 p[n], t[n], c[n];
    Vector2 m1, t1, m2, t2;
    Vector2 mean(0., 0.);
    double weight[n], sum_weight = 0., r, alpha, sig_max = -HUGE;
    // estimation of center point density = num. points/area
    // Note: 2*NumArcs() is just an estimate of the number of center points
    // which however comes remarkably close to the true number (which can be
    // calculated once all groups have been created: e.g.
    // true: 3114, estimated: 2898;  true: 5678, estimated: 5492)
    double point_dens = 2.*NumArcs(core)/(double)core->ImageArea();

    for(i = l, j = 0; i <= u; i++, j++)
    {
      p[2*j] = arcs[i]->tang_pt[0];
      t[2*j] = arcs[i]->tang[0];
      p[2*j+1] = arcs[i]->tang_pt[1];
      t[2*j+1] = arcs[i]->tang[1];
    }
    for(i = 0; i < n; i++)
    {
      try
      {
        j = (i + n/3)%n;
        k = (i + (2*n)/3)%n;
        weight[i] = 1.;
        m1 = MidPoint(p[i], p[j]);
        t1 = LineIntersection(p[i], t[i], p[j], t[j]);
        weight[i] *= fabs(Cross(t[i], t[j]));
        t1 = Normalise(t1 - m1);  // now is direction vector
        m2 = MidPoint(p[j], p[k]);
        t2 = LineIntersection(p[j], t[j], p[k], t[k]);
        weight[i] *= fabs(Cross(t[j], t[k]));
        t2 = Normalise(t2 - m2);
        c[i] = LineIntersection(m1, t1, m2, t2);
        weight[i] *= fabs(Cross(t1, t2));
        mean += c[i]*weight[i];
        sum_weight += weight[i];
      }
      catch (exception &e)
      {
        // lines did not intersect: ignore this point
        c[i] = Vector2(HUGE/2., HUGE/2.);
        weight[i] = 0.;
        // leave mean and sum_weight untouched

        //printf("ConvexArcGroup::ArcGroupSignificance: unknown exception during processing of images.\n");
        //cout << e.what() << endl;
      }
    }
    if(sum_weight > 0.)
    {
      mean /= sum_weight;
      // sort points according to distance
      cmp_mean = mean;
      qsort(c, n, sizeof(Vector2), CmpCenters);
      for(i = 0; i < n; i++)
      {
        r = fmax(Distance(mean, c[i]), 1.);
        alpha = CircleArea(r)*point_dens;
        // to be precise: we want PoissonCDF(i-1,alpha) for i=1..n
        // which is the same as PoissonCDF(i,alpha) for i=0..n-1
        sig_max = fmax(sig_max, -log(1. - PoissonCDF(i, alpha)));
      }
      return sig_max;
    }
    else
      return 0.;
  }
  else
    return 0.;
}

/**
 * @brief Get information about the convex arc group as string.
 * @param arcs Array of arc group arcs
 * @return Returns the gap, between the arcs in pixel.
 */
double ArcGroupClosedness(Array<Arc*> &arcs)
{
  unsigned i, j;
  double sum_len = 0., sum_gap = 0.;
  for(i = 0; i < arcs.Size(); i++)
  {
    j = (i != arcs.Size() - 1 ? i + 1 : 0);
    sum_len += arcs[i]->ArcLength();
    sum_gap += Distance(arcs[i]->point[END], arcs[j]->point[START]);
  }
  return sum_len/(sum_len + sum_gap);
}

/**
 * @brief Calculates the angular coverage (= total angular span) of a group of arcs.
 * The angular span is just two pi minus the largest gap in the group of arcs.
 * Arcs must be sorted in ascending start angle order.
 * @param arcs Array of arc group arcs
 * @return Returns the coverage in pixel.
 */
double ArcGroupAngularCoverage(Array<Arc*> &arcs)
{
  return 2*M_PI - ArcGroupLargestAngularGap(arcs);
}

/**
 * @brief Calculates the largest angular gap in the group of arcs.
 * Arcs must be sorted in ascending start angle order.
 * @param arcs Array of arc group arcs
 * @return Returns the angular gap.
 */
double ArcGroupLargestAngularGap(Array<Arc*> &arcs)
{
  if(arcs.Size() == 1)
  {
    return 2*M_PI - arcs[0]->angular_span;
  }
  else
  {
    double max_gap = 0., es, ss;
    unsigned j;
    // Note that since arcs are in ascending order, the differences
    // between end[i-1] and start[i] are always positive -> differences are
    // unique.
    // There might however be a slight overlap between end[i-1] and start[i]
    // (note that the arcs are only - sometimes coarse - approximations!).
    // If so start[i] - end[i-1] would be larger (e.g. 350 deg) than
    // start[i] - start[i-1]. Overlaps are treated as zero gaps and ignored.
    // Note: there are pathologic cases when even start[i] - start[i-1] is
    // "negative", e.g. 355 deg. Can happen for very flat parts of an ellipse.
    // In that case the daisy chain order of linking the arcs does not
    // correspond to an ascending start angle order.
    for(unsigned i = 0; i < arcs.Size(); i++)
    {
      j = (i == 0 ? arcs.Size() - 1 : i - 1);
      es = DiffAngle_0_2pi(arcs[i]->start_angle,
          arcs[j]->end_angle);
      ss = DiffAngle_0_2pi(arcs[i]->start_angle,
          arcs[j]->start_angle);
      // if no overlap
      //if(ss < M_PI && es < ss) // TODO: why did I require ss < M_PI?
      if(es < ss)
        max_gap = Max(max_gap, es);
    }
    return max_gap;
  }
}

/**
 * @brief Calculate arc group area
 * TODO Not yet implemented
 * @param arcs Array of arc group arcs
 * @return Returns the area of the arc group.
 */
double ArcGroupArea(Array<Arc*> &arcs)
{
	printf("ConvexArcGroup: ArcGroupArea: Not yet implemented!\n");
  return 0.;
}

/**
 * @brief Calculate the group gap area ratio.
 * @param arcs Array of arc group arcs
 * @return Returns the arc group gap area ratio.
 */
double ArcGroupGapAreaRatio(Array<Arc*> &arcs)
{
  unsigned i, j;
  double sum_gap = 0.;
  for(i = 0; i < arcs.Size(); i++)
  {
    j = (i != arcs.Size() - 1 ? i + 1 : 0);
    sum_gap += Distance(arcs[i]->point[END], arcs[j]->point[START]);
  }
  return sum_gap/ArcGroupArea(arcs);
}

}

