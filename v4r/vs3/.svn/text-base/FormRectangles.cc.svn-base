/**
 * @file FormRectangle.cc
 * @author Richtsfeld Andreas
 * @date October 2009
 * @version 0.1
 * @brief Gestalt principle class for forming rectangles.
 **/

#include <stdio.h>
#include "Rectangle.hh"
#include "Closure.hh"
#include "Line.hh"
#include "FormRectangles.hh"
#include "Vector.hh"

namespace Z
{

static int CmpRectangles(const void *a, const void *b)
{
  if( (*(Rectangle**)a)->sig > (*(Rectangle**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Add edgels from a segment edgel list to an OpenCV point array.
 * @param edgels Edgel array
 * @param num_edgels Number of added edgels.
 * @param new_edgels New edgel array.
 * @param start_idx Index of start point.
 * @param end_idx Index of end point.
 */
static void AddEdgels(CvPoint2D32f *edgels, int &num_edgels, const Array<Edgel> &new_edgels, unsigned start_idx, unsigned end_idx)
{
  for(unsigned i = start_idx; i <= end_idx; i++)
    edgels[num_edgels++] = cvPoint2D32f(new_edgels[i].p.x, new_edgels[i].p.y);
}


bool FormRectangles::NeedsOperate()
{
  return false;
}

/**
 * @brief Constructor of class FormRectangles
 * @param vc Vision core
 */
FormRectangles::FormRectangles(VisionCore *vc) : GestaltPrinciple(vc)
{}

/** TODO ARI: Diese Funktion funktioniert nicht? => wieso????
 * @brief Comparison function for sorting inner angles of a closure, smallerst to largest.
 * @param a ???
 * @param b ???
 */
static int CmpAngles(const void *a, const void *b)
{
  // if a is undefined, move it to end
  if(*(LJunction**)a == 0)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(LJunction**)b == 0)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( (*(LJunction**)a)->OpeningAngle() <
           (*(LJunction**)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Rank Rectangles
 */
void FormRectangles::Rank()
{
  RankGestalts(Gestalt::RECTANGLE, CmpRectangles);
}


/**
 * @brief Making Rectangles
 */
void FormRectangles::Mask()
{
  for(unsigned i=0; i<NumRectangles(core); i++)
  {
    for(unsigned j=0; j<NumRectangles(core); j++)
    {
      if(!Rectangles(core, i)->IsMasked() && !Rectangles(core, j)->IsMasked())
	if(Rectangles(core, i)->sig < Rectangles(core, j)->sig)
	  if(Rectangles(core, i)->IsInside(j))
	    Rectangles(core, i)->Mask(j);	
    }
  }
}


/** 
 * @brief InformNewGestalt
 * @param type Type of Gestalt
 * @param idx Index of the new Gestalt.
 */
void FormRectangles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  if (type == Gestalt::CLOSURE)
    CreateQuadrilateral(idx);
  Rank();
  Mask();
  StopRunTime();
}


/**
 * @brief Try to create a rectangle from a closure.
 * @param clos ID of new closure.
 */
void FormRectangles::CreateQuadrilateral(unsigned clos)
{
  if(Closures(core, clos)->NumLJunctions() == 4)
  {
    CreateWithFourLJ(clos);
  }
  else if (Closures(core, clos)->NumLJunctions() > 4)
  {
    if(!CreateWithMoreLJLine(clos))
      CreateWithMoreLJAngle(clos);
  }
}

/**
 * @brief Try to create a rectangles from closures with exactly four L-Junctions.
 * @param clos ID of closure
 */
void FormRectangles::CreateWithFourLJ(unsigned clos)
{
  Vector2 isct[4];
  for(unsigned i = 0, j = 0; i < Closures(core, clos)->jcts.Size(); i++)
  {
    LJunction *lj = Closures(core, clos)->jcts[i];
    if(Closures(core, clos)->jcts[i] != 0)                    // Note => Zero holes in jcts[]
      isct[j++] = lj->isct;
  }

  if (IsConvexPolygon(isct))
  {
    double parallelity = IsRectangle(isct);
    if(parallelity > 0.5)										// TODO ARI: parallelity-threshold
    {
      Rectangle *new_r = new Rectangle(core, Closures(core, clos), isct, parallelity);
      core->NewGestalt(GestaltPrinciple::FORM_RECTANGLES, new_r);
    }
  }
}


/**
 * @brief Try to create a rectangle, if closure has more than four L-Junctions. \n
 * We try to remove small lines to get the rectangle. This is only solveable with thresholds. \n
 * Copy of StereoBase::TmpSurf::Init() => See for comments.
 * @param clos ID of new closure
 */
bool FormRectangles::CreateWithMoreLJLine(unsigned clos)
{
  CvPoint2D32f edgels[10000];
  int num_edgels = 0;
  vector<TempLine> lines;
  unsigned first_l_jct = 0, i = 0;
  bool full_round = false;

  // move to the first L-jct
  while(i < Closures(core, clos)->jcts.Size() && Closures(core, clos)->jcts[i] == 0)
    i++;

  // note: in case clos is a circle, we have only collinearities!
  if(i == Closures(core, clos)->jcts.Size())
    return false;
  first_l_jct = i;

  while(!full_round)
  {
    // add edgels of RIGHT line of L-jct i, i.e. line i
    VisibleLine *line = (VisibleLine*)Closures(core, clos)->lines[i];
    AddEdgels(edgels, num_edgels, line->seg->edgels, line->idx[START], line->idx[END]);
    i = Closures(core, clos)->jcts.CircularNext(i);

    // if we have reached the next L-jct, our "straight" line is complete: fit line to edgels
    if(Closures(core, clos)->jcts[i] != 0)
    {
      float line_params[4];
      CvMat tmp = cvMat(num_edgels, 1, CV_32FC2, edgels);
      cvFitLine(&tmp, CV_DIST_L2, 0, 0.01, 0.01, line_params);
      lines.push_back(TempLine(line_params[2], line_params[3], line_params[0], line_params[1]));
      num_edgels = 0;
    }

    // if we have come round
    if(i == first_l_jct)
      full_round = true;
  }

  if(lines.size() < 3)
    return false;

  // minimum ratio of length to maximum length
  vector<Vector2> points(lines.size());
  vector<double> lengths(lines.size());
  bool done = false;
  while(!done)
  {
    double length_max = 0.;
    bool erased_short_line = false;
    for(unsigned i = 0; i < lines.size(); i++)
    {
      // line i-1 -> point i -> line i
      unsigned j = (i != 0 ? i - 1 : lines.size() - 1);
      points[i] = LineIntersection(lines[j].p, lines[j].d, lines[i].p, lines[i].d);
    }
    for(unsigned i = 0; i < lines.size(); i++)
    {
      // point i -> line i -> point i+1
      unsigned j = (i < lines.size() - 1 ? i + 1 : 0);
      lengths[i] = Distance(points[i], points[j]);
      length_max = max(length_max, lengths[i]);
    }
    for(unsigned i = 0; i < lines.size() && !erased_short_line; i++)
    {
      // don't erase if only 3 lines left
      if(lengths[i] < length_max*LENGTH_THR_FACTOR && lines.size() > 3)
      {
	lines.erase(lines.begin() + i);
	erased_short_line = true;
      }
    }
    done = !erased_short_line;
  }

  std::vector<Vector2> p;  // intersection (corner) points
  p.resize(lines.size());
  for(i = 0; i < lines.size(); i++)
  {
    unsigned j = (i < lines.size() - 1 ? i + 1 : 0);
    p[i] = LineIntersection(lines[i].p, lines[i].d, lines[j].p, lines[j].d);
  }

  if(p.size() == 4)
  {
    Vector2 isct[4]; 
    for(unsigned i=0; i<4; i++)
      isct[i] = p[i];

    double parallelity = IsRectangle(isct);
    if(parallelity > MIN_PARALLELITY)
    {
      Rectangle *new_r = new Rectangle(core, Closures(core, clos), isct, parallelity);
      core->NewGestalt(GestaltPrinciple::FORM_RECTANGLES, new_r);
      return true;
    }
  }
  return false;
}


/**
 * @brief Try to create a rectangle, if closure has more than four L-Junctions. \n
 * Find four L-junctions with a sum of angles with about 2PI (+-delta). This is not \n
 * logical grounded. Only a method to increase the efficiency.
 * @param clos ID of new closure
 */
bool FormRectangles::CreateWithMoreLJAngle(unsigned clos)
{
  const double delta = M_PI/4.;  											// TODO Another threshold 
  double sum_angles = 0.;

  Array<unsigned> ordered_ljcts;
  for(unsigned i=0; i<Closures(core, clos)->jcts.Size(); i++)
    if(Closures(core, clos)->jcts[i] != 0)			// "without holes ;-)"
      ordered_ljcts.PushBack(Closures(core, clos)->jcts[i]->ID());

  if (IsConvexPolygon(ordered_ljcts))
  {
//       ordered_ljcts.Sort(CmpAngles);		// sort by angles
    // TODO Search the biggest four ordered LJunctions (by hand, because sort-function don't want work.
    for(unsigned a=0; a<4; a++)
    {
      unsigned biggest = 0;
      double comp = 0.0;
      for(unsigned b=a; b<ordered_ljcts.Size(); b++)
      {
	if(LJunctions(core, ordered_ljcts[b])->OpeningAngle() > comp)
	{
	  comp = LJunctions(core, ordered_ljcts[b])->OpeningAngle();
	  biggest = b;
	}
      }
      unsigned sav = ordered_ljcts[a];
      ordered_ljcts[a] = ordered_ljcts[biggest];
      ordered_ljcts[biggest] = sav;
    }

    // calculate sum of the 4 greatest angles
    for(unsigned i = 0; i < 4; i++)
	    if(ordered_ljcts[i] != 0)
		    sum_angles += LJunctions(core, ordered_ljcts[i])->OpeningAngle();

    if(fabs(2*M_PI - sum_angles) <= delta)
    {
      // go throug junction-list and reorder the four junctions clockwise
      unsigned ljcts[4];
      for(unsigned i = 0, j = 0; i < Closures(core, clos)->jcts.Size() && j < 4; i++)
      {
	// if junction i is among the first 4
	// TODO: note that this is a bit inefficient..
	// TODO: sometimes j can become > 3 (hence the check in for(..)
	if(Closures(core, clos)->jcts[i] != 0)					// Note: Be carefull: Zero-Holes in jcts[]
	  if(ordered_ljcts.Find(Closures(core, clos)->jcts[i]->ID()) < 4)
	    ljcts[j++] = Closures(core, clos)->jcts[i]->ID();
      }

      Vector2 isct[4]; 
      for(unsigned i=0; i<4; i++)
	isct[i] = LJunctions(core, ljcts[i])->isct;

      double parallelity = IsRectangle(isct);
      if(parallelity > MIN_PARALLELITY) 
      {
printf("FormRectangles::CreateWithMoreLJAngle: Hier wurde ein Rechteck erzeugt (Angle ding)!\n");
	Rectangle *new_r = new Rectangle(core, Closures(core, clos), isct, parallelity);
	core->NewGestalt(GestaltPrinciple::FORM_RECTANGLES, new_r);
	return true;
      }
    }
  }
  return false;
}


/**
 * @brief Check if quadrilateral is a rectangle. Use therefore the parallelity of opposing edges.\n
 * We assume that we have a nearly perfect one-point projection. This means that at least one opposing \n
 * edge pair should be parallel.
 * @param ljcts The four L-Junctions of the rectangle.
 * @return Returns significance value for beeing a rectangle.
 */
double FormRectangles::IsRectangle(Vector2 isct[4])
{
  Vector2 line[4];      // lines between intersections
  Vector2 dir[4];       // direction of lines
  double phi[4];        // angle of lines

  // calc lines, direction, angle
  for(int i=0; i<4; i++)
  {
    int j=i+1;
    if (j==4) j=0;
    line[i].x=isct[i].x-isct[j].x;
    line[i].y=isct[i].y-isct[j].y;
    if (line[i].y!=0.) dir[i] = Normalise(line[i]);
    phi[i]= ScaleAngle_0_2pi(PolarAngle(dir[i]));
  }

  // calculate difference of angles from opposed edges
  double diff[2];
  diff[0] = fabs(fabs(phi[0]-phi[2])-M_PI);
  diff[1] = fabs(fabs(phi[1]-phi[3])-M_PI);

  // TODO ARI: check this calculation of parallelity (0-1)
  // The better opposing edge-pair counts 0.9, the worser one only 0.1 times.
  // calculate parallelity of best (min) opposing edge-pair (in degree): parallelity = 0.9*e^(-diff/20°)
  double parallelity = 0.9*exp(0.-((fmin(diff[0], diff[1]))/0.349));
  // calculate parallelity of worst (max) opposing edge-pair (in degree): parallelity = 0.1*e^(-diff/20°)
  parallelity += 0.1*exp(0.-((fmax(diff[0], diff[1]))/0.349));

  return parallelity;
}	


/**
 * @brief Check whether the set of intersections form a convex polygon.
 * If the cross products of vectors i-j and j-k for all i, j=i+1, k=j+1 has the
 * same sign, the polygon is convex.
 * @param isct The four intersection points of the rectangle.
 * @return Returns true, if the intersection points build a convex polygon.
 */
bool FormRectangles::IsConvexPolygon(Vector2 isct[4])
{
  Vector2 a, b;
  int i, j, k;
  int sig = 0;

  for(i = 0; i < 4; i++)
  {
    j = (i < 3 ? i + 1 : 0);
    k = (j < 3 ? j + 1 : 0);
    a = isct[j] - isct[i];
    b = isct[k] - isct[j];
    if(sig == 0)
      sig = Sign(Cross(a, b));
    else
    {
      if(sig != Sign(Cross(a, b)))
      	return false;
    }
  }
  return true;
}

/**
 * @brief ARI: Check whether the set of L-Junctions form a convex polygon.
 * If the cross products of neighbor-vectors have all the same sign, the
 * polygon is convex.
 * @param jcts The L-junctions of the closure as array.
 */
bool FormRectangles::IsConvexPolygon(Array<unsigned> ljcts)
{
  Vector2 a, b;	
  int i, j, k;
  int sig = 0;	// sign of cross product
  int m = 0;	// number of found LJunctions
	
  Vector2 intscts[ljcts.Size()];		// maximum LJunctions->isct Array

  // get every junction and check if LJunction (could also be a coll.)
  for (unsigned l=0; l<ljcts.Size(); l++)
  {
    try
    {
      LJunctions(core, ljcts[l])->isct;							/// TODO Ist das hier Blödsinn? Wieso try-catch block?
      intscts[m++] = LJunctions(core, ljcts[l])->isct;
    }
    catch (exception &e){}
  }	
	
  // copy intersections to array with size m
  Vector2 isct[m];
  for (int n=0; n<m; n++)
    isct[n] = intscts[n];
	
  // check the sign of the cross product between neighboring lines
  for(i=0; i<m; i++)
  {
    j = (i < (m-1) ? i + 1 : 0);
    k = (j < (m-1) ? j + 1 : 0);
    a = isct[j] - isct[i];
    b = isct[k] - isct[j];

    if(sig == 0)
      sig = Sign(Cross(a, b));
    else
    {
      if(sig != Sign(Cross(a, b)))
        return false;
    }
  }
  return true;
}

}
