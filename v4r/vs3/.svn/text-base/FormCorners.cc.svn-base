/**
 * @file FormCorners.cc
 * @author Andreas Richtsfeld
 * @date September 2010
 * @version 0.1
 * @brief Gestalt principle class to create corners.
 */

#include "FormCorners.hh"

namespace Z
{

static int CmpCorners(const void *a, const void *b)
{
  if( (*(Corner**)a)->sig > (*(Corner**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Rank corners.
 */
void FormCorners::Rank()
{
  // TODO ARI: 
  RankGestalts(Gestalt::CORNER, CmpCorners);
}

/**
 * @brief Constructor of class FormCorners
 * @param vc Vision core
 */
FormCorners::FormCorners(VisionCore *vc) : GestaltPrinciple(vc)
{}

/**
 * @brief InformNewGestalt: receive new Gestalts from vision core.
 * @param type Gestalt type.
 * @param idx Index of new Gestalt.
 */
void FormCorners::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      Create(idx);
      break;
    default:
      break;
  }
  Rank();
  StopRunTime();
}

/**
 * @brief Create a new corner, when a new L-Junction appears.
 * @param idx Index of the new L-junction.
 *
 */
void FormCorners::Create(unsigned idx)
{
  // Get l-junction
  LJunction *lj = LJunctions(core, idx);
  
  Array<LJunction*> ljcts;
  Array<Line*> lines;
  Array<unsigned> near_points;
	  
  // FIRST LINE
  // are there on the line ends other l-junctions?
  if((lj->line[0]->l_jct[lj->near_point[0]][0].Size() + lj->line[0]->l_jct[lj->near_point[0]][1].Size()) == 2)
  {
    // get all l-junctions on one lineEnd
    for(unsigned i=0; i<lj->line[0]->l_jct[lj->near_point[0]][0].Size(); i++)
    {
      ljcts.PushBack(lj->line[0]->l_jct[lj->near_point[0]][0][i]);
      if(!lines.Contains(lj->line[0]->l_jct[lj->near_point[0]][0][i]->line[0]))
      {
	lines.PushBack(lj->line[0]->l_jct[lj->near_point[0]][0][i]->line[0]);
	near_points.PushBack(lj->line[0]->l_jct[lj->near_point[0]][0][i]->near_point[0]);
      }
      if(!lines.Contains(lj->line[0]->l_jct[lj->near_point[0]][0][i]->line[1]))
      {
	lines.PushBack(lj->line[0]->l_jct[lj->near_point[0]][0][i]->line[1]);
	near_points.PushBack(lj->line[0]->l_jct[lj->near_point[0]][0][i]->near_point[1]);
      }
    }
    for(unsigned i=0; i<lj->line[0]->l_jct[lj->near_point[0]][1].Size(); i++)
    {
      ljcts.PushBack(lj->line[0]->l_jct[lj->near_point[0]][1][i]);
      if(!lines.Contains(lj->line[0]->l_jct[lj->near_point[0]][1][i]->line[0]))
      {
	lines.PushBack(lj->line[0]->l_jct[lj->near_point[0]][1][i]->line[0]);
	near_points.PushBack(lj->line[0]->l_jct[lj->near_point[0]][1][i]->near_point[0]);
      }
      if(!lines.Contains(lj->line[0]->l_jct[lj->near_point[0]][1][i]->line[1]))
      {
	lines.PushBack(lj->line[0]->l_jct[lj->near_point[0]][1][i]->line[1]);
	near_points.PushBack(lj->line[0]->l_jct[lj->near_point[0]][1][i]->near_point[1]);
      }
    }
    NewCorner(ljcts, lines, near_points);
  }
  
  /// Create corners with more than 3 arms?
// 	if((lj->line[0]->l_jct[lj->near_point[0]][0].Size() + lj->line[0]->l_jct[lj->near_point[0]][1].Size()) > 2)
// 		printf("FormCorners::Create: Candidate for a quad corner found!\n");


  // SECOND LINE
  // are there on the line ends other l-junctions?
  if((lj->line[1]->l_jct[lj->near_point[0]][0].Size() + lj->line[1]->l_jct[lj->near_point[0]][1].Size()) == 2)
  {
    // get all l-junctions on one lineEnd
    for(unsigned i=0; i<lj->line[1]->l_jct[lj->near_point[0]][0].Size(); i++)
    {
      ljcts.PushBack(lj->line[1]->l_jct[lj->near_point[0]][0][i]);
      if(!lines.Contains(lj->line[1]->l_jct[lj->near_point[0]][0][i]->line[0]))
      {
	lines.PushBack(lj->line[1]->l_jct[lj->near_point[0]][0][i]->line[0]);
	near_points.PushBack(lj->line[1]->l_jct[lj->near_point[0]][0][i]->near_point[0]);
      }
      if(!lines.Contains(lj->line[1]->l_jct[lj->near_point[0]][0][i]->line[1]))
      {
	lines.PushBack(lj->line[1]->l_jct[lj->near_point[0]][0][i]->line[1]);
	near_points.PushBack(lj->line[1]->l_jct[lj->near_point[0]][0][i]->near_point[1]);
      }
    }
    for(unsigned i=0; i<lj->line[1]->l_jct[lj->near_point[0]][1].Size(); i++)
    {
      ljcts.PushBack(lj->line[1]->l_jct[lj->near_point[0]][1][i]);
      if(!lines.Contains(lj->line[1]->l_jct[lj->near_point[0]][1][i]->line[0]))
      {
	lines.PushBack(lj->line[1]->l_jct[lj->near_point[0]][1][i]->line[0]);
	near_points.PushBack(lj->line[1]->l_jct[lj->near_point[0]][1][i]->near_point[0]);
      }
      if(!lines.Contains(lj->line[1]->l_jct[lj->near_point[0]][1][i]->line[1]))
      {
	lines.PushBack(lj->line[1]->l_jct[lj->near_point[0]][1][i]->line[1]);
	near_points.PushBack(lj->line[1]->l_jct[lj->near_point[0]][1][i]->near_point[1]);
      }
    }
    NewCorner(ljcts, lines, near_points);
  }
  
  /// Create corners with more than 3 arms?
// 	if((lj->line[1]->l_jct[lj->near_point[0]][0].Size() + lj->line[1]->l_jct[lj->near_point[0]][1].Size()) > 2)
// 		printf("FormCorners::Create: Candidate for a quad corner found!\n");
}


/**
 * @brief Create new corner, if it do not already exist.
 * @param ljcts L-Junctions of new corner
 * @param lines Lines of the new corner
 * @param near_points Near points (line end: START/END) of the lines to junction. 
 */
void FormCorners::NewCorner(Array<LJunction*> ljcts, Array<Line*> lines, Array<unsigned> near_points)
{
  if(CornerExists(lines, near_points) == UNDEF_ID)
    core->NewGestalt(GestaltPrinciple::FORM_CORNERS, new Corner(core, ljcts, lines, near_points));
// else
//   printf("FormCorners::NewCorner: Corner already exists!\n");
}

/**
 * @brief Checks, if the corner with the same lines already exists.
 * @param lines Lines of the corner to check.
 * @param near_points Near points (ends) of the lines.
 * @return Returns the number of the corner.
 * It is enough to check only the corners on one line.
 */
unsigned FormCorners::CornerExists(Array<Line*> lines, Array<unsigned> near_points)
{
  // get all corners from the first line end!
  bool isNewCorner = false;
  Array<Corner*> corners;
  for(unsigned i=0; i<lines[0]->corners[near_points[0]].Size(); i++)
    corners.PushBack(lines[0]->corners[near_points[0]][i]);
  
  // go through the corners and try to find the same one
  for(unsigned i=0; i<corners.Size(); i++)
  {
    // compare lines of corners
    bool equal = true;
    for(unsigned j=0; j<corners[i]->lines.Size(); j++)
    {
      if(!lines.Contains(corners[i]->lines[j])) 
      {
	equal = false;
      }
    }
    if(equal) return corners[i]->ID();
  }
  return UNDEF_ID;
}

}
