/**
 * @file FormArcJunctions.cc
 * @author Zillich, Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt principle FormArcJunctions.
 **/

#include "Arc.hh"
#include "AJunction.hh"
#include "FormArcJunctions.hh"

namespace Z
{

/**
 * @brief Constructor of class FormArcJunctions.
 * @param vc Vision core
 * @param vi Vote image
 */
FormArcJunctions::FormArcJunctions(VisionCore *vc) : GestaltPrinciple(vc)
{}


/**																							/// TODO Delete, when copied to vote_img
 * @brief Operate incremental.
 */
void FormArcJunctions::OperateIncremental()
{
//   if(first_op)
//   {
// 		// in first call just init search lines, don't create junctions
//     vote_img->SetNumLines(NumArcs(core)*8);
//     for(unsigned r = 0; r < NumArcs(core); r++)
//       InitSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
//     first_op = false;
//   }
//   else
//   {
//     // for all subsequent calls 
//     // try to be smart about growing search lines
//     if(grow_method == GROW_SMART)
// 		{
// 			printf("FormArcJunctions::OperateIncremental: grow_smart not yet implemented!\n");
//       // HACK: not implemented yet
//       //int r = ExpSelect(NumArcs(core) - 1);
//       //ExtendSmartLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
//     }
//     // lines grow probabilistically according to length
//     else if(grow_method == GROW_WEIGHTED)
//     {
//       int r = ExpSelect(NumArcs(core) - 1);
//       ExtendSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
//     }
//     // all search lines grow equally, this is less "anytime-ish"
//     else if(grow_method == GROW_EQUAL)
//     {
//       for(unsigned r = 0; r < NumArcs(core); r++)
//         ExtendSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
//     }
//   }
}


/**
 * @brief Create a new junction from the delivered intersections
 * @param sline is the search line which triggered the new intersections, it is
 * therefore a tangent or normal vote line.
 */
void FormArcJunctions::CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts)
{
	unsigned baseIndex = core->VI()->GetBaseIndex();
	unsigned vtype_i = sline%baseIndex;
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned vtype_j = iscts[k].id%baseIndex;

// printf("Found intersection between: %u - %u with sl: %u - %u @ end: %u - %u\n", i, j, vtype_i, vtype_j, end_i, end_j);

    // check if the intersection between vote line tyes is admissible
// printf(" vtype i, j: %u, %u\n", vtype_i, vtype_j);
    if(core->VI()->IsctTypeAdmissible(vtype_i, vtype_j) == 2)			// 2 == arc junctions!
    {
      bool ok = true;
      unsigned i = sline/baseIndex - core->VI()->GetArcOffset();
      unsigned j = iscts[k].id/baseIndex - core->VI()->GetArcOffset();
// printf(" Create junctions 2\n");
      int end_i = VOTE_END(vtype_i);
      int end_j = VOTE_END(vtype_j);
// printf(" Nr.OfLines: %u\n", NumLines(core));
// printf(" Create junctions 3: %u %u\n", i, j);
      Arc *arc_i = Arcs(core, i);
      Arc *arc_j = Arcs(core, j);
// printf(" FormArcJunctions::CreateJunctions: is A-JUNCTION: arcs: %u-%u\n", arc_i->ID(), arc_j->ID());
// printf(" Create junctions 4\n");
// printf("FormArcJunctions::CreateJunctions: Intersection valid!\n");
      // if a tangent hits a normal searchline "from behind", ignore
      // first check if one of the search lines is a normal
      if(VOTE_IS_NORMAL(vtype_i) || VOTE_IS_NORMAL(vtype_j)) 
      {
        Vector2 tang_i, tang_j;
        if(VOTE_END(end_i) == START)
          tang_i = arc_i->norm[end_i].NormalAntiClockwise(); //NormalAntiClockwise();
        else
          tang_i = arc_i->norm[end_i].NormalClockwise();
        if(VOTE_END(end_j) == START)
          tang_j = arc_j->norm[end_j].NormalAntiClockwise();
        else
          tang_j = arc_j->norm[end_j].NormalClockwise();
        // if tangents at respective ends point in the same direction
        // then normal search lines will be intersected "from behind"
        if(Dot(tang_i, tang_j) > 0.)
          ok = false;
      }
//       if(!ok)
//         printf("FormArcJunctions: Arc is not ok!\n");
//       if(arc_i->Convexity(arc_j) < 0.5)
//         printf("FormArcJunctions: Too low convexity: %4.4f\n", arc_i->Convexity(arc_j));
      if(ok)
        if(!IsJunctionBetween(arc_i, arc_j))
          //if(arc_i->ConvexWith(arc_j))
          // HACK: 0.5 is an arbitrary threshold!
          if(arc_i->Convexity(arc_j) >= 0.5)
            core->NewGestalt(GestaltPrinciple::FORM_ARC_JUNCTIONS, new AJunction(core, arc_i, arc_j, end_i, end_j));
    }
  }
// printf("  arc-jcts: create junctions end\n");
}

/**
 * @brief Checks, if there is already a A-junction in between.
 * @param arc_i First arc
 * @param arc_j Second arc
 * @return Returns true, if there is already a junction in between.
 */
bool FormArcJunctions::IsJunctionBetween(Arc *arc_i, Arc *arc_j)
{
  for(int end = START; end <= END; end++)
    for(unsigned k = 0; k < arc_i->jct[end].Size(); k++)
      if(arc_i->jct[end][k]->arc[end] == arc_j)
        return true;
  return false;
}

}

