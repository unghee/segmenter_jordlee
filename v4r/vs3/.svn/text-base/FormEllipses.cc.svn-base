/**
 * @file FormEllipses.cc
 * @author Andreas Richtsfeld
 * @date December 2009
 * @version 0.1
 * @brief Form ellipses from formed convex arc groups.
 */

#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "Ellipse.hh"
#include "FormArcs.hh"
#include "FormEllipses.hh"

namespace Z
{

static int CmpEllipses(const void *a, const void *b)
{
  if( (*(Ellipse**)a)->sig > (*(Ellipse**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Constructor of class FormEllipses.
 */
FormEllipses::FormEllipses(VisionCore *vc) : GestaltPrinciple(vc)
{}

/**
 * @brief Inform new Gestalt.
 * @param type type of the new Gestalt.
 * @param idx index of the new Gestalt.
 */
void FormEllipses::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
	
  switch(type)
  {
    case Gestalt::CONVEX_ARC_GROUP:
      HaveNewArcGroup(idx);
      break;
    default:
      break;
  }
  
  StopRunTime();
}

/**
 * @brief Process new received arc group.
 * @param idx index of the arc group
 */
void FormEllipses::HaveNewArcGroup(unsigned idx)
{
  double x, y, a, b, phi;
  ConvexArcGroup *group = ConvexArcGroups(core, idx);
	
  // HACK: do we really want to reject if angular coverage smaller pi?
//   if(group->ang_cover < M_PI)
//     return;

  // fit ellipse only when 2 different segments are available
  bool different = false;                                             /// TODO Set true, to calculate all ellipses
  for(unsigned i= 0; i<group->arcs.Size()-1; i++) {
    for(unsigned j=i+1; j<group->arcs.Size(); j++) {
//       unsigned j=i+1;
      if(group->arcs[i]->seg->ID() != group->arcs[j]->seg->ID())
        different = true;
    }
  }
  
  if(different) {
    printf("FormEllipses::HaveNewArcGroup: ########################################################### Review this part ###########################################################\n");
    if(FitEllipse(group->arcs, x, y, a, b, phi)) {
      printf("FormEllipses::HaveNewArcGroup: 2 different segments => one new ellipse !!!\n");
      // note: in FormArcs we defined MIN_RADIUS for an arc, thus
      // the smaller axis cannot be smaller than MIN_RADIUS
      if(b > FormArcs::MIN_RADIUS)
        core->NewGestalt(GestaltPrinciple::FORM_ELLIPSES, new Ellipse(core, group, x, y, a, b, phi));
    }
  }
  Rank();
  Mask();
}

/**
 * @brief Rank the Gestalts with respect to the calculated significance.
 */
void FormEllipses::Rank()
{
  RankGestalts(Gestalt::ELLIPSE, CmpEllipses);
}

/**
 * @brief Mask the bad results.
 */
void FormEllipses::Mask()
{
  // array containing for each arc the ID of the highest ranking ellipse which
  // contains that arc
  Array<unsigned> owner(NumArcs(core));
  owner.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumEllipses(core); j++)
  {
    Ellipse *ell = (Ellipse*)core->RankedGestalts(Gestalt::ELLIPSE, j);
    ell->Mask(UNDEF_ID);
    // first check if any previous (i.e. stronger) ellipse uses one of my arcs
    for(unsigned a = 0; a < ell->group->arcs.Size(); a++)
    {
      if(owner[ell->group->arcs[a]->ID()] != UNDEF_ID)
        ell->Mask(owner[ell->group->arcs[a]->ID()]);
      owner[ell->group->arcs[a]->ID()] = ell->ID();
    }
  }
}

}

