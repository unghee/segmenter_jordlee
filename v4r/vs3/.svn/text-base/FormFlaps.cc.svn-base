/**
 * @file FormFlaps.cc
 * @author Michael Zillich, Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Form flaps from closures (flap-areas from polygons)
 */

#include "Array.hh"
#include "VisionCore.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Closure.hh" 
#include "Flap.hh"
#include "FormFlaps.hh"

namespace Z
{

/**
 * @brief Compare function for ranking.
 * @param a Gestalt a
 * @param b Gestalt b
 */
static int CmpFlaps(const void *a, const void *b)
{
  if( (*(Flap**)a)->sig > (*(Flap**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Check if junctions coincide.
 * TODO Unimplemented
 * @param rect1 Index of rectangle 1
 * @param rect2 Index of rectangle 2
 * @param line Index of line
 */
static bool JunctionsCoincide(unsigned rect1, unsigned rect2, unsigned line)
{
  /*unsigned i, i1=UNDEF_ID, i2=UNDEF_ID, j1, j2;
  Vector2 p1, p2;
  const double max_dist = Lines(line)->Length()/10.;  // TODO: nasty threshold
  // find which line of rectangle 1 is the common one
  for(i = 0; i < 4; i++)
    if(Rectangles(rect1)->lines[i] == line)
    {
      i1 = i;
      break;
    }
  for(i = 0; i < 4; i++)
    if(Rectangles(rect2)->lines[i] == line)
    {
      i2 = i;
      break;
    }
  // first junction of line 1 (in counter-clockwise order)
  j1 = (i1 > 0 ? i1 - 1 : 3);
  // must coincide with last junction of line 2 (in c.c.o.)
  j2 = i2;
  p1 = LJunctions(Rectangles(rect1)->jcts[j1])->isct;
  p2 = LJunctions(Rectangles(rect2)->jcts[j2])->isct;
  //printf("1. coin? %u-%u\n", Rectangles(rect1)->jcts[j1],
  //  Rectangles(rect2)->jcts[j2]);
  if(Distance(p1, p2) > max_dist)
    return false;
  //printf("yes\n");
  // last junction of line 1 (in c.c.o)
  j1 = i1;
  // must coincide with first junction of line 2 (in c.c.o)
  j2 = (i2 > 0 ? i2 - 1 : 3);
  p1 = LJunctions(Rectangles(rect1)->jcts[j1])->isct;
  p2 = LJunctions(Rectangles(rect2)->jcts[j2])->isct;
  //printf("2. coin? %u-%u\n", Rectangles(rect1)->jcts[j1],
  //     Rectangles(rect2)->jcts[j2]);
  if(Distance(p1, p2) > max_dist)
    return false;
  //printf("yes\n\n");*/
  return true; 
}


/**
 * @brief Constructor FormFlaps
 * @param core Vision core
 */
FormFlaps::FormFlaps(VisionCore *vc) : GestaltPrinciple(vc)
{
}


/**
 * @brief Inform new Gestalt
 * @param type Type of Gestalt to inform.
 * @param idx Index of new received Gestalt.
 */
void FormFlaps::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
  switch(type)
  {
    case Gestalt::CLOSURE:
      HaveNewClosure(idx);
      break;
    default:
      break;
  }
  StopRunTime();
}

/**
 * @brief StartOfCollinearRun:
 * pos_i is the position of a common line along closure i, pos_j the position
 * of that line along closure j.
 * We assume that the common line is traveresed in different directions along
 * both closures, i.e. closures lie aside.
 * Then if the previous line on i is not the same as the next line on j, that
 * line is the start of a common run of lines (start as seen from closure i).
 */
bool FormFlaps::StartOfCollinearRun(Closure *clos_i, unsigned pos_i, Closure *clos_j, unsigned pos_j)
{
  unsigned pos_ip = clos_i->lines.CircularPrev(pos_i);
  unsigned pos_jn = clos_j->lines.CircularNext(pos_j);
  return clos_i->lines[pos_ip] != clos_j->lines[pos_jn];
}

/**
 * @brief EndOfCollinearRun:
 * pos_i is the position of a common line along closure i, pos_j the position
 * of that line along closure j.
 * We assume that the common line is traveresed in different directions along
 * both closures, i.e. closures lie aside.
 * Then if the next line on i is not the same as the previous line on j, that
 * line is the end of a common run of lines (end as seen from closure i).
 * @param clos_i Closure i
 * @param pos_i Position of i
 * @param clos_j Closure j
 * @param pos_j Position of j
 */
bool FormFlaps::EndOfCollinearRun(Closure *clos_i, unsigned pos_i, Closure *clos_j, unsigned pos_j)
{
  unsigned pos_in = clos_i->lines.CircularNext(pos_i);
  unsigned pos_jp = clos_j->lines.CircularPrev(pos_j);
  return clos_i->lines[pos_in] != clos_j->lines[pos_jp];
}

/**
 * @brief HaveNewClosure:
 * Find all existing closures which share part of the contour, in opposite
 * senses (i.e. contours lie side by side, not one inside the other).
 * @param i Index of new closure
 */
void FormFlaps::HaveNewClosure(unsigned i)
{
  Closure *clos_i = Closures(core, i);

  if(core->use_masking)
    if(clos_i->IsMasked())
      return;

  for(unsigned l = 0; l < clos_i->lines.Size(); l++)
    for(unsigned n = 0; n < clos_i->lines[l]->closures.Size(); n++)
    {
      Closure *clos_j = clos_i->lines[l]->closures[n];
      // i is not its own neighbour
      if(clos_i != clos_j)
      {
        // now i and j are neighbours via common line l
        unsigned pos_i = l;
        // position of line on closure j
        unsigned pos_j = clos_j->lines.Find(clos_i->lines[pos_i]);
        bool same_sense = clos_i->senses[pos_i] == clos_j->senses[pos_j];
        // if not same sense, the closures lie alongside
        if(!same_sense)
        {
          // note that closures might share more than one line. only create new
          // flap once, e.g. at start or end of the set of shared lines
          //if(StartOfCollinearRun(i, l, j, pos_j))
          if(EndOfCollinearRun(clos_i, pos_i, clos_j, pos_j))
            NewFlap(clos_i, clos_j);
        }
      }
    }
}

/**
 * @brief NewFlap: Create new flap from closures i and j.
 * @param clos_i Closure i
 * @param clos_j Closure j
 */
void FormFlaps::NewFlap(Closure *clos_i, Closure *clos_j)
{
  core->NewGestalt(GestaltPrinciple::FORM_FLAPS, new Flap(core, clos_i, clos_j));

  // TODO: this has O(n^2 log n) complexity, which is bad ... However typically
  // the number of flaps tends to be small. So we are fine for a while.
  // TODO: perhaps disable, as actually we do want flaps sharing closures: think
  // of a cube!
  RankGestalts(Gestalt::FLAP, CmpFlaps);
  Mask();
}

/**
 * @brief Mask ranked Gestalts.
 */
void FormFlaps::Mask()
{
  // array containing for each closure the ID of the highest ranking flap which
  // contains that closure
  Array<unsigned> owner(NumClosures(core));
  owner.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumFlaps(core); j++)
  {
    Flap *flap = (Flap*)core->RankedGestalts(Gestalt::FLAP, j);
    flap->Mask(UNDEF_ID);
    for(unsigned l = 0; l < 2; l++)
    {
      // first check if any previous (i.e. stronger) flap uses one of my
      // closures. if yes, let that flap mask me
      if(owner[flap->clos[l]->ID()] != UNDEF_ID)
        flap->Mask(owner[flap->clos[l]->ID()]);
      // now the line belongs to me (and I might mask even lesser closures)
      owner[flap->clos[l]->ID()] = flap->ID();
    }
  }
}

}

