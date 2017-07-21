/**
 * @file GestaltPrinciple.cc
 * @author Michael Zillich, Anddreas Richtsfeld
 * @date 2006, 2010
 * @version 0.1
 * @brief Gestalt principle class
 */

#include <string.h>
#include "VisionCore.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "FORM_SEGMENTS",
  "FORM_LINES",
  "FORM_E_JUNCTIONS",
  "FORM_ARCS",
  "FORM_ARC_JUNCTIONS",
  "FORM_CONVEX_ARC_GROUPS",
  "FORM_ELLIPSES",
  "FORM_CIRCLES",
	"FORM_EXT_ELLIPSES",
	"FORM_CYLINDERS",
	"FORM_CONES",
  "FORM_JUNCTIONS",
  "FORM_CORNERS",
  "FORM_CLOSURES",
  "FORM_RECTANGLES",
  "FORM_FLAPS",
  "FORM_FLAPS_ARI",
  "FORM_CUBES",
  "UNDEF"
  };


/**
 * @brief Constructor of GestaltPrinciple
 * @param vc Vision core
 */
GestaltPrinciple::GestaltPrinciple(VisionCore *vc)
{
  core = vc;
  ResetRunTime();
}


/**
 * @brief Returns the name of a given Gestalt principle type.
 * @param t Type of Gestalt principle
 * @return Name of Gestalt principle
 */
const char* GestaltPrinciple::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Return the enum type of a given Gestalt principle type name.
 * @param type_name Type name of the Gestalt principle
 * @return Gestalt principle type
 */
GestaltPrinciple::Type GestaltPrinciple::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 * @brief Rank Gestalts
 * @param type Type of Gestalt principle
 * @param compar Compare function
 */
void GestaltPrinciple::RankGestalts(Gestalt::Type type, int(*compar)(const void *, const void *))
{
  // first rank gestalts
  core->RankedGestalts(type).Sort(compar);
  // then set rank for each gestalt
  for(unsigned i = 0; i < core->NumGestalts(type); i++)
    core->RankedGestalts(type, i)->SetRank(i);
}

/**
 * @brief Starts the timer for calculation of the processing time \n
 * for each Gestalt principle.
 */
void GestaltPrinciple::StartRunTime()
{
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &startTime);
}

/**
 * @brief Stops the timer for calculation of the processing time for \n
 * each Gestalt principle and adds the time to the runtime counter.
 */
void GestaltPrinciple::StopRunTime()
{
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &endTime);
  runtime += timespec_diff(&endTime, &startTime);
}

}

