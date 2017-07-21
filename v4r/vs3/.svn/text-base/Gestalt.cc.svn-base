/**
 * @file Gestalt.cc
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2007, 2010
 * @version 0.1
 * @brief Class file of Gestalt base class.
 **/

#include "Math.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"
#include <cstdio>

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "SEGMENT",
  "LINE",
  "ARC",
  "CONVEX_ARC_GROUP",
  "ELLIPSE",
  "CIRCLE",
  "COLLINEARITY",
  "T_JUNCTION",
  "L_JUNCTION",
  "A_JUNCTION",
  "E_JUNCTION",
  "CORNER",
  "EXT_ELLIPSE",
  "CYLINDER",
  "CONE",
  "CLOSURE",
  "RECTANGLE",
  "FLAP",
  "FLAP_ARI",
  "CUBE",
  "UNDEF"
  };

static const int type_names_length[] = {7, 4, 3, 16, 7, 6, 12, 10, 10, 10, 10, 6, 11, 8, 4, 7, 9, 4, 8, 4, 5};

/**
 * @brief Returns the name of a given gestalt type.
 * @param t Type of Gestalt
 * @return Name of the Gestalt as text.
 */
const char* Gestalt::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Returns the length of the name of a given gestalt type.
 * @param t Type of Gestalt
 * @return Lenght of Gestalt type name.
 */
const int Gestalt::TypeNameLength(Type t)
{
  return type_names_length[t];
}

/**
 * @brief Return the enum type of a given gestalt type name.
 * @param type_name Name of the Gestalt as text.
 * @return Type of the Gestalt
 */
Gestalt::Type Gestalt::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 * @brief Gestalt constructor.
 * A new gestalt gets assigned the next higher id, i.e. the current number of
 * gestalts of that type.
 * @param c Vision core
 * @param t Type of Gestalt
 */
Gestalt::Gestalt(VisionCore *c, Type t)
{
  core = c;
  type = t;
  SetID(core->NumGestalts(type));
  SetRank(id); // as long as no ranking is performed, the rank is simply the id
  acc = 0.;
  sig = 0.;
  masked = UNDEF_ID;
}

/**
 * @brief Return some textual information.
 * Note: the returned string is allocated statically and will be modified by
 * the next call to GetInfo().
 */
const char* Gestalt::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "  Gestalt Type: %s \n"
      "  ID: %u \n"
      "  rank: %u \n"
      "  masked: %u\n"
			"  non-acc: %f\n"
			"  significance: %5.2f\n"
      "  --------------------\n",
      TypeName(type), id, rank, masked, 1. - acc, sig);
  return info_text;
}

}

