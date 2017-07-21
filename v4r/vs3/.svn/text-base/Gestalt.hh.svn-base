/**
 * @file Gestalt.hh
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt base class.
 **/

#ifndef Z_GESTALT_HH
#define Z_GESTALT_HH

#include <cstdio>
#include "Array.hh"
#include "Math.hh"

namespace Z
{

class VisionCore;

/**
 * @brief Base class for all Gestalts
 */
class Gestalt
{
public:
  enum Type
  {
    SEGMENT,            // 0
    LINE,               // 1
    ARC,                // 2
    CONVEX_ARC_GROUP,   // 3
    ELLIPSE,            // 4
    CIRCLE,             // 5
    COLLINEARITY,       // 6
    T_JUNCTION,         // 7
    L_JUNCTION,         // 8
    A_JUNCTION,         // 9
    E_JUNCTION,         // 10
    CORNER,             // 11
    EXT_ELLIPSE,        // 12
    CYLINDER,           // 13
    CONE,
    CLOSURE,
    RECTANGLE,
    FLAP,
    FLAP_ARI,
    CUBE,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:
  VisionCore *core;
  Type type;
  unsigned id;
  unsigned rank;

public:
  double acc;		///< probability of accidental occurrance
  double sig;		///< significance
  unsigned masked;  	///< TODO: might be masked by more gestalts

public:
  static const char* TypeName(Type t);
  static const int TypeNameLength(Type t);
  static Type EnumType(const char *type_name);

  Gestalt(VisionCore *c, Type t);
  virtual ~Gestalt() {}
  Type GetType() const {return type;}
  unsigned ID() const {return id;}
  void SetID(unsigned i) {id = i;}
  unsigned Rank() {return rank;}
  void SetRank(unsigned r) {rank = r;}
  void Mask(unsigned by) {masked = by;}
  bool IsMasked() {return masked != UNDEF_ID;}
  bool IsUnmasked() {return masked == UNDEF_ID;}
  virtual void Draw(int detail = 0) {}
  virtual void DrawInfo() {}
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y) {return false;}
};

}

#endif

