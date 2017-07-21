/**
 * @file Flap.hh
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt class for flaps.
 */

#ifndef Z_FLAP_HH
#define Z_FLAP_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Closure.hh"

namespace Z
{

/**
 * @class Flap Gestalt class for flaps.
 * @brief Flaps, created from closures.
 */
class Flap : public Gestalt
{
public:
  Closure *clos[2];			///< The two closures of the flap

  Flap(VisionCore *vc, Closure *c0, Closure *c1);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Flaps(VisionCore *core)
{
  return core->Gestalts(Gestalt::FLAP);
}
inline Flap* Flaps(VisionCore *core, unsigned id)
{
  return (Flap*)core->Gestalts(Gestalt::FLAP, id);
}
inline unsigned NumFlaps(VisionCore *core)
{
  return core->NumGestalts(Gestalt::FLAP);
}

}

#endif

