/**
 * @file Segment.hh
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Header of Gestalt class segment.
 **/

#ifndef Z_SEGMENT_HH
#define Z_SEGMENT_HH

#include "Array.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Edgel.hh"

namespace Z
{

/**
 * @brief Gestalt class Segment
 */
class Segment : public Gestalt
{
private:
  void CalculateSignificance();

public:
  int b_id;                      // boundary id for object segmenter
  Array<Edgel> edgels;

  Segment(VisionCore *c, const Array<Edgel> &arr, int _b_id = 0);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  Vector2 Tangent(int i, int l = 0, int u = UNDEF_ID);
};

inline Array<Gestalt*>& Segments(VisionCore *core)
{
  return core->Gestalts(Gestalt::SEGMENT);
}
inline Segment* Segments(VisionCore *core, unsigned id)
{
  return (Segment*)core->Gestalts(Gestalt::SEGMENT, id);
}
inline unsigned NumSegments(VisionCore *core)
{
  return core->NumGestalts(Gestalt::SEGMENT);
}

}

#endif

