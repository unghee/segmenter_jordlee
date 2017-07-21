/**
 * $Id: ParallelLineGroup.hh,v 1.5 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_PARALLEL_LINE_GROUP_HH
#define Z_PARALLEL_LINE_GROUP_HH

#include "Array.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class ParallelLineGroup : public Gestalt
{
private:
  void CalculateSignificance();

public:
  Array<unsigned> lines;
  unsigned seed_line;
  double sum_length;

  ParallelLineGroup(const Array<unsigned> &ls, unsigned seed);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& ParallelLineGroups()
{
  return Gestalts(Gestalt::PARALLEL_LINE_GROUP);
}

inline ParallelLineGroup* ParallelLineGroups(unsigned id)
{
  return (ParallelLineGroup*)Gestalts(Gestalt::PARALLEL_LINE_GROUP, id);
}

inline unsigned NumParallelLineGroups()
{
  return NumGestalts(Gestalt::PARALLEL_LINE_GROUP);
}

}

#endif

