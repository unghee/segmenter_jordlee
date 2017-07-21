/**
 * @file ConvexArcGroup.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Header file of Gestalt ConvexArcGroup.
 */

#ifndef Z_CONVEX_ARC_GROUP_HH
#define Z_CONVEX_ARC_GROUP_HH

#include "Math.hh"
#include "Array.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Arc;

/**
 * @brief Class ConvexArcGroup
 */
class ConvexArcGroup : public Gestalt
{
public:
  Array<Arc*> arcs;
  double ang_cover;
  double fit_quality;
  double closedness;
  double sig2;

private:
  void CalculateSignificance();

public:
  ConvexArcGroup(VisionCore *vc);
  ConvexArcGroup(VisionCore *vc, Array<Arc*> &a, unsigned l, unsigned u,
      double s = -HUGE);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  bool HasArc(Arc *arc);
  bool StillConvex(Arc *arc);
  Arc* FirstArc() {return (arcs.Size() > 0 ? arcs[0] : 0);}
  Arc* LastArc() {return (arcs.Size() > 0 ? arcs[arcs.Size()-1] : 0);}
};

extern double ArcGroupLargestAngularGap(Array<Arc*> &arcs);
extern double ArcGroupAngularCoverage(Array<Arc*> &arcs);
extern double ArcGroupClosedness(Array<Arc*> &arcs);
extern double ArcGroupArea(Array<Arc*> &arcs);
extern double ArcGroupGapAreaRatio(Array<Arc*> &arcs);
extern double ArcGroupSignificance(VisionCore *core, Array<Arc*> &arcs, unsigned l, unsigned u);

inline Array<Gestalt*>& ConvexArcGroups(VisionCore *core)
{
  return core->Gestalts(Gestalt::CONVEX_ARC_GROUP);
}
inline ConvexArcGroup* ConvexArcGroups(VisionCore *core, unsigned id)
{
  return (ConvexArcGroup*)core->Gestalts(Gestalt::CONVEX_ARC_GROUP, id);
}
inline unsigned NumConvexArcGroups(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CONVEX_ARC_GROUP);
}

}

#endif

