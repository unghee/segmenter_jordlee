/**
 * $Id: FormArcs.hh,v 1.15 2007/02/04 23:53:03 mxz Exp mxz $
 */

#ifndef Z_FORM_ARCS_HH
#define Z_FORM_ARCS_HH

#include "GestaltPrinciple.hh"
#include "Array.hh"
#include "Edgel.hh"

namespace Z
{

class FormArcs : public GestaltPrinciple
{
public:
  static const double MAX_CIRCLE_DIST;	///< max dist of edgel from circle
  static const double RANSAC_DIST;  		///< max dist of edgel from circle
  static const unsigned MIN_LENGTH;			///< min number of pixels in arc
  static const double MIN_RADIUS;				///< min radius
  static const double MAX_RADIUS;				///< max radius
  int FIT_METHOD;												///< fit method
 
private:
  void Create();
  void Rank();
  bool EdgelsOnCircle(Array<Edgel> &edgels, unsigned i, unsigned j, unsigned k);
  int Support(Array<Edgel> &edgels, int l, int u, int &i, int &j, int &k, Vector2 &center, double &radius);
  bool FitArc(Segment * seg, unsigned *cur_pos);
  void FitArcRANSAC(Segment *seg, int l, int u);

public:
  FormArcs(VisionCore *vc);
  virtual void Reset();
  virtual void PreOperate();
};
 
}

#endif

