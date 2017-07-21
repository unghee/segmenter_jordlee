/**
 * @file FormSegments.hh
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Gestalt principle class for forming segments.
 **/

#ifndef Z_FORM_SEGMENTS_HH
#define Z_FORM_SEGMENTS_HH

#include "IdImage.hh"
#include "Segment.hh"
#include "GestaltPrinciple.hh"
#include "CEdge.hh"     //jp, 20091111

namespace Z
{

/**
 * @brief Gestalt principle class for forming segments.
 */
class FormSegments : public GestaltPrinciple
{
private:
  float alpha, omega;								///< alpha and omega of the canny e.d.
  bool done;

  CEdge computeEdges;   //jp, 20091111
  void CreateSegmentsCEdge();

  void CreateSegmentsMatas();
  void DrawToEdgeImage(Segment *seg);
  void Create();
  void Rank();
  void SegmentArcsLines();

public:
  static IdImage *edge_img; 				///< ID image containing segment ids for each edgel
  static float *dir_img;						///< Image containing gradient orientations
  static int num_edgels;						///< Total number of edgels

  FormSegments(VisionCore *vc);
  virtual ~FormSegments();
  virtual void Reset();
  virtual void PreOperate();
  virtual bool NeedsOperate() {return !done;}
  virtual void SetCanny(float a, float o);
};

}

#endif
