/**
 * @file FormJunctions.hh
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormJunctions.
 **/

#ifndef Z_FORM_JUNCTIONS_HH
#define Z_FORM_JUNCTIONS_HH

#include "LJunction.hh"
#include "Collinearity.hh"

namespace Z
{

class FormJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;

  void Rank();
  void CreateL(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void CreateC(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void CreateT(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void SplitLine(Line *l, const Vector2 &inter, const Vector2 &dir,
      Line **l_left, Line **l_right,
      int *end_left, int *end_right, Collinearity **c_new);
  Line* SplitLine(Line *l, const Vector2 &p, Collinearity **c_new);
  unsigned FindLineSplitIdx(Line *l, const Vector2 &p);
  void UpdateClosures(Line *l1, Line *l2, Collinearity *coll);
  Collinearity* NewCollinearity(Line *line_i, Line *line_j,
      int end_i, int end_j, bool inform);
  LJunction* NewL(Line *line_i, Line *line_j, int end_i, int end_j);
  TJunction* NewT(Line *pole, Line *left, Line *right,
      int end_p, int end_l, int end_r,
      Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r);

public:

  FormJunctions(VisionCore *vc);
  virtual ~FormJunctions();
  virtual void CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts);
};

}

#endif

