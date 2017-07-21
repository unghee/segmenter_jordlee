/**
 * @file FormConvexArcGroups.hh
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormConvexArcGroups.
 * 
 * TODO:
 * - create tangents with the expected number of intersections, so that no
 *   later resizing of the arrays is necessary.
 * - avoid generation of double groups, using signature
 */

#ifndef Z_FORM_CONVEX_ARC_GROUPS_HH
#define Z_FORM_CONVEX_ARC_GROUPS_HH

#include "Array.hh"
#include "Gestalt.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class ArcLink
{
public:
  unsigned id;  ///< ID of the other intersecting tangent
  double sig;
  ArcLink() {}
  ArcLink(unsigned i, double s) : id(i), sig(s) {}
};

class FormConvexArcGroups : public GestaltPrinciple
{
private:
  int SEARCH_METHOD;
  int GROUP_CRITERION;
  static const int HASH_SIZE = 29989;  // TODO: this is quite arbitrary
  unsigned hashtable[HASH_SIZE];
  Array<Arc*> arcs;
//   Array< Array<ArcLink> > links;
//   Array<ConvexArcGroup*> cands;
//   unsigned cands_created;

  double Evaluate(Array<Arc*> &arcs, unsigned l, unsigned u, int criterion);
  Arc* NextEnd(Array<Arc*> &arcs, unsigned l, unsigned u, int &end, int criterion);
  double GrowBest(Array<Arc*> &arcs, unsigned &l, unsigned &u);
  //void GrowAll(Array<unsigned> &arcs, unsigned l, unsigned u,
  //    Array<unsigned> &arcs_opt, unsigned &size_opt, double q_opt);
  //void DrawSearchLines(Arc *arc);
  //void CheckSearchLines(Arc *arc);
  //bool CreateIntersection(Arc *arc_i, int end_i, Arc *arc_j, int end_j);
  //void CreateArcJunctions(Arc *arc_i, int end_i, Array<VoteImage::Elem> &iscts);
  //void Rank();
  void ClearHashTable();
  unsigned Hash(Array<Arc*> &arcs);
  unsigned Hash(Array<Arc*> &arcs, unsigned l, unsigned u);
  //void ExhaustiveSearch(unsigned arc, Array<unsigned> &arcs,
  //    Array<unsigned> &arcs_opt);
  void GreedySearch(Arc *arc, Arc *forced_next, Array<Arc*> &arcs);
  //void NewCand(ConvexArcGroup *g);
  //void InstantiateBestCand();
  //double LinkSignificance(unsigned i, unsigned j);
  void HaveNewAJunction(unsigned idx);
  void HaveNewArc(unsigned idx);

public:
//   static VoteImage *vote_img;

  FormConvexArcGroups(VisionCore *vc);
  virtual ~FormConvexArcGroups();
  virtual void Reset();
  virtual void Draw(int detail = 0);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};
 
}

#endif

