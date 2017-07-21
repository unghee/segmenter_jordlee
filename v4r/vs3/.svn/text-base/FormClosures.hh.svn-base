/**
 * @file FormClosures.hh
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Gestalt principle class for forming closed convex contours (closures).
 **/


#ifndef Z_FORM_CLOSURES_HH
#define Z_FORM_CLOSURES_HH

#include <list>
#include <ext/hash_map>
#include "Array.hh"
#include "GestaltPrinciple.hh"

using namespace __gnu_cxx;

namespace Z
{

/**
 * @brief Gestalt principle class for creating closed convex contours (closures).
 */
class FormClosures : public GestaltPrinciple
{
private:
    bool debug; // HACK

  // using SIG_OFFSET - sig as cost value favours high signature paths as short
  // paths while maintaining positive costs.
  // Note: make sure that offset is larger than the maximum signficance.
  // TODO: maybe there is a nicer way of achieving the same.
  static const double SIG_OFFSET = 100.;
  // not LEFT nor RIGHT
  static const int NONE;

  // map of all closure candidates which a line is the start of
  hash_map< unsigned, list<unsigned> > is_start_of;
  // map of all closure candidates which a line is the end of
  hash_map< unsigned, list<unsigned> > is_end_of;
  vector<double> dist;           // known distances from s to v
  vector<Line*> prev;            // previous vertex on path
  vector<int> exts;              // number of extension leading from a node
  vector<int> ins;               // number of incoming links into a node
  vector<int> ends;              // ends of lines open for extension
                                 // jct i is the jct between line i and prev[i]
  vector<LJunction*> jcts;       // L-jcts between lines of path
  vector<Collinearity*> colls;   // collinearities between lines of path
  vector<int> bend;              // bend direction for path through a node
  vector<bool> blacklist;        // whether a node should be excluded from
                                 // further searches
  vector<Line*> Q;               // priority queue of all unvisited vertices
  size_t Q_size;

  void HaveNewLJunction(unsigned idx);
  void HaveNewCollinearity(unsigned idx);
  void HaveNewTJunction(unsigned idx);
  void UpdateLNeighbors(LJunction *ljct);
  void UpdateCNeighbors(Collinearity *coll);
  bool IsIsolatedL(LJunction *ljct);
  bool IsIsolatedC(Collinearity *coll);
  bool InSameClosure(Line *a, Line *b);
  void NewClosure(Line *first, Line *last);
  bool ShortestPath(LJunction *ljct, Collinearity *coll, Line *s, Line *t, int end_s, int end_t);
  void ClearBlacklist();
  void ClearPaths();
  void ClearNode(Line *u);
  int ExtendLJunctions(Line *u);
  int ExtendLJunctions(Line *u, int side);
  int ExtendNonConvexLJunctions(Line *u);
  int ExtendCollinearities(Line *u);
  bool LineIntersectingPath(Line *l_new, LJunction *j_new, Collinearity *c_new);
  bool ClosingLineIntersectingPath(Line *l_close, Line *l_open, LJunction *j_init, Collinearity *c_init);
  void Mask();
  void PrintHeap();


public:
  FormClosures(VisionCore *vc);
  virtual void Reset();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif

