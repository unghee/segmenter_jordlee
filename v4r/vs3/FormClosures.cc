/**
 * @file FormClosures.cc
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Gestalt principle class for forming closures.
 **/

#include <algorithm>
#include <vector>
#include <cstdio>
#include "Vector.hh"
#include "VisionCore.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "FormClosures.hh"

namespace Z
{

// en/disable printfs
// #define DEBUG

// try to also find non-convex paths
// TODO: this still has bugs
//#define FIND_NON_CONVEX

// not LEFT nor RIGHT
const int FormClosures::NONE = RIGHT + 1;


static int CmpClosures(const void *a, const void *b)
{
  if( (*(Closure**)a)->sig > (*(Closure**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

#define COST_INF HUGE

/**
 * @brief Comparison function object for heap.
 * Note that the default STL heap is a max-heap, i.e. the first element is the
 * maximum. The default STL heap uses the less-than operator '<'.
 * By having heap use this LargerThan operator we get a min-heap, which is
 * required by the Dijkstra algorithm.
 */
class CompCostLargerThan
{
public:
  vector<double> &costs;
  CompCostLargerThan(vector<double> &c) : costs(c) {}
  bool operator ()(const Line*& a, const Line*& b)
  {
    return costs[a->ID()] > costs[b->ID()];
  }
  bool operator ()(Line*& a, Line*& b)
  {
    return costs[a->ID()] > costs[b->ID()];
  }
};

/**
 * @brief Comparison function for sorting LJunctions of a closure according to openeing
 * angles, smallest to largest.
 * @param a 
 * @param b
 */
static int CmpAngles(const void *a, const void *b)
{
  // if a is undefined, move it to end
  if(*(LJunction**)a == 0)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(LJunction**)b == 0)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( (*(LJunction**)a)->OpeningAngle() <
           (*(LJunction**)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Constructor of Gestalt principle class FormClosures.
 * @param vc Vision core
 */
FormClosures::FormClosures(VisionCore *vc) : GestaltPrinciple(vc)
{
  debug = false;
}

/**
 * @brief Reset Gestalt principle class
 */
void FormClosures::Reset()
{}

/**
 * @brief Inform about new Gestalt.
 * @param type Type of Gestalt.
 * @param idx Index of new Gestalt.
 */
void FormClosures::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      HaveNewLJunction(idx);
      break;
    case Gestalt::COLLINEARITY:
      HaveNewCollinearity(idx);
      break;
    case Gestalt::T_JUNCTION:
      HaveNewTJunction(idx);
      break;
    default:
      break;
  }
}

/**
 * @brief Add new L-Junction and intitiate shortest path search.
 * @param idx Index of new Gestalt.
 */
void FormClosures::HaveNewLJunction(unsigned idx)
{
  LJunction *ljct = LJunctions(core, idx);
// TODO  core->stats.attempt_path_search++;
  // only attempt shortest path search if the L-jct is not isolated
  if(!IsIsolatedL(ljct))
  {
    ClearBlacklist();
    while(ShortestPath(ljct, 0,
                       ljct->line[RIGHT],
                       ljct->line[LEFT],
                       Other(ljct->near_point[RIGHT]),
                       Other(ljct->near_point[LEFT])));
  }
}

/**
 * @brief Add new Collinearity and intitiate shortest path search.
 * @param idx Index of new Gestalt.
 */
void FormClosures::HaveNewCollinearity(unsigned idx)
{
  Collinearity *coll = Collinearities(core, idx);
// TODO  core->stats.attempt_path_search++;
  // only attempt shortest path search if the collinearity is not isolated
  if(!IsIsolatedC(coll))
  {
    ClearBlacklist();
    while(ShortestPath(0, coll,
                       coll->line[0],
                       coll->line[1],
                       Other(coll->near_point[0]),
                       Other(coll->near_point[1])));
  }
}

/**
 * @brief Add new T-Junction and intitiate shortest path search.
 * @param idx Index of new Gestalt.
 */
void FormClosures::HaveNewTJunction(unsigned idx)
{
//   TJunction *tjct = TJunctions(core, idx);
//   UpdateTNeighbors(tjct);
    printf("FormClosures::HaveNewTJunction: function is obsolete: splitting T-Junctions into 2L+C?\n");
}


/**
 * @brief Checks whether the given L-junction is isolated, i.e. can not form a closure. \n
 * Only if the open ends of LEFT and RIGHT arm have at least one continuation, \n
 * the junction could possibly form a closure.
 * @param ljct L-Junction
 */
bool FormClosures::IsIsolatedL(LJunction *ljct)
{
  Line *left = ljct->line[LEFT];
  Line *right = ljct->line[RIGHT];
  int open_l = Other(ljct->near_point[LEFT]);
  int open_r = Other(ljct->near_point[RIGHT]);
  // number of possible continuing junctions at open end of LEFT line
  int cont_l = left->l_jct[open_l][RIGHT].Size() +
               left->coll[open_l].Size();
  // same for right line
  int cont_r = right->l_jct[open_r][LEFT].Size() +
               right->coll[open_r].Size();
  // if one end has no continuation, this junction is isolated
  return cont_l == 0 || cont_r == 0;
}

/**
 * @brief Checks whether the given Collinearity is isolated, i.e. can not form a \n
 * closure.  Only if the open ends of first and second line have at least one \n
 * continuation, the collinearity could possibly form a closure.
 * @param coll Collinearity
 */
bool FormClosures::IsIsolatedC(Collinearity *coll)
{
  Line *a = coll->line[0];
  Line *b = coll->line[1];
  int open_a = Other(coll->near_point[0]);
  int open_b = Other(coll->near_point[1]);
  // number of possible continuing junctions at open end of first line
  int cont_a = a->l_jct[open_a][LEFT].Size() +
               a->l_jct[open_a][RIGHT].Size() +
               a->coll[open_a].Size();
  // same for second line
  int cont_b = b->l_jct[open_b][LEFT].Size() +
               b->l_jct[open_b][RIGHT].Size() +
               b->coll[open_b].Size();
  // if one end has no continuation, this junction is isolated
  return cont_a == 0 || cont_b == 0;
}

/**
 * @brief Return whether lines a and b already are part of the same closure.
 * This can only happen if there is first e.g. a collinearity and then an
 * L-junction created between the same lines.
 * @param a First line a.
 * @param b Second lien b.
 */
bool FormClosures::InSameClosure(Line *a, Line *b)
{
  return a->closures.Intersect(b->closures);
}

/**
 * @brief Dijkstra shortest path algorithm. \n
 * Find a path from right to left line taking only left turns. \n
 *
 * @param ljct L-Junction from which we start
 * @param coll Collinearity from which we start
 * @param s  start line, which is the RIGHT arm of an L-jct
 * @param t  target line, which is the LEFT arm of that L-jct
 * @param end_s  the open end of line s (opposite to the joined end)
 * @param end_t  the open end of line t (opposite to the joined end)
 */
bool FormClosures::ShortestPath(LJunction *ljct, Collinearity *coll,
    Line *s, Line *t, int end_s, int end_t)
{
  Line *u = 0;
  bool have_connected_vertices = true;
  bool have_closure = false;
  CompCostLargerThan comp(dist);

  ClearPaths();
  // Distance from s to s is 0.
  dist[s->ID()] = 0.;
  // Note prev[s] is UNDEF_ID, not t. This serves as stopping criterion when
  // traversing the path backwards.
  prev[s->ID()] = 0;
  // But s does have an open end
  ends[s->ID()] = end_s;
  // and a junction (or collinearity), namely that with t
  jcts[s->ID()] = ljct;
  colls[s->ID()] = coll;
  // iff we have an L-jct, we already know the bending direction
  if(jcts[s->ID()] != 0)
    bend[s->ID()] = LEFT;
  // prepare min-heap (priority queue)
  Q_size = Q.size();
  make_heap(Q.begin(), Q.begin() + Q_size, comp);
  if(debug) printf("%s: start search s: %d, t: %d\n", __FUNCTION__, s->ID(), t->ID());
  while(!have_closure && have_connected_vertices && Q_size > 0)
  {
    if(debug) printf("before pop:\n");
    PrintHeap();
    // Remove best vertex from priority queue
    u = Q.front();
    if(debug) printf("%s: searching: visiting: %d from %d\n", __FUNCTION__,
        u->ID(), (prev[u->ID()] != 0 ? prev[u->ID()]->ID() : -1));
    // If the best vertex has infinite distance (i.e. there is no path to it)
    // then there is no point in continuing
    if(dist[u->ID()] != COST_INF)
    {
      pop_heap(Q.begin(), Q.begin() + Q_size, comp);
      Q_size--;
      if(!blacklist[u->ID()])
      {
        // If we reached the target and the ends interlock (u's open end along
        // the path is opposite the open end of the original L-jct)
        if(u == t && ends[u->ID()] == Other(end_t))
        {
          if(!ClosingLineIntersectingPath(u, s, ljct, coll))
          {
            if(debug) printf("%s: have closure ****************************\n",
                __FUNCTION__);
            NewClosure(s, t);
            if(debug) printf("after new closure:\n");
            PrintHeap();
            // found a path to target, which is guaranteed to be the shortest,
            // so can stop here
            have_closure = true;
          }
        }
        else
        {
          int cnt = 0;
          // For each edge (u,v) outgoing from u and not intersecting the path
          // so far
          cnt += ExtendLJunctions(u);
          cnt += ExtendCollinearities(u);
#ifdef FIND_NON_CONVEX
          if(cnt == 0)
            cnt += ExtendNonConvexLJunctions(u);
#endif
          exts[u->ID()] = cnt;
          if(cnt > 0)
            make_heap(Q.begin(), Q.begin() + Q_size, comp);
          if(debug) printf("%s: extensions: %d\n", __FUNCTION__, cnt);
        }
      }
      // else if u is blacklisted, do nothing, continue with next node
    }
    else
    {
      if(debug) printf("INF cost -> end search\n");
      have_connected_vertices = false;
    }
  }
  if(debug) printf("%s: end search\n\n", __FUNCTION__);
  // HACK: gather search statistics
// TODO  core->stats.path_max_visited =
//     max(core->stats.path_max_visited, (int)(Q.size() - Q_size));
//   core->stats.path_avg_visited =
//     (double)(core->stats.path_search*core->stats.path_avg_visited +
//      (int)(Q.size() - Q_size))/(double)(core->stats.path_search + 1);
//   core->stats.path_search++;
  return have_closure;
}

/**
 * @brief Clear node blacklist, which is _not_ cleared between searches.
 */
void FormClosures::ClearBlacklist()
{
  unsigned n = NumLines(core);
  blacklist.resize(n);
  fill(blacklist.begin(), blacklist.end(), false);
}

/**
 * @brief Clear all temporary path stuff for new search.
 */
void FormClosures::ClearPaths()
{
  unsigned n = NumLines(core);
  dist.resize(n);
  fill(dist.begin(), dist.end(), COST_INF);
  prev.resize(n);
  fill(prev.begin(), prev.end(), (Line*)0);
  exts.resize(n);
  fill(exts.begin(), exts.end(), 0);
  ins.resize(n);
  fill(ins.begin(), ins.end(), 0);
  ends.resize(n);
  fill(ends.begin(), ends.end(), NONE);
  jcts.resize(n);
  fill(jcts.begin(), jcts.end(), (LJunction*)0);
  colls.resize(n);
  fill(colls.begin(), colls.end(), (Collinearity*)0);
  bend.resize(n);
  fill(bend.begin(), bend.end(), NONE);
  Q.resize(n);
  for(unsigned i = 0; i < n; i++)
    // enter each node 0..n-1 into the priority queue
    Q[i] = Lines(core, i);
}

/**
 * @brief Clear all node stuff for new search.
 * @param u Line
 */
void FormClosures::ClearNode(Line *u)
{
  dist[u->ID()] = COST_INF;
  prev[u->ID()] = (Line*)0;
  exts[u->ID()] = 0;
  ins[u->ID()] = 0;
  ends[u->ID()] = NONE;
  jcts[u->ID()] = (LJunction*)0;
  colls[u->ID()] = (Collinearity*)0;
  bend[u->ID()] = NONE;
}

/**
 * @brief Extend path using L-junctions.
 * @param u  last line of path so far
 */
int FormClosures::ExtendLJunctions(Line *u)
{
  int cnt = 0;
  if(bend[u->ID()] == NONE)
  {
    cnt += ExtendLJunctions(u, LEFT);
    cnt += ExtendLJunctions(u, RIGHT);
  }
  else if(bend[u->ID()] == LEFT)
  {
    cnt += ExtendLJunctions(u, LEFT);
  }
  else if(bend[u->ID()] == RIGHT)
  {
    cnt += ExtendLJunctions(u, RIGHT);
  }
  return cnt;
}

/**
 * @brief Extend non-convex path using L-junctions.
 * @param u  last line of path so far
 */
int FormClosures::ExtendNonConvexLJunctions(Line *u)
{
  int cnt = 0;
  if(bend[u->ID()] == LEFT)
    cnt += ExtendLJunctions(u, RIGHT);
  else if(bend[u->ID()] == RIGHT)
    cnt += ExtendLJunctions(u, LEFT);
  return cnt;
}

/**
 * @brief Extend path using L-junctions.
 * TODO: decide for a proper cost function
 * @param u  last line of path so far
 * @param side  extend LEFT or RIGHT L-junctions at the end of u
 * @return Returns the number of extensions.
 */
int FormClosures::ExtendLJunctions(Line *u, int side)
{
  int cnt = 0;
  int other = Other(side);
  // note: If side is LEFT:
  // s is the RIGHT arm of an L-jct. So for a convex continuation s
  // must be the LEFT arm of the next L-jct. Same for all subsequent u's.
  for(unsigned i = 0; i < u->l_jct[ends[u->ID()]][side].Size(); i++)
  {
    LJunction *j = u->l_jct[ends[u->ID()]][side][i];
    Line *v = j->line[other];
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    double alt = dist[u->ID()] + j->acc; 
    // alt = dist[u] + 1.; 
    // alt = dist[u] + SIG_OFFSET - LJunctions(j)->sig; 
    // alt = dist[u] + 1./LJunctions(j)->sig; 
    if(!LineIntersectingPath(u, j, 0) &&
       alt < dist[v->ID()])
    {
      dist[v->ID()] = alt; 
      prev[v->ID()] = u;
      ends[v->ID()] = Other(j->near_point[other]);
      jcts[v->ID()] = j;
      colls[v->ID()] = 0;
      // If side is LEFT (u is the LEFT arm of an L-jct), then v is the RIGHT,
      // and we have a LEFT bending curve. RIGHT accordingly.
      bend[v->ID()] = side;
      ins[v->ID()]++;
      cnt++;
    }
    else
    {
      // else just note that there is a further (more expensive) path to node v
      ins[v->ID()]++;
    }
  }
  return cnt;
}

/**
 * @brief Extend path using collinearities.
 * @param u  last line of path so far
 * @return Returns the number of extensions.
 */
int FormClosures::ExtendCollinearities(Line *u)
{
  int cnt = 0;
  for(unsigned i = 0; i < u->coll[ends[u->ID()]].Size(); i++)
  {
    Collinearity *c = u->coll[ends[u->ID()]][i];
    Line *v = c->OtherLine(u);
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    double alt = dist[u->ID()] + c->acc;
    // alt = dist[u] + 1.; 
    // alt = dist[u] + SIG_OFFSET - Collinearities(c)->sig; 
    // alt = dist[u] + 1./Collinearities(c)->sig; 
    if(!LineIntersectingPath(u, 0, c) &&
       alt < dist[v->ID()])
    {
      dist[v->ID()] = alt;
      prev[v->ID()] = u;
      ends[v->ID()] = Other(c->WhichEndIs(v));
      colls[v->ID()] = c;
      jcts[v->ID()] = 0;
      // collinearities don't change the bending direction
      bend[v->ID()] = bend[u->ID()];
      ins[v->ID()]++;
      cnt++;
    }
    else
    {
      // else just note that there is a further (more expensive) path to node v
      ins[v->ID()]++;
    }
  }
  return cnt;
}

/**
 * @brief Form a new closure along path from first to last line. \n
 * Note: path formation made sure that the resulting polygon is non-intersecting \n
 * and (mostly) convex.
 * @param first First line of new closure.
 * @param last Last line of new closure.
 * TODO: use a proper constructor
 */
void FormClosures::NewClosure(Line *first, Line *last)
{
  Line *u = last;
  int b = bend[u->ID()];
  CompCostLargerThan comp(dist);
  Closure *new_c = new Closure(core);
  bool reached_branch_point = false;

  // At this point jct i is the jct between line i and prev[i] (same with coll).
  // Put lines (and jcts) into closure in reverse order (last in path is first
  // in list).
  // So after that jct i is the jct between line i-1 and line i.
  while(u != 0)
  {
    new_c->lines.PushBack(u);
    new_c->jcts.PushBack(jcts[u->ID()]);
    new_c->colls.PushBack(colls[u->ID()]);
    u->closures.PushBack(new_c);
    Line *p = prev[u->ID()];
    if(p != 0 && !reached_branch_point)
    {
      // If u has a single predecessor p and p has several extenions, there
      // might be another, longer path through p. Blacklist u, so that this
      // shortest path is not found again.
      assert(ins[u->ID()] > 0);
      if(ins[u->ID()] == 1 && exts[p->ID()] > 1)
      {
        if(debug) printf(
            "alternative path from line %d, adding %d to blacklist\n",
            p->ID(), u->ID());
        blacklist[u->ID()] = true;
        reached_branch_point = true;
      }
      // Note: If u has several predecessors, there might be a further, longer
      // path through u, so do not blacklist it. Continue until eventually
      // reaching the branching point where the several paths leading to u
      // separate, and blacklist there.
    }
    // If there was no branching and we reached the start node (which has p = 0)
    // blacklist the start node, essentially terminating any further search.
    else if(p == 0 && !reached_branch_point)
    {
      if(debug) printf(
          "reached start node, adding %d to blacklist\n",
          u->ID());
      blacklist[u->ID()] = true;
    }
    /*if(p != 0 && !reached_branch_point)
    {
      assert(exts[p->ID()] > 0);
      // if the predecessor has only one extension 
      if(exts[p->ID()] == 1)
      {
        if(debug) printf("re-adding to heap %d with INF cost\n", u->ID());
        ClearNode(u);
        assert(Q_size <= Q.size() - 1);
        Q[Q_size] = u;
        push_heap(Q.begin(), Q.begin() + Q_size, comp);
        Q_size++;
      }
      else
      {
        if(debug) printf("not re-adding to heap %d\n", u->ID());
        if(debug) printf(
            "alternative path from line %d, branches reduced from %d to %d\n",
            p->ID(), exts[p->ID()], exts[p->ID()] - 1);
        exts[p->ID()]--;
        reached_branch_point = true;
      }
    }*/
    u = p;
  }
  // if bend is left, we have to reverse the list to put lines into counter
  // clockwise order.
  if(b == LEFT)
  {
    new_c->lines.Reverse();
    new_c->jcts.Reverse();
    new_c->colls.Reverse();
  }
  // if bend is right, the above reverse order already put the lines in
  // counter clockwise order, but we have to shift junctions one forward
  else
  {
    LJunction *l_last = new_c->jcts.Last();
    Collinearity *c_last = new_c->colls.Last();
    for(unsigned i = new_c->lines.Size() - 1; i > 0; i--)
    {
      new_c->jcts[i] = new_c->jcts[i-1];
      new_c->colls[i] = new_c->colls[i-1];
    }
    new_c->jcts[0] = l_last;
    new_c->colls[0] = c_last;
  }
  // lines are now in counter-clockwise order, now find their senses
  // note:  jct i is the jct between line i and prev[i]
  new_c->senses.Resize(new_c->lines.Size());
  for(unsigned i = 0; i < new_c->lines.Size(); i++)
  {
    if(new_c->jcts[i] != 0)
    {
      // line is the RIGHT end of the L-jct
      if(new_c->jcts[i]->near_point[RIGHT] == START)
        new_c->senses[i] = SAME;
      else
        new_c->senses[i] = OPP;
    }
    else
    {
      if(new_c->colls[i]->WhichEndIs( new_c->lines[i]) == START)
        new_c->senses[i] = SAME;
      else
        new_c->senses[i] = OPP;
    }
  }
  new_c->CalculateSignificance();

  // TODO: does this really belong here...?
  //if(core->IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_SURFACES))
  //  SolveHCF();
  // HACK: collect statistics
//   core->stats.avg_clos_sig =
//     (core->stats.new_clos*core->stats.avg_clos_sig +
//      new_c->sig)/(double)(core->stats.new_clos + 1);
//   core->stats.new_clos++;
  // HACK END

  core->NewGestalt(GestaltPrinciple::FORM_CLOSURES, new_c);
  // now re-rank all closure
  // TODO: this has O(n^2 log n) complexity, which is bad ... However typically
  // the number of closures tends to be small. So we are fine for a while.
//   RankGestalts(Gestalt::CLOSURE, CmpClosures);
//   Mask();
}


/**
 * @brief Helper function to get the vertex point of whatever there is between two
 * lines.
 * @param j  L-jct between lines or UNDEF_ID
 * @param c  collinearity between lines or UNDEF_ID
 */
inline static Vector2 GetVertex(LJunction *j, Collinearity *c)
{
  if(j != 0 && c != 0)
    throw std::runtime_error("FormClosures: GetVertex: Need either L-jct or collinearity");
  else if(j != 0)
    return j->isct;
  else if(c != 0)
    return c->vertex;
  else
    throw std::runtime_error("FormClosures: GetVertex: Need one of L-jct or collinearity");
}

/**
 * @brief Helper function to get the vertex point of whatever there is between line
 * l and its previous line.
 * @param l_idx  line of interest
 * @param jcts  array of L-junctions of lines, containing some UNDEF_IDs
 * @param colls  array of collinearities of lines, containing some UNDEF_IDs
 */
inline static Vector2 GetVertex(Line *line, vector<LJunction*> &jcts,
    vector<Collinearity*> &colls)
{
  return GetVertex(jcts[line->ID()], colls[line->ID()]);
}

/**
 * @brief Check whether a new line with its new L-jct or collinearity intersects the
 * path so far.
 * @param l_last  new line
 * @param j_new  the L-jct which connects l_last to the path
 * @param c_new  the coll. which connects l_last to the path
 *               (one and only one of j_new, c_new must be != UNDEF)
 * The last line must not intersect geometrically with the previous lines in the
 * path.
 * @return Returns true for success
 */
bool FormClosures::LineIntersectingPath(Line *l_last, LJunction *j_new,
    Collinearity *c_new)
{
  // note that the first line we have to check is the next to last of the path
  Line *l = prev[l_last->ID()];
  Vector2 a, b, p, q;

  p = GetVertex(l_last, jcts, colls);
  q = GetVertex(j_new, c_new);
  while(l != 0 && prev[l->ID()] != 0)
  {
    try
    {
      a = GetVertex(prev[l->ID()], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch (exception &e)
    {
      // lines do not intersect, carry on
//       printf("FormClosures::LineIntersectingPath: unknown exception.\n");
//       std::cout << e.what() << std::endl;
    }
    l = prev[l->ID()];
  }
  return false;
}

/**
 * @brief Check whether the closing line of a possible closure intersects the
 * path so far.
 * @param l_close  the closing line = last on path
 * @param l_open  the opening line = first on path
 * @param j_init  the initialising L-jct between l_open and l_close, or UNDEF
 * @param c_init  the initialising coll. between l_open and l_close, or UNDEF
 * Note that one of j_init, c_init must be defined.
 * This function differs from LineIntersectingPath() by the fact that we do not
 * check against the opening line (which by definition must intersect with the
 * closing line).
 * @return Returns true for success
 */
bool FormClosures::ClosingLineIntersectingPath(Line *l_close,
    Line *l_open, LJunction *j_init, Collinearity *c_init)
{
  Line *l = prev[l_close->ID()];
  Vector2 a, b, p, q;

  p = GetVertex(l_close, jcts, colls);
  q = GetVertex(j_init, c_init);
  while(l != 0 && prev[l->ID()] != 0 && prev[l->ID()] != l_open)
  {
    try
    {
      a = GetVertex(prev[l->ID()], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch (exception &e)
    {
      // lines do not intersect, carry on
//       printf("FormClosures::ClosingLineIntersectingPath: unknown exception.\n");
//       std::cout << e.what() << std::endl;
    }
    l = prev[l->ID()];
  }
  return false;
}

/* TODO: check complete polygon for simple
bool FormClosures::SimplePolygon(unsigned l_close,
    unsigned l_open, unsigned j_init, unsigned c_init)
{
  if(!ClosingLineIntersectingPath(l_close, l_open, j_init, c_init))
  {
  }
}
*/

/**
 * @brief Mask closures.
 */
void FormClosures::Mask()
{
  // array containing for each line the ID of the highest ranking closure which
  // contains that line
  Array<unsigned> owner(NumLines(core));
  owner.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumClosures(core); j++)
  {
    Closure *clos = (Closure*)core->RankedGestalts(Gestalt::CLOSURE, j);
    clos->Mask(UNDEF_ID);
    for(unsigned l = 0; l < clos->lines.Size(); l++)
    {
      // first check if any previous (i.e. stronger) closure uses one of my
      // lines. if yes, let that closure mask me
      if(owner[clos->lines[l]->ID()] != UNDEF_ID)
      {
        // only mask if closures use line in same sense (one closure inside the
        // other), so first get line senses
        Closure *clos_j = Closures(core, owner[clos->lines[l]->ID()]);
        unsigned pos_i = l;
        unsigned pos_j = clos_j->lines.Find(clos->lines[pos_i]);
        bool same_sense = clos->senses[pos_i] == clos_j->senses[pos_j];
        if(same_sense)
          clos->Mask(owner[clos->lines[l]->ID()]);
      }
      // now the line belongs to me (and I might mask even lesser closures)
      owner[clos->lines[l]->ID()] = clos->ID();
    }
  }
}

/**
 * @brief Print the heap
 */
void FormClosures::PrintHeap()
{
  if(debug) printf("id   prev   exts  cost\n");
  for(size_t i = 0; i < Q_size; i++)
  {
    if(debug) printf("%3d %3d %3d %lf\n",
        Q[i]->ID(),
        (prev[Q[i]->ID()] != 0 ? prev[Q[i]->ID()]->ID() : -1),
        exts[Q[i]->ID()],
        dist[Q[i]->ID()]
        );
  }
  if(debug) printf("removed: -------------\n");
  for(size_t i = Q_size; i < Q.size(); i++)
  {
    if(debug) printf("%3d %3d %3d %lf\n",
        Q[i]->ID(),
        (prev[Q[i]->ID()] != 0 ? prev[Q[i]->ID()]->ID() : -1),
        exts[Q[i]->ID()],
        dist[Q[i]->ID()]
        );
  }
  if(debug) printf("----------------------\n");

}

}

