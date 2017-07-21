/**
 * @file FormConvexArcGroups.cc
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormConvexArcGroups.
 **/

#include <assert.h>
#include <algorithm>
#include <cstdio>
#include "Math.hh"
#include "Draw.hh"
#include "VisionCore.hh"
#include "Segment.hh"
#include "Arc.hh"
#include "AJunction.hh"
#include "ConvexArcGroup.hh"
#include "FormConvexArcGroups.hh"
#include "Ellipse.hh"

namespace Z
{

/* Determine whether two tangents belong to different same arcs.
 * Note that arc i has tangents 2*i and 2*i+1 */
#define FROM_DIFF_ARCS(i,j) ((i)/2 != (j)/2)

/* Returns true if one tangent is even and the other odd.
 */
#define DIFF_ODDITY(i,j) ((i)%2 != (j)%2)

static const int SEARCH_EXHAUSTIVE = 0;
static const int SEARCH_GREEDY = 1;

static const int CRIT_COCURV = 0;
static const int CRIT_YUEN = 1;
static const int CRIT_ELL = 2;

// debug thingy: peek at a specific group
unsigned peek = UNDEF_ID;

static int CmpConvexArcs(const void *a, const void *b)
{
  if( ((ConvexArcGroup*)a)->sig > ((ConvexArcGroup*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpFcnLinkSig(const void *a, const void *b)
{
  if( ((ArcLink*)a)->sig > ((ArcLink*)b)->sig )
    return -1;  // a is first
  else
    return 1;   // b is first
}

static bool Convex(Array<Arc*> &arcs, unsigned l, unsigned u, Arc *arc)
{
  for(unsigned i = l; i <= u; i++)
    if(!arcs[i]->ConvexWith(arc))
      return false;
  return true;
}

static bool Contains(Array<Arc*> &arcs, unsigned l, unsigned u, Arc *arc)
{
  for(unsigned i = l; i <= u; i++)
    if(arcs[i] == arc)
      return true;
  return false;
}


FormConvexArcGroups::FormConvexArcGroups(VisionCore *vc) : GestaltPrinciple(vc), arcs(1000)
{
  // COCURV is the default of: COCURV, YUEN, ELL
  core->GetConfig().AddItem("CONVEXARCS_STRONG_CONVEXITY", "1");
  // GREEDY is the default of: EXHAUSTIVE, GREEDY
  core->GetConfig().AddItem("CONVEXARCS_SEARCH_METHOD", "GREEDY");
  // COCURV is the default of: COCURV, YUEN, ELL
  core->GetConfig().AddItem("CONVEXARCS_GROUP_CRITERION", "COCURV"/*"ELL"*/);			/// HACK ARI: Geändert auf ELL!!!

// 	printf("################# ClearHashTable beim Start von FormConvexArcGroups #################\n");
	ClearHashTable();		/// HACK ARI
}

FormConvexArcGroups::~FormConvexArcGroups()
{
}

void FormConvexArcGroups::Reset()
{
	ClearHashTable();		/// HACK ARI: Hinzugefügt: Richtig?
}

void FormConvexArcGroups::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
	
  Arc::STRICT_CONVEXITY =
    core->GetConfig().GetValueInt("CONVEXARCS_STRONG_CONVEXITY");
  if(core->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION") == "COCURV")
    GROUP_CRITERION = CRIT_COCURV;
  else if(core->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION") == "YUEN")
    GROUP_CRITERION = CRIT_YUEN;
  else if(core->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION") == "ELL")
    GROUP_CRITERION = CRIT_ELL;
  else
  {
    fprintf(stderr, "* convex arcs: unkown criterion: %s\n",
      core->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION").c_str());
    return;
  }
  if(core->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD") == "EXHAUSTIVE")
    SEARCH_METHOD = SEARCH_EXHAUSTIVE;
  else if(core->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD") == "GREEDY")
    SEARCH_METHOD = SEARCH_GREEDY;
  else
  {
    fprintf(stderr, "* convex arcs: unkown search method: %s\n",
      core->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD").c_str());
    return;
  }

  switch(type)
  {
    case Gestalt::ARC:
      HaveNewArc(idx);
      break;
    case Gestalt::A_JUNCTION:
      HaveNewAJunction(idx);
      break;
    default:
      break;
  }
  
  StopRunTime();
}

void FormConvexArcGroups::HaveNewArc(unsigned idx)
{
  unsigned l = 0, u = 0, hash;
  arcs[l] = Arcs(core, idx);
  hash = Hash(arcs, l, u);
// printf("    l, u: %u, %u -- hash: %u -- hashtable: %u\n", l, u, hash, hashtable[hash]);
  if(hashtable[hash] == 0)
  {
    core->NewGestalt(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS, new ConvexArcGroup(core, arcs, l, u,
          Evaluate(arcs, l, u, GROUP_CRITERION)));
    hashtable[hash] = 1;
  }
}

void FormConvexArcGroups::HaveNewAJunction(unsigned idx)
{
  AJunction *ajct = AJunctions(core, idx);
  GreedySearch(ajct->arc[START], ajct->arc[END], arcs);
  /*if(SEARCH_METHOD == SEARCH_EXHAUSTIVE)
  {
    ExhaustiveSearch((Arc*)core->RankedGestalts(Gestalt::ARC, rank),
        arcs, arcs_opt);
  }
  else if(SEARCH_METHOD == SEARCH_GREEDY)
  {
  }*/
}

/**
 * Dijkstra shortest path algorithm
 * Find a path from right to left line taking only left turns.
 *
 * @param s  start line, which is the RIGHT arm of an L-jct
 * @param t  target line, which is the LEFT arm of that L-jct
 * @param end_s  the open end of line s (opposite to the joined end)
 * @param end_t  the open end of line t (opposite to the joined end)
 */
/*void FormConvexArcGroups::ShortestPaths(Arc *s, Arc *sn)
{
  Arc *u = 0;
  bool have_connected_vertices = true;
  CompCostLargerThan comp(dist);
  unsigned n = NumArcs(core);

  dist.resize(n);
  dist.assign(n, COST_INF);
  prev.resize(n);
  prev.assign(n, 0);
  next.resize(n);
  next.assign(n, 0);
  Q.resize(n);
  for(unsigned i = 0; i < n; i++)
    Q[i] = Arcs(core, i);
  // Distance from s to s is 0.
  dist[s->ID()] = 0.;
  // prepare min-heap (priority queue)
  make_heap(Q.begin(), Q.end(), comp);
  while(!Q.empty() && have_connected_vertices)
  {
    // Remove best vertex from priority queue
    u = Q.front();
    pop_heap(Q.begin(), Q.end(), comp);
    Q.pop_end();
    // If the best vertex has infinite distance (i.e. there is no path to it)
    // then there is no point in continuing
    if(dist[u->ID()] != COST_INF)
    {
      int cnt = 0;
      // For each edge (u,v) outgoing from u
      cnt += ExtendCollinearities(u);
      // distances have changed -> re-heap
      make_heap(Q.begin(), Q.end(), comp);
    }
    else
      have_connected_vertices = false;
  }
}*/

/*
void FormConvexArcGroups::ExhaustiveSearch(Arc *arc, Array<Arc*> &arcs,
    Array<Arc*> &arcs_opt)
{
  unsigned l = arcs.Size()/2, u = l, hash, size_opt ;
  //cands.Clear();
  arcs[l] = arc;
  arcs_opt[0] = arc;
  hash = Hash(arcs, l, u);
  NewCand(new ConvexArcGroup(core, arcs, l, u));
  hashtable[hash] = 1;
  GrowAll(arcs, l, u, arcs_opt, size_opt, Evaluate(arcs, l, u, CRIT_ELL));
  //InstantiateBestCand();
  // TODO: this more efficient version does not work yet
  //NewGestalt(new ConvexArcGroup(arcs_opt, 0, size_opt -1));
}*/

void FormConvexArcGroups::GreedySearch(Arc *arc, Arc *forced_next, Array<Arc*> &arcs)
{
  double sig;
  unsigned l = arcs.Size()/2, u = l, hash;
  arcs[l] = arc;
  if(forced_next != 0)
  {
    arcs[l + 1] = forced_next;
    u = l + 1;
  }
  sig = GrowBest(arcs, l, u);
  hash = Hash(arcs, l, u);
  if(hashtable[hash] == 0)
  {
    core->NewGestalt(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS, new ConvexArcGroup(core, arcs, l, u, sig));
    hashtable[hash] = 1;
  }
}

/*
void FormConvexArcGroups::Rank()
{
  RankGestalts(Gestalt::CONVEX_ARC_GROUP, CmpConvexArcs);
}
*/

/**
 * @brief Draw from the vote image.
 * @param detail Degree of detail.
 */
void FormConvexArcGroups::Draw(int detail)
{
}

/**
 * @brief Find the next end.
 * @param arcs
 * @param l
 * @param u
 * @param end
 * @param criterion
 */
Arc* FormConvexArcGroups::NextEnd(Array<Arc*> &arcs, unsigned l,
   unsigned u, int &end, int criterion)
{
  Arc *cand[2] = {0, 0}, *next = 0;
  double sig[2] = {-HUGE, -HUGE}, st;

  assert(l > 0 && u < arcs.Size() - 1);
  // find best admissible arc at START
  for(unsigned i = 0; i < arcs[l]->jct[START].Size(); i++)
  {
    Arc *t = arcs[l]->jct[START][i]->arc[START];
    if(!Contains(arcs, l, u, t) && Convex(arcs, l+1, u, t))
    {
      if(criterion == CRIT_COCURV)
      {
        cand[START] = t;
        sig[START] = arcs[l]->jct[START][i]->sig;
        break;  // links are sorted best sig first, so we are done
      }
      else
      {
        arcs[l-1] = t;
        st = Evaluate(arcs, l-1, u, criterion);
        if(st > sig[START])
        {
          cand[START] = t;
          sig[START] = st;
        }
      }
    }
  }
  // find best admissible arc at END
  for(unsigned i = 0; i < arcs[u]->jct[END].Size(); i++)
  {
    Arc *t = arcs[u]->jct[END][i]->arc[END];
    if(!Contains(arcs, l, u, t) && Convex(arcs, l, u-1, t))
    {
      if(criterion == CRIT_COCURV)
      {
        cand[END] = t;
        sig[END] = arcs[u]->jct[END][i]->sig;
        break;  // links are sorted best sig first, so we are done
      }
      else
      {
        arcs[u+1] = t;
        st = Evaluate(arcs, l, u+1, criterion);
        if(st > sig[END])
        {
          cand[END] = t;
          sig[END] = st;
        }
      }
    }
  }
  if(cand[START] == 0 && cand[END] == 0)
  {
    next = 0;
    end = STOP;
  }
  else
  {
    if(sig[START] > sig[END])
    {
      next = cand[START];
      end = START;
    }
    else
    {
      next = cand[END];
      end = END;
    }
  }
  return next;
}

double FormConvexArcGroups::Evaluate(Array<Arc*> &arcs, unsigned l, unsigned u, int criterion)
{
  if(criterion == CRIT_ELL)
  {
    double x, y, a, b, phi, error;
    FitEllipse(arcs, l, u, x, y, a, b, phi);
    return EllipseSupport(arcs, l, u, x, y, a, b, phi, error);
  }
  else // CRIT_YUEN or CRIT_COCURV
    return ArcGroupSignificance(core, arcs, l, u);
}

/**
 * @brief Grow end with best arc.
 * @param arcs
 * @param l
 * @param u
 * @return 
 */
double FormConvexArcGroups::GrowBest(Array<Arc*> &arcs, unsigned &l,
    unsigned &u)
{
  double q1;
  Arc *first, *last, *next;
  int cont;

  first = arcs[l];
  last = arcs[u];
  q1 = Evaluate(arcs, l, u, GROUP_CRITERION);
  next = NextEnd(arcs, l, u, cont, GROUP_CRITERION);
  if(cont != STOP)
  {
    unsigned l_try = l, u_try = u;
    double q2;
    if(cont == START)
    {
      assert(l_try > 0);
      l_try--;
      arcs[l_try] = next;
    }
    else
    {
      assert(u_try < arcs.Size() - 1);
      u_try++;
      arcs[u_try] = next;
    }
    q2 = GrowBest(arcs, l_try, u_try);
    if(q2 >= q1)
    {
      l = l_try;
      u = u_try;
      return q2;
    }
  }
  return q1;
}

#if 0
void FormConvexArcGroups::GrowAll(Array<Arc*> &arcs, unsigned l, unsigned u,
    Array<Arc*> &arcs_opt, unsigned &size_opt, double q_opt)
{
  unsigned i, s = 2*arcs[l], e = 2*arcs[u]+1, t, hash;
  unsigned MAX_HYP = 1000;
  // add all admissible arcs at START
  for(i = 0; i < min((unsigned)MAX_HYP,links[s].Size()); i++)
  {
    t = links[s][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l+1, u, t))
    {
      assert(l > 0);
      arcs[l-1] = t;
      hash = Hash(arcs, l-1, u);
      if(hashtable[hash] == 0)
      {
        // TODO: this more efficient version does not work yet
        /*q = Evaluate(arcs, l-1, u, CRIT_ELL);
        if(q > q_opt)
        {
          for(j = l-1, k = 0; j <= u; j++, k++)
            arcs_opt[k] = arcs[j];
          size_opt = u - l + 2;
          q_opt = q;
        }*/
        NewCand(new ConvexArcGroup(arcs, l-1, u));
        hashtable[hash] = 1;
        GrowAll(arcs, l-1, u, arcs_opt, size_opt, q_opt);
      }
    }
  }
  // add all admissible arcs at END
  for(i = 0; i < min((unsigned)MAX_HYP,links[e].Size()); i++)
  {
    t = links[e][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l, u-1, t))
    {
      assert(u < arcs.Size() - 1);
      arcs[u+1] = t;
      hash = Hash(arcs, l, u+1);
      if(hashtable[hash] == 0)
      {
        // TODO: this more efficient version does not work yet
        /*q = Evaluate(arcs, l, u+1, CRIT_ELL);
        if(q > q_opt)
        {
          for(j = l, k = 0; j <= u + 1; j++, k++)
            arcs_opt[k] = arcs[j];
          size_opt = u - l + 2;
          q_opt = q;
        }*/
        NewCand(new ConvexArcGroup(arcs, l, u+1));
        hashtable[hash] = 1;
        GrowAll(arcs, l, u+1, arcs_opt, size_opt, q_opt);
      }
    }
  }
}

void FormConvexArcGroups::NewCand(ConvexArcGroup *g)
{
  //cands.PushBack(g);
  //cands_created++;
  // forget about candidate list and instantiating best candidate, just create
  // all groups
  NewGestalt(g);
}

void FormConvexArcGroups::InstantiateBestCand()
{
  if(cands.Size() > 0)
  {
    ConvexArcGroup *g_opt = cands[0];
    double q, q_opt = Evaluate(g_opt->arcs, 0, g_opt->arcs.Size()-1, CRIT_ELL);
    for(unsigned i = 1; i < cands.Size(); i++)
    {
      q = Evaluate(cands[i]->arcs, 0, cands[i]->arcs.Size()-1, CRIT_ELL);
      if(q > q_opt)
      {
        q_opt = q;
        g_opt = cands[i];
      }
    }
    NewGestalt(g_opt);
    for(unsigned i = 0; i < cands.Size(); i++)
      if(cands[i] != g_opt)
        delete cands[i];
    cands.Clear();
  }
}
#endif

unsigned FormConvexArcGroups::Hash(Array<Arc*> &arcs)
{
  if(arcs.Size() > 0)
    return Hash(arcs, 0, arcs.Size()-1);
  else
    return 0;
}

/**
 * @brief Hash function.
 * The standard reference from Knuth's "The Art of Computer Programming",
 * volume 3 "Sorting and Searching", chapter 6.4.
 */
unsigned FormConvexArcGroups::Hash(Array<Arc*> &arcs, unsigned l, unsigned u)
{
  // first find one distinct arc in the group: maximum arc id
  unsigned i, i_max = l, max = arcs[l]->ID(), hash;
  for(i = l+1; i <= u; i++)
    if(arcs[i]->ID() > max)
    {
      i_max = i;
      max = arcs[i]->ID();
    }
  hash = u - l + 1;
  for(i = i_max; i <= u; i++) 
    hash = ((hash<<5)^(hash>>27))^arcs[i]->ID();
  for(i = l; i < i_max; i++) 
    hash = ((hash<<5)^(hash>>27))^arcs[i]->ID();
  hash = hash % HASH_SIZE;
  return hash;
}

void FormConvexArcGroups::ClearHashTable()
{
  for(int i = 0; i < HASH_SIZE; i++)
    hashtable[i] = 0;
}

#if 0
/**
 * Calculate significance between links i and j
 */
double FormConvexArcGroups::LinkSignificance(unsigned i, unsigned j)
{
  double r, theta, d_phi;
  double n = 2.*(double)NumArcs(), t;
  // make sure i is the link of the longer arc
  if(Arcs(i/2)->ArcLength() < Arcs(j/2)->ArcLength())
    swap(i, j);
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  Vector2 ti =
    Normalise((Arcs(i/2)->point[i%2] - Arcs(i/2)->center).NormalClockwise());
  Vector2 tj =
    Normalise((Arcs(j/2)->point[j%2] - Arcs(j/2)->center).NormalClockwise());
  r = v.Length();
  if(r > 0.)
  {
    v /= r;
    t = Cross(ti, v);
    t = fmax(t, -1.);
    t = fmin(t, 1.);
    theta = fabs(asin(t));
  }
  else
  {
    theta = M_PI_2;
    // avoid r == 0
    r = 1.;
  }
  // to avoid dphi = 0 and thus acc = 0, assume a minium of 2°
  d_phi = fmax(fabs(acos(Dot(ti, tj))), 0.03);
  // proximity factor
  double f_prox = theta*r*r/(double)VisionCore::ImageArea();
  // orientation factor
  double f_ori = d_phi/M_PI;
  double acc = 1. - exp(-n*f_prox*f_ori);
  double sig = -log(acc);
  return sig;

  /*
  // TODO: this seems to be a better sig than the nice Poisson probability above
  // the probability is too often = 0.0000000 for any larger gap
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  double r = max(1., v.Length());
  double sig = 1/r;
  return sig;*/

  /* old: i is the longer arc
  double s, g, l;
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  s = fabs(Cross(v, ti));
  g = fabs(Dot(v, ti));
  l = Arcs(i/2)->ArcLength();
  double theta = acos(Dot(ti, tj));
  double sig = -log(theta*s*g/pow(l, 2.));*/
}
#endif

}

