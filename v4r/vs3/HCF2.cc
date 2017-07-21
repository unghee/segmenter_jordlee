/**
 * $Id: HCF2.cc,v 1.7 2007/07/27 17:01:25 mxz Exp mxz $
 *
 * 0 is the uncommited label.
 * Increasing label corresponds to increasing depth.
 *
 * TODO: replace label_i with label, otherwise it's confusing with
 *       Closures(i)->label
 */

#include <algorithm>
#include <vector>
#include <set>
#include <iostream>
#include <iterator>
#include "Line.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "HCF.hh"

namespace Z
{

//#define HCF_DEBUG

class Stab
{
public:
  int site;
};

ostream& operator<<(ostream &os, const Stab &stab)
{
  os << Closures(stab.site)->stability;
  return os;
}

class ContourNeighborIter
{
private:
  unsigned c;  // this closure
  unsigned l;  // line counter
  unsigned n;  // neighbor counter
  unsigned nc_p; // previous neighbour closure (if closures share a collinear
                 // run of lines we only want to find one neighbour)
                 // If two contours however share two separate collinear runs we
                 // will count them twice as neighbours (this is rare anyway).

  void FindNeighbor()
  {
    bool found = false;  // found neighbor
    while(l < Closures(c)->lines.Size() && !found)
    {
      while(n < Lines(Closures(c)->lines[l])->closures.Size() && !found)
      {
        if(Lines(Closures(c)->lines[l])->closures[n] != c &&
           Lines(Closures(c)->lines[l])->closures[n] != nc_p)
        {
          found = true;
          nc_p = Lines(Closures(c)->lines[l])->closures[n];
        }
        else
          n++;
      }
      if(!found)
      {
        l++;
        n = 0;
      }
    }
  }

public:
  ContourNeighborIter(unsigned closure)
  {
    c = closure;
    l = n = 0;
    nc_p = UNDEF_ID;
    FindNeighbor();
  }
  void Next()
  {
    n++;
    FindNeighbor();
  }
  bool End()
  {
    return l >= Closures(c)->lines.Size();
  }
  unsigned Get()
  {
    return Lines(Closures(c)->lines[l])->closures[n];
  }
};

double Stability(int i);

/**
 * Comparison function object for heap.
 * Note that the default STL heap is a max-heap, i.e. the first element is the
 * maximum. The default STL heap uses the less-than operator '<'.
 * By having heap use this LargerThan operator we get a min-heap, which is
 * required by the HCF algorithm.
 */
class CompStabLargerThan
{
public:
  bool operator ()(const Stab &a, const Stab &b)
  {
    return Closures(a.site)->stability > Closures(b.site)->stability;
    //return Stability(a.site) > Stability(b.site);
  }
};

static int min_label = -10;  // HACK: arbitrary number of labels
static int max_label = +10;
static int propose_min_label = 0;
static int propose_max_label = 0;
static const int UNCOMMITTED = INT_MAX;
static const int DEFAULT_DEPTH = 0;
const int UNDEF_DEPTH = UNCOMMITTED;
const int MASKED = max_label;  // maximum depth
static vector<Stab> stab_heap;
static vector<int> stab_idx;

int MinDepth()
{
  return min_label;
}

int MaxDepth()
{
  return max_label;
}

int DefaultDepth()
{
  return DEFAULT_DEPTH;
}

int NumLabels()
{
  return max_label - min_label + 1;
}

void ProposeNewMinLabel(int new_min)
{
  propose_min_label = new_min;
}

void ProposeNewMaxLabel(int new_max)
{
  propose_max_label = new_max;
}

void AdjustLabelSet()
{
  if(propose_min_label < min_label)
    min_label = propose_min_label;
  if(propose_max_label > max_label)
    max_label = propose_max_label;
}

bool LabelIsUsed(int l)
{
  for(unsigned c = 0; c < NumClosures(); c++)
    if(Closures(c)->label == l)
      return true;
  return false;
}

// TODO: still errors
// TODO: this is inefficient
void CompactifyLabelSet()
{
  int hole_start = min_label;
  int hole_end = INT_MIN;
  bool used = false;
  bool used_prev = false;
  for(int l = min_label; l <= max_label; l++)
  {
    used = LabelIsUsed(l);
    if(!used && used_prev)
      hole_start = l;
    else if(used && !used_prev)
      hole_end = l;
    if(l == max_label && !used_prev)
      hole_end = l;
    if(hole_end > hole_start)
    {
      printf("label hole %d -> %d\n", hole_start, hole_end);
      // hole_start is the first unused label, hole_end the last unused label
      // move all previous labels up by hole width
      for(unsigned c = 0; c < NumClosures(); c++)
        if(Closures(c)->label <= hole_end)
          Closures(c)->label += (hole_end - hole_start + 1);
      min_label += (hole_end - hole_start + 1);
      hole_start = l;
      hole_end = INT_MIN;
    }
    used_prev = used;
  }
}

/**
 * Returns whether j is a neighbor of i.
 */
bool IsNeighbor(unsigned i, unsigned j)
{
  return Closures(i)->neighbors.find(j) != Closures(i)->neighbors.end();
}

int ClosureInside(unsigned i, unsigned j)
{
  return 1; // HACK
}

/**
 */
double DataEnergy(int i, int label_i)
{
  /*if(Closures(i)->label == UNCOMMITTED)
    return (double)abs(label_i - DEFAULT_DEPTH);
  else
  {
    int d = 0;
    if(label_i < min_label)
      d = min_label - label_i;
    else if(label_i > max_label)
      d = label_i - max_label;
    return (double)d;
  }*/

  // keeps label set tight by punishing moving away from the default depth.
  // but seems to limit it to about 6 labels, which is
  // not enough and eventualy does not allow conflict resolution.
  //return Closures(i)->sig*(double)abs(label_i - DEFAULT_DEPTH);

  // drive uncommitted sites towards default depth
  // does not restrict later growth of label set at all
  if(Closures(i)->label == UNCOMMITTED)
    return (double)abs(label_i - DEFAULT_DEPTH);
    //return Closures(i)->sig*(double)abs(label_i - DEFAULT_DEPTH);
  else
    return 0.;
}

/**
 * If line_i is a common line of clos_i and clos_j, but its previous line
 * is not a common line, line_i starts a collinear run.
 */
bool StartOfCollinearRun(unsigned clos_i, unsigned line_i, unsigned clos_j)
{
  unsigned line_p = Closures(clos_i)->lines.CircularPrev(line_i);
  return !Lines(Closures(clos_i)->lines[line_p])->closures.Contains(clos_j);
}

/**
 * If line_i is a common line of clos_i and clos_j, but its next line
 * is not a common line, line_i ends a collinear run.
 */
bool EndOfCollinearRun(unsigned clos_i, unsigned line_i, unsigned clos_j)
{
  unsigned line_n = Closures(clos_i)->lines.CircularNext(line_i);
  return !Lines(Closures(clos_i)->lines[line_n])->closures.Contains(clos_j);
}

/**
 * pos_i is the position of a common line along closure i, pos_j the position
 * of that line along closure j.
 * We assume that the common line is traveresed in different directions along
 * both closures, i.e. closures lie aside.
 * Then if the previous line on i is not the same as the next line on j, that
 * line is the start of a common run of lines (start as seen from closure i).
 * TODO: make distinction between same and different sense
 */
bool StartOfCollinearRun(unsigned clos_i, unsigned pos_i, unsigned clos_j,
    unsigned pos_j)
{
  unsigned pos_ip = Closures(clos_i)->lines.CircularPrev(pos_i);
  unsigned pos_jn = Closures(clos_j)->lines.CircularNext(pos_j);
  return Closures(clos_i)->lines[pos_ip] != Closures(clos_j)->lines[pos_jn];
}

/**
 * pos_i is the position of a common line along closure i, pos_j the position
 * of that line along closure j.
 * We assume that the common line is traveresed in different directions along
 * both closures, i.e. closures lie aside.
 * Then if the next line on i is not the same as the previous line on j, that
 * line is the end of a common run of lines (end as seen from closure i).
 */
bool EndOfCollinearRun(unsigned clos_i, unsigned pos_i, unsigned clos_j,
    unsigned pos_j)
{
  unsigned pos_in = Closures(clos_i)->lines.CircularNext(pos_i);
  unsigned pos_jp = Closures(clos_j)->lines.CircularPrev(pos_j);
  return Closures(clos_i)->lines[pos_in] != Closures(clos_j)->lines[pos_jp];
}

double ContourEnergies(int i, int label_i)
{
  double sum_energies = 0.;
  for(map<unsigned, double>::iterator it = Closures(i)->neighbors_aside.begin();
      it != Closures(i)->neighbors_aside.end(); ++it)
    if(label_i != Closures(it->first)->label)
      sum_energies += it->second;
  /*for(map<unsigned, double>::iterator it = Closures(i)->neighbors_outside.begin();
      it != Closures(i)->neighbors_outside.end(); ++it)
    if(label_i != MASKED && Closures(it->first)->label != MASKED)
      sum_energies += it->second;*/
  return sum_energies;
}

double TEnergies(int i, int label_i)
{
  double sum_energies = 0.;
  for(map<unsigned, double>::iterator it = Closures(i)->neighbors_above.begin();
      it != Closures(i)->neighbors_above.end(); ++it)
    // if label i not below label of above neighbor
    if(label_i <= Closures(it->first)->label)
      sum_energies += it->second;
  return sum_energies;
}

double PTEnergies(int i, int label_i)
{
  double sum_energies = 0.;
  for(map<unsigned, double>::iterator it = Closures(i)->neighbors_below.begin();
      it != Closures(i)->neighbors_below.end(); ++it)
    // if label i not above label of above neighbor
    if(label_i >= Closures(it->first)->label)
      sum_energies += it->second;
  return sum_energies;
}

double CliqueEnergies(int i, int label_i)
{
  double energy = 0.;
  // only if site i (and therefore cliques containing i) is committed
  if(Closures(i)->label != UNCOMMITTED)
  {
    // 2-cliques
    energy += ContourEnergies(i, label_i);
    energy += TEnergies(i, label_i);
    energy += PTEnergies(i, label_i);
  }
  return energy;
}

double Energy(int i, int label_i)
{
  double energy = DataEnergy(i, label_i) + CliqueEnergies(i, label_i);
  return energy;
}

double StabilityUncommitted(int i)
{
  double e, e_min1 = HUGE, e_min2 = HUGE;  // smallest and second smallest eng.
  // always search one label further to see if label set needs extension
  //for(int l = min_label - 1; l <= max_label + 1; l++)
  for(int l = min_label; l <= max_label; l++)
  {
    e = Energy(i, l);
    if(e < e_min1)
    {
      e_min2 = e_min1;
      e_min1 = e;
    }
    else if(e < e_min2)
      e_min2 = e;
  }
  return e_min1 - e_min2;
}

double StabilityCommitted(int i)
{
  double e, e_min = HUGE;  // smallest energy
  // always search one label further to see if label set needs extension
  //for(int l = min_label - 1; l <= max_label + 1; l++)
  for(int l = min_label; l <= max_label; l++)
  {
    if(l != Closures(i)->label)
    {
      e = Energy(i, l);
      if(e < e_min)
        e_min = e;
    }
  }
  return e_min - Closures(i)->energy;
  //return e_min - Energy(i, Closures(i)->label);
}

/**
 * note: energy of site i must be up-to-date!
 */
double Stability(int i)
{
  if(Closures(i)->label != UNCOMMITTED)
    return StabilityCommitted(i);
  else
    return StabilityUncommitted(i);
}

void ChangeStateUncommitted(int i)
{
  double e, e_min = HUGE;  // smallest energy
  int l_min = UNCOMMITTED;  // label of smallest energy
  // always search one label further to see if label set needs extension
  //for(int l = min_label - 1; l <= max_label + 1; l++)
  for(int l = min_label; l <= max_label; l++)
  {
    e = Energy(i, l);
    if(e < e_min)
    {
      e_min = e;
      l_min = l;
    }
  }
  Closures(i)->label = l_min;
  // if the minimum label is not yet in the label set, extend label set
  if(l_min == min_label - 1)
    ProposeNewMinLabel(min_label - 1);
  else if(l_min == max_label + 1)
    ProposeNewMaxLabel(max_label + 1);
}

/**
 * TODO: output e_min, so does not have to be recalculated
 */
void ChangeStateCommitted(int i)
{
  double e, e_min = HUGE;  // smallest energy
  int l_min = UNCOMMITTED;  // label at smallest energy
  // always search one label further to see if label set needs extension
  //for(int l = min_label - 1; l <= max_label + 1; l++)
  for(int l = min_label; l <= max_label; l++)
  {
    if(l != Closures(i)->label)
    {
      e = Energy(i, l);
      if(e < e_min)
      {
        e_min = e;
        l_min = l;
      }
    }
  }
  Closures(i)->label = l_min;
  // if the minimum label is not yet in the label set, extend label set
  if(l_min == min_label - 1)
    ProposeNewMinLabel(min_label - 1);
  else if(l_min == max_label + 1)
    ProposeNewMaxLabel(max_label + 1);
}

void ChangeState(int i)
{
  if(Closures(i)->label != UNCOMMITTED)
    ChangeStateCommitted(i);
  else
    ChangeStateUncommitted(i);
}

/**
 * As energy we take the smaller of the two possible from the two junctions
 * giving rise to the common contour.
 * TODO: A closure i should not be outside contour neighbour of a closure j and
 * later (as search lines grow to the other side of the perhaps narrow j) also
 * become inside neighbour. Of course significanes of the later junctions would
 * be very small anywa.
 */
void FindContourNeighbors(unsigned i)
{
  for(unsigned l = 0; l < Closures(i)->lines.Size(); l++)
    for(unsigned n = 0; n < Lines(Closures(i)->lines[l])->closures.Size(); n++)
    {
      unsigned j = Lines(Closures(i)->lines[l])->closures[n];
      // i is not its own neighbour
      if(j != i)
      {
        // now i and j are neighbours via common line l
        unsigned pos_i = l;
        // position of line on closure j
        unsigned pos_j = Closures(j)->lines.Find(Closures(i)->lines[l]);
        bool same_sense = Closures(i)->senses[l] == Closures(j)->senses[pos_j];
        // if not same sens, the closure lie alongside
        if(!same_sense)
        {
          double energy = 0.;
          if(StartOfCollinearRun(i, l, j, pos_j))
          {
            // count smaller energy of start junction of line on i and end
            // junction of line on j
            unsigned pos_jn = Closures(j)->lines.CircularNext(pos_j);
            double energy_i = Closures(i)->GetJunctionSig(l);
            double energy_j = Closures(j)->GetJunctionSig(pos_jn);
            energy += min(energy_i, energy_j);
          }
          if(EndOfCollinearRun(i, l, j, pos_j))
          {
            // count smaller energy of end junction of line on i and start
            // junction of line on j
            unsigned pos_in = Closures(i)->lines.CircularNext(l);
            double energy_i = Closures(i)->GetJunctionSig(pos_in);
            double energy_j = Closures(j)->GetJunctionSig(pos_j);
            energy += min(energy_i, energy_j);
          }
          Closures(i)->neighbors_aside[j] += energy;
          Closures(i)->neighbors.insert(j);
        }
        else  // same sense
        {
          double energy = 0.;
          unsigned pos_ip = Closures(i)->lines.CircularPrev(pos_i);
          unsigned pos_jp = Closures(j)->lines.CircularPrev(pos_j);
          unsigned pos_in = Closures(i)->lines.CircularNext(pos_i);
          unsigned pos_jn = Closures(j)->lines.CircularNext(pos_j);
          unsigned pos_inn = Closures(i)->lines.CircularNext(pos_in);
          unsigned pos_jnn = Closures(j)->lines.CircularNext(pos_jn);
          // if start of collinear run
          if(Closures(i)->lines[pos_ip] != Closures(j)->lines[pos_jp])
          {
            // get first vertex, which is not shared between closures i and j
            Vector2 v_ip = Closures(i)->GetVertex(pos_ip);
            // count smaller energy of start junction of line on i and end
            // junction of line on j
            double energy_i = Closures(i)->GetJunctionSig(pos_i);
            double energy_j = Closures(j)->GetJunctionSig(pos_j);
            energy += min(energy_i, energy_j);
            // if i is inside j, consider i above j
            if(Closures(j)->Inside(v_ip))
            {
              Closures(i)->neighbors_outside[j] = 0;
              Closures(i)->neighbors_below[j] += energy;
              Closures(i)->neighbors.insert(j);
            }
            else // i is is outside j, consider j above i
            {
              Closures(i)->neighbors_inside[j] = 0;
              Closures(i)->neighbors_above[j] += energy;
              Closures(i)->neighbors.insert(j);
            }
          }
          // if end of collinear run
          if(Closures(i)->lines[pos_in] != Closures(j)->lines[pos_jn])
          {
            // get first vertex, which is not shared between closures i and j
            Vector2 v_inn = Closures(i)->GetVertex(pos_inn);
            // count smaller energy of end junction of line on i and start
            // junction of line on j
            double energy_i = Closures(i)->GetJunctionSig(pos_in);
            double energy_j = Closures(j)->GetJunctionSig(pos_jn);
            energy += min(energy_i, energy_j);
            // if i is inside j, consider i above j
            if(Closures(j)->Inside(v_inn))
            {
              Closures(i)->neighbors_outside[j] = 0;
              Closures(i)->neighbors_below[j] += energy;
              Closures(i)->neighbors.insert(j);
            }
            else // i is is outside j, consider j above i
            {
              Closures(i)->neighbors_inside[j] = 0;
              Closures(i)->neighbors_above[j] += energy;
              Closures(i)->neighbors.insert(j);
            }
          }
          // TODO: handle case where i and j are incompatible
          /*
          //if(Closures(i)->sig < Closures(j)->sig)
          //  ...  HACK
          Closures(i)->neighbors_outside[j] = min(Closures(i)->sig,
              Closures(j)->sig);
          Closures(j)->neighbors_outside[i] = Closures(i)->neighbors_outside[j];*/
        }
      }
    }
}

/**
 * TODO: maybe set energy of C-neighbors which have the bar of a T to 0 instead
 * of adding significances of L-junctions.
 */
void FindTNeighbors(unsigned i)
{
  for(unsigned l = 0; l < Closures(i)->lines.Size(); l++)
    for(unsigned end = START; end <= END; end++)
    {
      unsigned t = Lines(Closures(i)->lines[l])->t_jct[end];
      // if line l is the pole of a T-junction
      if(t != UNDEF_ID)
      {
        unsigned left = TJunctions(t)->line[LEFT];
        unsigned right = TJunctions(t)->line[RIGHT];
        // closures must share some contour line (left or right bar of T-jct)
        //if(left == Closures(i)->lines.CircularNext(l) ||
        //   left == Closures(i)->lines.CircularPrev(l) ||
        //   right == Closures(i)->lines.CircularNext(l) ||
        //   right == Closures(i)->lines.CircularPrev(l))
        if(Closures(i)->lines.Contains(left) ||
           Closures(i)->lines.Contains(right))
        {
          // all closures j of which line 'left' and 'right' are part, occlude
          // closure i
          for(unsigned n = 0; n < Lines(left)->closures.Size(); n++)
          {
            unsigned j = Lines(left)->closures[n];
            // i is not its own neighbour
            if(j != i)
            {
              unsigned pos_l = Closures(j)->lines.Find(left);
              unsigned next = Closures(j)->lines.CircularNext(pos_l);
              unsigned prev = Closures(j)->lines.CircularPrev(pos_l);
              // if right line of T-jct is the next line of closure j
              // we have a T-jct from outside, and j genuinely occludes i,
              // j is above i.
              if(right == Closures(j)->lines[next])
              {
                // note that we add the energies of the L-jcts to compensate for
                // the fact that these L-jcts will vote for same depth as
                // contour neighbors.
                double energy = TJunctions(t)->sig +
                  LJunctions(TJunctions(t)->ljct[LEFT])->sig +
                  LJunctions(TJunctions(t)->ljct[RIGHT])->sig;
                Closures(i)->neighbors_above[j] += energy;
                Closures(j)->neighbors_below[i] += energy;
                Closures(i)->neighbors.insert(j);
                Closures(j)->neighbors.insert(i);
              }
              // if right line of T-jct is the previous line of closure j
              // we have a T-jct from inside and i lies "inside" j,
              // j lies outside i.
              else if(right == Closures(j)->lines[prev])
              {
                double energy = TJunctions(t)->sig +
                  LJunctions(TJunctions(t)->ljct[LEFT])->sig +
                  LJunctions(TJunctions(t)->ljct[RIGHT])->sig;
                Closures(i)->neighbors_below[j] += energy;
                Closures(j)->neighbors_above[i] += energy;
                Closures(i)->neighbors.insert(j);
                Closures(j)->neighbors.insert(i);
              }
            }
          }
        }
      }
    }
}

void FindPTNeighbors(unsigned i, set<unsigned> &neighbors)
{
  for(unsigned l = 0; l < Closures(i)->lines.Size(); l++)
    for(unsigned end = START; end <= END; end++)
    {
      for(unsigned side = LEFT; side <= RIGHT; side++)
      {
        for(unsigned k = 0;
            k < Lines(Closures(i)->lines[l])->pt_jct[end][side].Size(); k++)
        {
          // line l is a bar of a T-junction
          unsigned t = Lines(Closures(i)->lines[l])->pt_jct[end][side][k];
          unsigned pole = TJunctions(t)->line[POLE];
          // the other bar line
          unsigned other = TJunctions(t)->line[Other(side)];
          unsigned next = Closures(i)->lines.CircularNext(l);
          // only if both bars are part of closure i do we have a passive T-jct
          // of closure i
          if(other == Closures(i)->lines[next])
          {
            // all closures j of which line 'pole' is part, are occluded by
            // closure i
            for(unsigned n = 0; n < Lines(pole)->closures.Size(); n++)
            {
              unsigned j = Lines(pole)->closures[n];
              // closures must share a line (left or right bar of T-jct)
              //if(Closures(j)->lines.Contains(Closures(i)->lines[l]) ||
              //   Closures(j)->lines.Contains(other))
              {
                // i is not its own neighbour
                if(j != i)
                {
                  // if line l is LEFT bar and l+1 is RIGHT bar, we have a T-jct
                  // from outside, and i genuinely occludes j.
                  if(side == LEFT)
                  {
                    double energy = TJunctions(t)->sig +
                      LJunctions(TJunctions(t)->ljct[LEFT])->sig +
                      LJunctions(TJunctions(t)->ljct[RIGHT])->sig;
                    Closures(i)->neighbors_above[j] += energy;
                    Closures(j)->neighbors_below[i] += energy;
                    Closures(i)->neighbors.insert(j);
                    Closures(j)->neighbors.insert(i);
                  }
                  // if l is RIGHT bar and l+1 is LEFT bar, we have a T-jct
                  // from inside, and j lies "inside" i. j could e.g. be a
                  // surface marking. we consider it above i.
                  else
                  {
                    double energy = TJunctions(t)->sig +
                      LJunctions(TJunctions(t)->ljct[LEFT])->sig +
                      LJunctions(TJunctions(t)->ljct[RIGHT])->sig;
                    Closures(i)->neighbors_outside[j] += energy;
                    Closures(j)->neighbors_inside[i] += energy;
                    Closures(i)->neighbors.insert(j);
                    Closures(j)->neighbors.insert(i);
                  }
                }
              }
            }
          }
        }
      }
    }
}

void ClearNeighbors(unsigned i)
{
  Closures(i)->neighbors.clear();
  Closures(i)->neighbors_aside.clear();
  Closures(i)->neighbors_above.clear();
  Closures(i)->neighbors_below.clear();
  Closures(i)->neighbors_inside.clear();
  Closures(i)->neighbors_outside.clear();
}

void FindNeighbors(unsigned i)
{
  FindContourNeighbors(i);
  FindTNeighbors(i);
  //FindPTNeighbors(i);  is covered by TNeighbors
}

void UpdateNeighborEnergies(int i)
{
  for(set<unsigned>::iterator it = Closures(i)->neighbors.begin();
      it != Closures(i)->neighbors.end(); ++it)
  {
    Closures(*it)->energy = Energy(*it, Closures(*it)->label);
#ifdef HCF_DEBUG
    printf("updated neighbor %d energy to %f\n", *it, Closures(*it)->energy);
#endif
  }
}

void UpdateNeighborStabilities(int i)
{
  for(set<unsigned>::iterator it = Closures(i)->neighbors.begin();
      it != Closures(i)->neighbors.end(); ++it)
  {
    Closures(*it)->stability = Stability(*it);
#ifdef HCF_DEBUG
    printf("updated neighbor %d stab to %f\n", *it, Closures(*it)->stability);
#endif
  }
}

/**
 * TODO: don't need to clear and refill every time
 */
void CreateHeap()
{
  CompStabLargerThan comp;
  stab_heap.clear();
  for(unsigned i = 0; i < NumClosures(); i++)
  {
    Stab s;
    s.site = i;
    stab_heap.push_back(s);
    push_heap(stab_heap.begin(), stab_heap.end(), comp);
  }
}

void InitSite(int i)
{
  Closures(i)->label = UNCOMMITTED;
  Closures(i)->energy = Energy(i, Closures(i)->label);
  Closures(i)->stability = Stability(i);
#ifdef HCF_DEBUG
  printf("init clos %d: e: %f s: %f\n", i, Closures(i)->energy,
      Closures(i)->stability);
#endif
}

void UpdateAllNeighborhoods()
{
  for(unsigned i = 0; i < NumClosures(); i++)
    ClearNeighbors(i);
  for(unsigned i = 0; i < NumClosures(); i++)
    FindNeighbors(i);
}

void UpdateAllEnergies()
{
  for(unsigned i = 0; i < NumClosures(); i++)
    Closures(i)->energy = Energy(i, Closures(i)->label);
}

void UpdateAllStabilities()
{
  for(unsigned i = 0; i < NumClosures(); i++)
    Closures(i)->stability = Stability(i);
}

void LogSites()
{

}

double GlobalEnergy()
{
  double energy = 0.;
  for(unsigned i = 0; i < NumClosures(); i++)
    energy += Closures(i)->energy;
  return energy;
}

void SolveHCF()
{
  bool improvement = true;
  int iter = 0;
  int iter_max = max(10*(int)NumClosures(), 10*NumLabels());
  static int n_sites_prev = 0;
  int n_sites = NumClosures();

  // initialise all new sites, added since the last call
  for(int i = n_sites_prev; i < n_sites; i++)
    InitSite(i);
  n_sites_prev = n_sites;

  // new junctions might have been created since last call, so neighborhoods,
  // energies and stabilites might have changed
  UpdateAllNeighborhoods();
  UpdateAllEnergies();
  UpdateAllStabilities();

  // prepare stability heap
  CreateHeap();

  while(improvement && iter < iter_max)
  {
    iter++;
#ifdef HCF_DEBUG
    printf("iter: %d\n", iter);
#endif
    CompStabLargerThan comp;
    Stab s = stab_heap.front();
    int i = s.site;
    if(Closures(i)->stability < 0.)
    {
#ifdef HCF_DEBUG
      printf("chose clos %d with label %d stability %f\n", i,
         Closures(i)->label, Closures(i)->stability);
#endif
      ChangeState(i);
      Closures(i)->energy = Energy(i, Closures(i)->label);
      Closures(i)->stability = Stability(i);
#ifdef HCF_DEBUG
  printf("clos %d changed to label %d, e: %f s: %f\n", i, Closures(i)->label,
      Closures(i)->energy, Closures(i)->stability);
#endif
      UpdateNeighborEnergies(i);
      UpdateNeighborStabilities(i);
      make_heap(stab_heap.begin(), stab_heap.end(), comp);
    }
    else
      improvement = false;
#ifdef HCF_DEBUG
    if(iter%100 == 0)
      printf("HCF iter %d\n", iter);
#endif
    // printf("  global energy: %.6f\n", GlobalEnergy());
  }
#ifdef HCF_DEBUG
  if(iter >= iter_max)
    printf("HCF max iter reached\n");
#endif
  //AdjustLabelSet();
#ifdef HCF_DEBUG
  printf("num labels: %d\n", NumLabels());
#endif
  //CompactifyLabelSet();
  //printf("  after compactify: %d\n", NumLabels());
}

// HACK
void PrintStabilityCommitted(int i)
{
  double e, e_min = HUGE;  // smallest energy
  // always search one label further to see if label set needs extension
  for(int l = min_label - 1; l <= max_label + 1; l++)
  //for(int l = min_label; l <= max_label; l++)
  {
    if(l != Closures(i)->label)
    {
      e = Energy(i, l);
      printf("energy(%d) = %f\n", l, e);
      if(e < e_min)
        e_min = e;
    }
  }
      printf("stability = %f - %f = %f\n", e_min, Closures(i)->energy,
          e_min - Closures(i)->energy);
  //return e_min - Closures(i)->energy;
  //return e_min - Energy(i, Closures(i)->label);
}

}


