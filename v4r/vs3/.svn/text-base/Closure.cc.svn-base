/**
 * @file Closure.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt Closure.
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "TJunction.hh"
#include "Closure.hh"
// #include "HCF.hh"
#include <cstdio>

namespace Z
{
// HACK: this belongs to HCF2.cc, which is at the moment not built
// static int min_label = -10;  // arbitrary number of labels
// static int max_label = +10;
// static const int UNCOMMITTED = INT_MAX;
// static const int DEFAULT_DEPTH = 0;
// const int UNDEF_DEPTH = UNCOMMITTED;
// const int MASKED = max_label;  // maximum depth

// int MinDepth()
// {
//   return min_label;
// }
// 
// int MaxDepth()
// {
//   return max_label;
// }
// 
// int DefaultDepth()
// {
//   return DEFAULT_DEPTH;
// }
// 
// int NumLabels()
// {
//   return max_label - min_label + 1;
// }

// HACK END

/**
 * @brief Constructor of class Closure.
 * @param c Vision core.
 */
Closure::Closure(VisionCore *c) : Gestalt(c, CLOSURE)
{
  stability = 0.;
  energy = 0.;
  label = 0;/*DefaultDepth();*/			// for HCF2
}

/**
 * @brief Get vertex: Returns the intersection point of a junction or a collinearity.
 * @param i ID of the junction (wheter l-junction or collinearity)
 * @return Returns the intersection point of a junction.
 */
Vector2 Closure::GetVertex(unsigned i)
{
  if(jcts[i] != 0 && colls[i] != 0)
    throw std::runtime_error("Closure::GetVertex: Need either L-jct or collinearity");
  else if(jcts[i] != 0)
    return jcts[i]->isct;
  else if(colls[i] != 0)
    return colls[i]->vertex;
  else
    throw std::runtime_error("Closure::GetVertex: need one of L-jct or collinearity");
}

/**
 * @param Get Signifacance value, calculated from the junctions.
 * @param i ID of the junction
 * @return Returns the significance value of a junction.
 */
double Closure::GetJunctionSig(unsigned i)
{
  if(jcts[i] != 0 && colls[i] != 0)
    throw std::runtime_error("Closure::GetJunctionSig: Need either L-jct or collinearity");
  else if(jcts[i] != 0)
    return jcts[i]->sig;
  else if(colls[i] != 0)
    return colls[i]->sig;
  else
    throw std::runtime_error("Closure::GetJunctionSig: Need one of L-jct or collinearity");
}

/**
 * @brief Draw closures and additional information into an image.
 * @param detail Degree of detail
 */
void Closure::Draw(int detail)
{
  if(detail == 0)
  {
    // note: there are as many lines as junctions
    for(unsigned i = 0; i < lines.Size(); i++)
    {
      unsigned j = (i < lines.Size()-1 ? i+1 : 0);
      Vector2 p, q, r, s;
      if(senses[i] == SAME)
      {
        p = lines[i]->point[START];
        q = lines[i]->point[END];
      }
      else
      {
        p = lines[i]->point[END];
        q = lines[i]->point[START];
      }
      DrawLine2D(p.x, p.y, q.x, q.y);
      r = GetVertex(j);
      DrawLine2D(q.x, q.y, r.x, r.y);
      if(senses[j] == SAME)
        s = lines[j]->point[START];
      else
        s = lines[j]->point[END];
      DrawLine2D(r.x, r.y, s.x, s.y);
    }
  }
  if(detail >= 1 && detail <= 2)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
      lines[i]->Draw(detail - 1);
  }
  if(detail >= 3)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
    {
      if(jcts[i] != 0)
        jcts[i]->Draw(detail - 3);
      else if(colls[i] != 0)
        colls[i]->Draw(detail - 3);
    }
  }
}

//extern void PrintStabilityCommitted(int i);  // HACK 

/**
 * @brief Return information about feature as string.
 * @return Returns a string with all information about the Gestalt
 */
const char* Closure::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;

  n += snprintf(info_text + n, info_size - n, "%s  %u lines:",
      Gestalt::GetInfo(), lines.Size());
  for(unsigned i = 0; i < lines.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %c%u",
      (senses[i] == SAME ? '+' : '-'), lines[i]->ID());
  n += snprintf(info_text + n, info_size - n, "\n ");

  for(unsigned i = 0; i < lines.Size(); i++)
    if(jcts[i] != 0)
      n += snprintf(info_text + n, info_size - n, " L(%d)", jcts[i]->ID());
    else if(colls[i] != 0)
      n += snprintf(info_text + n, info_size - n, " C(%d)", colls[i]->ID());
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "  neighbors:");
  for(set<Closure*>::iterator it = neighbors.begin();
      it != neighbors.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d", (*it)->ID());
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "  aside:");
  for(map<Closure*, double>::iterator it = neighbors_aside.begin();
      it != neighbors_aside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  above:");
  for(map<Closure*, double>::iterator it = neighbors_above.begin();
      it != neighbors_above.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  below:");
  for(map<Closure*, double>::iterator it = neighbors_below.begin();
      it != neighbors_below.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  inside:");
  for(map<Closure*, double>::iterator it = neighbors_inside.begin();
      it != neighbors_inside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  outside:");
  for(map<Closure*, double>::iterator it = neighbors_outside.begin();
      it != neighbors_outside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n,
      "  label: %d energy: %f stability: %f\n", label, energy, stability);
  //PrintStabilityCommitted(id);  // HACK
  return info_text;
}

/**
 * 
 * @param x 
 * @param y 
 * @return 
 */
bool Closure::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    if(lines[i]->IsAtPosition(x, y))
      return true;
  return false;
}

/**
 * @brief Calculate significance of closures.
 * TODO Different implementations for significance calculation.
 * 1. Signifacance is relation between number of gap-pixels and gap+line pixels
 * 2. Significance is equal to the sum line significances
 * 3. Significance is equal to the area of the closure
 */
void Closure::CalculateSignificance()
{
  double l = SumLines();
  double g = SumGaps();
  sig = l/(g+l);
// printf("Sig of Closure 1: %4.2f\n", sig);

  /*sig = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    sig += lines[i]->sig;
    sig += GetJunctionSig(i);
  }*/
  
//   sig = Area();
// printf("Sig of Closure 2: %4.2f\n", sig);
}

/**
 * @brief Returns the number of L-Junctions.
 * @return Returns the number of L-Junctions.
 */
unsigned Closure::NumLJunctions()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < jcts.Size(); i++)
  {
    if(jcts[i] != 0)
      cnt++;
	}
  return cnt;
}

/**
 * @brief Returns the number of Collinearities.
 * @return Returns the number of Collinearities.
 */
unsigned Closure::NumCollinearities()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < colls.Size(); i++)
    if(colls[i] != 0)
      cnt++;
  return cnt;
}

/**
 * @brief Calculate the sum of all gaps between the single lines.
 * @return Returns the sum of all geps between the single lines in pixels.
 */
double Closure::SumGaps()
{
  double sum = 0.;
  // note: there are as many lines as junctions
  for(unsigned i = 0; i < lines.Size(); i++) {
    if(jcts[i] != 0)
      sum += jcts[i]->r;          // TODO: also use corrected distance here
    else if(colls[i] != 0)
      sum += colls[i]->gap_cor;
  }
  return sum;
}

/**
 * @brief Returns the sum of the length of all lines.
 * @return Returns the sum of the length of all lines.
 */
double Closure::SumLines()
{
  double sum = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
    sum += lines[i]->Length();
  return sum;
}

/**
 * @brief Circumference
 * @return Returns the calculated circumference 
 */
double Closure::Circumference()
{
  double circ = 0.;
  // note: there are as many lines as junctions
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    unsigned j = (i < lines.Size()-1 ? i+1 : 0);
    Vector2 p = GetVertex(i);
    Vector2 q = GetVertex(j);
    circ += Distance(p, q);
  }
  return circ;
}

/**
 * @brief Find the neighbors of the closure.
 */
void Closure::FindTNeighbors()
{
  for(unsigned l = 0; l < lines.Size(); l++)
    for(int end = START; end <= END; end++)
    {
      TJunction *t = lines[l]->t_jct[end];
      // if line l is the pole of a T-junction
      if(t != 0)
      {
        Line *left = t->line[LEFT];
        Line *right = t->line[RIGHT];
        // closures must share some contour line (left or right bar of T-jct)
        //if(left == lines.CircularNext(l) ||
        //   left == lines.CircularPrev(l) ||
        //   right == lines.CircularNext(l) ||
        //   right == lines.CircularPrev(l))
        if(lines.Contains(left) || lines.Contains(right))
        {
          // all closures j of which line 'left' and 'right' are part, occlude
          // this closur
          for(unsigned n = 0; n < left->closures.Size(); n++)
          {
            Closure *j = left->closures[n];
            // closures is not its own neighbour
            if(j != this)
            {
              unsigned pos_l = j->lines.Find(left);
              unsigned next = j->lines.CircularNext(pos_l);
              unsigned prev = j->lines.CircularPrev(pos_l);
              // if right line of T-jct is the next line of closure j
              // we have a T-jct from outside, and j genuinely occludes this.
              if(right == j->lines[next])
              {
                neighbors_above[j] += t->sig;
                j->neighbors_below[this] += t->sig;
              }
              // if right line of T-jct is the previous line of closure j
              // we have a T-jct from inside and this lies "inside" j.
              else if(right == j->lines[prev])
              {
                neighbors_outside[j] += t->sig;
                j->neighbors_inside[this] += t->sig;
              }
            }
          }
        }
      }
    }
}

/**
 * @brief Checks whether point p is inside closure. \n
 * Uses the Jordan curve theorem. Note that points on the boundary are undefined.
 * Code thanks to
 *  W Randolph Franklin (WRF)
 *  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 *  int pnpoly(int npol, float *xp, float *yp, float x, float y)
 *  {
 *    int i, j, c = 0;
 *    for (i = 0, j = npol-1; i < npol; j = i++) {
 *      if ((((yp[i]<=y) && (y<yp[j])) ||
 *           ((yp[j]<=y) && (y<yp[i]))) &&
 *          (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
 *
 *        c = !c;
 *    }
 *    return c;
 *  }
 * @param p Point to be checkt.
 */
bool Closure::Inside(Vector2 p)
{
  int i, j, npol = lines.Size();
  bool c = false;
  Vector2 pi, pj;
  for(i = 0, j = npol-1; i < npol; j = i++)
  {
    pi = GetVertex(i);
    pj = GetVertex(j);
    if((((pi.y <= p.y) && (p.y < pj.y)) ||
        ((pj.y <= p.y) && (p.y < pi.y))) &&
       (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x))

      c = !c;
  }
  return c;
}

/**
 * @brief Returns the area of the closure.
 * @return Returns the area of the closure.
 */
double Closure::Area()
{
  double area = 0.;
  Vector2 p1, p2;
  unsigned i, j;
  for(i = 0; i < jcts.Size(); i++)
  {
    j = jcts.CircularNext(i);
    p1 = jcts[i] != 0 ? jcts[i]->isct : colls[i]->vertex;
    p2 = jcts[j] != 0 ? jcts[j]->isct : colls[j]->vertex;
    area += p1.x*p2.y - p1.y*p2.x;
  }
  return area/2.;
}


}

