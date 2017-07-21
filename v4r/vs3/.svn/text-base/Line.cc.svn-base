/**
 * @file Line.cc
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2007, 2010
 * @version 0.1
 * @brief Class file of Gestalt Line (VisibleLine).
 **/

#include "Segment.hh"
#include "Line.hh"
#include "TJunction.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "FormSegments.hh"
#include "FormLines.hh"
#include "FormJunctions.hh"
#include <cstdio>

namespace Z
{

/**
 * @brief Constructor of class Line
 * @param vc Vision core
 */
Line::Line(VisionCore *vc) : Gestalt(vc, LINE)
{
  for(int i = START; i <= END; i++)
    t_jct[i] = 0;
}


/**
 * @brief Calculate parameters
 * TODO TODO Documentation
 */
void Line::CalculateParameters()
{
  Vector2 v = point[END] - point[START];
  dir = Normalise(v);
  tang[START] = -dir;
  tang[END] = dir;
  phi = ScaleAngle_0_2pi(PolarAngle(dir));
  len = v.Length();
	
  // Hessian parameters
  /*if(IsZero(dir.x))
  {
    if(start_point.x >= 0.)
    {
      s = start_point.x;
      theta = 3*M_PI_2;
    }
    else
    {
      s = -start_point.x;
      theta = M_PI_2;
    }
  }
  else
  {
    double y = start_point.y - dir.y*start_point.x/dir.x;  // point on y axis
    theta = ScaleAngle_0_2pi(atan(dir.y/dir.x));
    s = y*cos(theta);
    if(s < 0.)
    {
      s = -s;
      theta = ScaleAngle_0_2pi(theta + M_PI);
    }
  }*/
}


/**
 * @brief Draw line. 
 * @param detail Degree of detail for drawing.
 */
void Line::Draw(int detail)
{
  DrawLine2D(point[START].x, point[START].y, point[END].x, point[END].y);
  if(detail >= 1)
  {
    char id_str[20];
    Vector2 mid_point = (point[START] + point[END])/2. + Vector2(4., -4.);
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, mid_point.x, mid_point.y, RGBColor::red);
    DrawArrow(point[START], point[END], RGBColor::red);
  }
  /*if(detail >= 2)
  {
    // draw T-junctions
    if(t_jct[START] != UNDEF_ID)
      TJunctions(t_jct[START])->Draw(0);
    if(t_jct[END] != UNDEF_ID)
      TJunctions(t_jct[END])->Draw(0);

    // draw L-junctions
    for(int end = START; end <= END; end++)
      for(int side = LEFT; side <= RIGHT; side++)
        for(unsigned i = 0; i < l_jct[end][side].Size(); i++)
          LJunctions(l_jct[end][side][i])->Draw(0);
  }*/
}


/**
 * @brief Draw vote lines of a visible line.
 */
void Line::DrawVotes()
{
  VoteImage *vi = core->VI();
  if(vi == 0)
    return;
  for(int x = 0; x < vi->width; x++)
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        if(el->id/vi->GetBaseIndex() == id)
        {
          unsigned vtype = el->id%vi->GetBaseIndex();
          switch(vtype)
          {
            case VOTE_TS:
            case VOTE_NLS:
            case VOTE_NRS:
              DrawPoint2D(x, y, RGBColor::magenta);
              break;
            case VOTE_TE:
            case VOTE_NLE:
            case VOTE_NRE:
              DrawPoint2D(x, y, RGBColor::cyan);
              break;
            default:
              DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
}

/**
 * @brief Draw info.
 */
void Line::DrawInfo()
{
  char str[100];
  FillRect2D(0., 0., 0.5, 1., mean_col[LEFT]);
  DrawText2D("left", 0.1, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", mean_col[LEFT].r, mean_col[LEFT].g, mean_col[LEFT].b);
  DrawText2D(str, 0.1, 0.6, RGBColor::green);
  FillRect2D(0.5, 0., 1., 1., mean_col[RIGHT]);
  DrawText2D("right", 0.6, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", mean_col[RIGHT].r, mean_col[RIGHT].g, mean_col[RIGHT].b);
  DrawText2D(str, 0.6, 0.6, RGBColor::green);
}

/**
 * @brief Get information about the Gestalt as string.
 * @return Returns information about the Gestalt as string.
 */
const char* Line::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%s  (%.0f %.0f)-(%.0f %.0f)\n"
      "  length: %4.2f\n"
      "  phi: %4.3f\n"
      "  neighbours: %d\n"
      "  --------------------\n",
      Gestalt::GetInfo(),
     point[START].x, point[START].y, point[END].x, point[END].y, Length(), phi, (int)neighbors.size());
  return info_text;
}

/**
 * @brief Minimum endpoint distance:
 * @param l 
 * @return Returns the 
 */
double Line::MinEndpointDistance(const Line* l)
{
  double dist = HUGE;
  for(int i = START; i <= END; i++)
    for(int j = START; j <= END; j++)
      dist = fmin(dist, Distance(point[i], l->point[j]));
  return dist;
}

/**
 * @brief Calculate the distance to a point
 * @param q Vector to the point
 * @return Returns the distance from the point to the line.
 */
double Line::DistanceToPoint(const Vector2 &q)
{
  Vector2 start_to_q = q - point[START];
  double s = Dot(start_to_q, dir);
  if(s >= 0. && s <= Length())
    return fabs(Cross(start_to_q, dir));
  else
    return Distance(q, point[(s<0. ? START : END)]);
}


/**
 * @brief Add a L-junction to the line.
 * @param end Line end (start/end)
 * @param side Line side (left/right)
 * @param jct L-junction
 */
void Line::AddLJunction(int end, int side, LJunction* jct)
{
  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < l_jct[end][side].Size())
  {
    // rank according to accidentalness
    if(jct->acc < l_jct[end][side][i]->acc)
    {
      l_jct[end][side].InsertBefore(i, jct);
      return;
    }
    i++;
  }
  l_jct[end][side].PushBack(jct);
  neighbors.insert(jct->line[Other(side)]);
}

/**
 * @brief Add a Collinearity to the line.
 * @param end Line end (start/end)
 * @param co Collinearity
 */
void Line::AddCollinearity(int end, Collinearity* co)
{
  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < coll[end].Size())
  {
    // rank according to significance
    if(co->sig > coll[end][i]->sig)
    {
      coll[end].InsertBefore(i, co);
      return;
    }
    i++;
  }
  coll[end].PushBack(co);
  neighbors.insert(co->OtherLine(this));
}

/**
 * @brief Add a passive T-junction.
 * @param end Line end (start/end)
 * @param side Line side (left/right)
 * @param jct T-junction
 */
void Line::AddPassiveTJunction(int end, int side, TJunction* jct)
{
  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < pt_jct[end][side].Size())
  {
    // rank according to accidentalness
    if(jct->acc < pt_jct[end][side][i]->acc)
    {
      pt_jct[end][side].InsertBefore(i, jct);
      return;
    }
    i++;
  }
  pt_jct[end][side].PushBack(jct);
  // note that any T-jct Tabc implies L-jcts and collinearities Lab, Lca and
  // Cbc, so we don't need to update neighborhood here, as it is handled by L
  // and C.
}

/**
 * @brief Move junction after splitting a line. We have to move the junctions 
 * of the line end to the end of the new split line.
 * @param l2 Line
 * @param end Line end (start/end)
 */
void Line::MoveJunctions(Line *l2, int end)
{
  // T-junctions
  if(t_jct[end] != 0)
  {
    TJunction *t = t_jct[end];
    t->line[POLE] = l2;
    t->Recalc();
    l2->t_jct[end] = t;
    t_jct[end] = 0;
  }

  // passive T-junctions
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < pt_jct[end][side].Size(); i++)
    {
      TJunction *t = pt_jct[end][side][i];
      t->line[side] = l2;
      t->Recalc();
      l2->AddPassiveTJunction(end, side, t);
    }
    pt_jct[end][side].Clear();
  }

  // collinearities
  for(unsigned i = 0; i < coll[end].Size(); i++)
  {
    Collinearity *c = coll[end][i];
    int w = c->WhichLineIs(this);
    c->line[w] = l2;
    // near_point stays the same
    c->Recalc();
    l2->AddCollinearity(end, c);
  }
  coll[end].Clear();

  // L-junctions
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < l_jct[end][side].Size(); i++)
    {
      LJunction *j = l_jct[end][side][i];
      j->line[side] = l2;
      j->Recalc();
      l2->AddLJunction(end, side, j);
    }
    l_jct[end][side].Clear();
  }
}


/**
 * @brief Constructor of class VisibleLine.
 * @param c Vision core
 * @param s Segment
 * @param i Start index
 * @param j End index
 */
VisibleLine::VisibleLine(VisionCore *c, Segment* s, unsigned i, unsigned j) : Line(c)
{
  defer_vote = this;
  next = 0;
  seg = s;
  idx[START] = i;
  idx[END] = j;
  label = 0;
  Recalc();
}

/**
 * @brief Recalculate visible line. \n
 * Calculate parameters, colors, significance
 */
void VisibleLine::Recalc()
{
  point[START] = seg->edgels[idx[START]].p;
  point[END] = seg->edgels[idx[END]].p;
  CalculateParameters();
  CalculateColors();
  CalculateSignificance();
}

/**
 * @brief Get info about the visible line.
 * @return Returns information about the visible line as string.
 */
const char* VisibleLine::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
  n += snprintf(info_text + n, info_size, 
      "%s"
      "  Visible line\n"
      "  seg: %u\n"
      "  label: %d\n"
      "  energy: %4.2f \n"
      "  stability: %f\n"
      "  Ts %d %d\n",

      Line::GetInfo(), seg->ID(), label, energy, stability,
      (t_jct[START] ? t_jct[START]->ID() : UNDEF_ID),
      (t_jct[END] ? t_jct[END]->ID() : UNDEF_ID));
  n += snprintf(info_text + n, info_size - n, "  %u closures:", closures.Size());
  for(unsigned i = 0; i < closures.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", closures[i]->ID());
  n += snprintf(info_text + n, info_size - n, "\n");
//   n += snprintf(info_text + n, info_size - n, "  %u rectangles:", rectangles.Size());
//   for(unsigned i = 0; i < rectangles.Size(); i++)
//     n += snprintf(info_text + n, info_size - n, " %u", rectangles[i]->ID());  return info_text;
  return info_text;
}

/**
 * @brief Draw Gestalt visible line.
 * @param detail Degree of detail
 */
void VisibleLine::Draw(int detail)
{
  if(detail <= 1)
    Line::Draw(detail);
  if(detail == 2)
    for(unsigned i = idx[START]; i <= idx[END]; i++)
      DrawPoint2D(seg->edgels[i].p.x, seg->edgels[i].p.y, RGBColor::green);
  if(detail == 3)
  {
    // TODO: avoid double code , use same as CalculateColors
    const unsigned NUM_SAMPLES = 16;
    unsigned i, j, s, e;
    int x1, y1, x2, y2;
    Vector2 n;
    RGBColor col;
    // sample at regular intervals
    for(j = 0; j < NUM_SAMPLES; j++)
    {
      s = idx[START] + (idx[END] - idx[START])/4;
      e = idx[START] + ((idx[END] - idx[START])*3)/4;
      i = s + ((e - s)*j)/(NUM_SAMPLES - 1);
      // find the points halfway between line and next edge
      // left
      n = dir.NormalAntiClockwise();
      x1 = (int)(seg->edgels[i].p.x + 2.*n.x);
      y1 = (int)(seg->edgels[i].p.y + 2.*n.y);
      if(FormSegments::edge_img->FindLineEnd(x1, y1,
          x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      {
        DrawPoint2D(x2, y2, RGBColor::blue);
        DrawPoint2D((x1+x2)/2, (y1+y2)/2, RGBColor::cyan);
      }
      // right
      n = -n;
      x1 = (int)(seg->edgels[i].p.x + 2.*n.x);
      y1 = (int)(seg->edgels[i].p.y + 2.*n.y);
      if(FormSegments::edge_img->FindLineEnd(x1, y1,
          x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      {
        DrawPoint2D(x2, y2, RGBColor::blue);
        DrawPoint2D((x1+x2)/2, (y1+y2)/2, RGBColor::cyan);
      }
    }
  }
  if(detail == 4)
    DrawVotes();
  if(detail == 10)
  {
    DrawPoint2D(point[START].x, point[START].y, RGBColor::red);
    DrawPoint2D(point[END].x, point[END].y, RGBColor::red);
  }
}

/**
 * @brief Checks if Gestalt is at this position.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if the Gestalt is at this position.
 */
bool VisibleLine::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = idx[START]; i <= idx[END]; i++)
    if(IsEqual(xd, seg->edgels[i].p.x) && IsEqual(yd, seg->edgels[i].p.y))
      return true;
  return false;
}

/**
 * @brief Calculate the significance for the visible line.
 */
void VisibleLine::CalculateSignificance()
{
  sig = -log(pow(core->p_ee, (double)NumEdgels()));
  
  if(core->roi_sigma > 0)  // if we have a ROI defined, weight significance
  {
    printf("VisibleLine::CalculateSignificance: Warning: ROI defined => weight significance!\n");
    Vector2 m = MidPoint(point[START], point[END]);
    double weight = BivariateGaussianPDF(
          core->roi_center.x, core->roi_center.y,
          core->roi_sigma, m.x, m.y);
    sig *= weight;
  }
}

/**
 * @brief Sample some points at either side and calculate mean colors.
 */
void VisibleLine::CalculateColors()
{
  const unsigned NUM_SAMPLES = 16;
  unsigned i, j, s, e;
  int x1, y1, x2, y2;
  int rl = 0, gl = 0, bl = 0;
  int rr = 0, gr = 0, br = 0;
  Vector2 n;
  RGBColor col;
  // sample at regular intervals
  for(j = 0; j < NUM_SAMPLES; j++)
  {
    s = idx[START] + (idx[END] - idx[START])/4;
    e = idx[START] + ((idx[END] - idx[START])*3)/4;
    i = s + ((e - s)*j)/(NUM_SAMPLES - 1);
    // Find the points halfway between line and next edge. Search next edge at
    // max l pixels away, where l is line length.
    // left
    n = dir.NormalAntiClockwise();
    x1 = (int)(seg->edgels[i].p.x + 2.*n.x);
    y1 = (int)(seg->edgels[i].p.y + 2.*n.y);
    if(FormSegments::edge_img->FindLineEnd(x1, y1,
        x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      col = core->Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    rl += (int)col.r;
    gl += (int)col.g;
    bl += (int)col.b;
    // right
    n = -n;
    x1 = (int)(seg->edgels[i].p.x + 2.*n.x);
    y1 = (int)(seg->edgels[i].p.y + 2.*n.y);
    if(FormSegments::edge_img->FindLineEnd(x1, y1,
        x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      col = core->Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    rr += (int)col.r;
    gr += (int)col.g;
    br += (int)col.b;
  }
  mean_col[LEFT].r = rl/NUM_SAMPLES; 
  mean_col[LEFT].g = gl/NUM_SAMPLES; 
  mean_col[LEFT].b = bl/NUM_SAMPLES; 
  mean_col[RIGHT].r = rr/NUM_SAMPLES; 
  mean_col[RIGHT].g = gr/NUM_SAMPLES; 
  mean_col[RIGHT].b = br/NUM_SAMPLES; 
}

/**
 * @brief Find index of edgel of line l which is closest to point p.
 * Is used to find split point for lines.
 * @param p Point p
 * @return Returns the minimum split index
 */
unsigned VisibleLine::FindSplitIdx(const Vector2 &p)
{
  unsigned i, imin = UNDEF_ID;
  double d, dmin = HUGE;
  for(i = idx[START]; i <= idx[END]; i++)
  {
    d = DistanceSquare(seg->edgels[i].p, p);
    if(d < dmin)
    {
      dmin = d;
      imin = i;
    }
  }
  return imin;
}

/**
 * @brief Split visible line and create new line.
 * @param p Point p
 * @return  Returns the rest of the line, as a new line.
 */
Line* VisibleLine::Split(const Vector2 &p)
{
  unsigned idx_split = FindSplitIdx(p);
  // create new line
  Line *l_new = new VisibleLine(core, seg, idx_split, idx[END]);
  core->NewGestalt(GestaltPrinciple::FORM_LINES, l_new);
  // this line has no vote lines associated, therefore does not vote itself but
  // defers vote to original line
  l_new->defer_vote = this;
  // insert properly in the split chain
  l_new->next = next;
  next = l_new;
  // shorten old line
  idx[END] = idx_split;
  Recalc();
  // copy END junctions of old line to END of new line
  MoveJunctions(l_new, END);
  return l_new;
}

}

