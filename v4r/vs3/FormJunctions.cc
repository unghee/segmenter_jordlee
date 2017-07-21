/**
 * @file FormJunctions.cc
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Class file of Gestalt principle FormJunctions.
 * 
 * TODO: remember and ignore search lines which reached the image border => do it not
 * TODO: avoid collinearity C(ac)    ----- ------ ------
 *                                     a      b      c
 *      note: this requires (possibly expensive) path search
 * TODO: stop if (at some very late time) no single search line can be
 *       extended any further.
 * TODO: check whether junction between lines i and j at respective ends as
 *       early as possible, right after detection of search line intersection
 */

#include "FormJunctions.hh"

namespace Z
{


// Minimal line length. A line of 2 pixels is per definition always a line and
// therefore totally insignificant. Any meaningful line must be at least 3
// pixels long.
static const double MIN_LINE_LENGTH = 3.;										///< Minimum line length

enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};		///< Grow method for search lines in the vote image.
static GrowMethod grow_method = GROW_SMART;									///< Default grow method


/**
 * @brief Compare function for all different types of junctions.
 */
static int CmpLJunctions(const void *a, const void *b)
{
  if( (*(LJunction**)a)->acc < (*(LJunction**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpCollinearities(const void *a, const void *b)
{
  if( (*(Collinearity**)a)->acc < (*(Collinearity**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpTJunctions(const void *a, const void *b)
{
  if( (*(TJunction**)a)->acc < (*(TJunction**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpLines(const void *a, const void *b)
{
  if( (*(Line**)a)->sig > (*(Line**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static bool InsideLine(Line *l, const Vector2 &p)
{
  double d = Dot(l->dir, p - l->point[START]);
  return d >= 0. && d <= l->Length();
}

/**
 * @brief Check whether there is an L-jct between lines i and j.
 * @param l_i 
 * @param end_i 
 * @param side_i 
 * @param l_j 
 * @return LJunction
 */
static LJunction* LBetween(Line *l_i, int end_i, int side_i, Line *l_j)
{
  for(unsigned k = 0; k < l_i->l_jct[end_i][side_i].Size(); k++)
    if(l_i->l_jct[end_i][side_i][k]->line[OtherSide(side_i)] == l_j)
      return l_i->l_jct[end_i][side_i][k];
  return 0;
}

/**
 * @brief Check whether there is no L-jct between lines i and j.
 */
static bool NoLJunctionYet(Line *l_i, int end_i, Line *l_j)
{
  // TODO: it would be nice to know which side to look
  for(int side = LEFT; side <= RIGHT; side++)
    for(unsigned k = 0; k < l_i->l_jct[end_i][side].Size(); k++)
      if(l_i->l_jct[end_i][side][k]->line[OtherSide(side)] == l_j)
        return false;
  return true;
}

/**
 * @brief NoTJunctionYet
 * @param l_i 
 * @param end_i 
 * @return 
 */
static bool NoTJunctionYet(Line *l_i, int end_i)
{
  return l_i->t_jct[end_i] == 0;
}

/**
 * @brief Check whether there is no collinearity yet between lines i and j. 
 */
static bool NoCollinearityYet(Line *l_i, int end_i, Line *l_j)
{
  for(unsigned k = 0; k < l_i->coll[end_i].Size(); k++)
    if(l_i->coll[end_i][k]->OtherLine(l_i) == l_j)
      return false;
  return true;
}


// ----------------------------------------------------------------------------//
// ------------------------------ FormJunctions -------------------------------//
// ----------------------------------------------------------------------------//
/**
 * @brief Constructor of FormJunctions
 * @param vc Vision core
 */
FormJunctions::FormJunctions(VisionCore *vc) : GestaltPrinciple(vc)
{}

/**
 * @brief Destructor of FormJunctions
 */
FormJunctions::~FormJunctions()
{}

/**
 * @brief Rank junctions.
 */
void FormJunctions::Rank()
{
  RankGestalts(Gestalt::L_JUNCTION, CmpLJunctions);
  RankGestalts(Gestalt::COLLINEARITY, CmpCollinearities);
  RankGestalts(Gestalt::T_JUNCTION, CmpTJunctions);
  RankGestalts(Gestalt::LINE, CmpLines);
}


/**
 * @brief Create new line junctions from delivered intersections.
 * @param sline Search line id
 * sline is the search line which triggered the new intersections, it is
 * therefore a tangent or normal vote line.
 * @param iscts Intersection element
 */
void FormJunctions::CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts)
{
// printf("FormJunctions::CreateJunctions: Try to create junctions from intersections\n");
	
	unsigned baseIndex = core->VI()->GetBaseIndex();
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned i = sline/baseIndex;
		unsigned vtype_i = sline%baseIndex;
    unsigned j = iscts[k].id/baseIndex;
		unsigned vtype_j = iscts[k].id%baseIndex;
    // check if the intersection between vote line tyes is admissible
    // TODO: this admissibility check could go into VoteImage
    if(core->VI()->IsctTypeAdmissible(vtype_i, vtype_j) == 1)
    {
// printf("FormJunctions::CreateJunctions 1: line-sline: %u-%u - vtypes: %u\n", i, sline, vtype_i);
// printf("FormJunctions::CreateJunctions 2: line-sline: %u-%u - vtypes: %u\n", j, iscts[k].id, vtype_j);
      Line *line_i = Lines(core, i);
      Line *line_j = Lines(core, j);
// printf("FormJunctions::CreateJunctions: lines: %u-%u - votes: %u-%u\n", i, j, vtype_i, vtype_j);
      // TODO: After a T-jct there can be no further T-jcts or type1
      // collinearities. -> really?
      if(vtype_j == VOTE_E)
        CreateT(line_i, vtype_i, line_j, vtype_j);
      else
      {
        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(line_i->phi, line_j->phi) < M_PI/6)
          CreateC(line_i, vtype_i, line_j, vtype_j);
        else
          CreateL(line_i, vtype_i, line_j, vtype_j);
      }
// printf("FormJunctions::CreateJunctions: end\n");
      /*else if(VOTE_IS_TANGENT(vtype_i) && VOTE_IS_TANGENT(vtype_j))
        CreateL(i, vtype_i, j, vtype_j);
      else if((VOTE_IS_NORMAL(vtype_i) && VOTE_IS_TANGENT(vtype_j)) ||
              (VOTE_IS_TANGENT(vtype_i) && VOTE_IS_NORMAL(vtype_j)))
        CreateC(i, vtype_i, j, vtype_j);*/
    }
  }
}

/**
 * @brief Creates a T-junction between lines i and j.
 * The extension of line i at given end meets line j. I.e. line i is the pole of
 * the T, Line j is the bar.
 * Line j is split into left and right bar and L-junctions with the pole are
 * created accordingly.
 *              b
 *             --+---- j
 *                g
 *               |
 *               | i
 *
 * TODO: a line which is already pole of a T-jct can not become arm of a T-jct
 * @param line_i First line i
 * @param vtype_i Type of search line (vote line type)
 * @param line_j Second line j
 * @param vtype_j Type of search line (vote line type)
 */
void FormJunctions::CreateT(Line *line_i, int vtype_i, Line *line_j, int vtype_j)
{
  try
  {
    int end_i = VOTE_END(vtype_i);
    // if a line is part of a split and we have junction at end, defer
    // creation of junctions to last line in that split
    // TODO: use convenience function for this
    if(end_i == END)
      while(line_i->next != 0)
        line_i = line_i->next;
    // if line j is part of a split find that part where the intersection
    // point actually lies
    if(line_j->IsSplit())
      while(line_j != 0 && !InsideLine(line_j, line_i->point[end_i]))
        line_j = line_j->next;
    if(line_j != 0 && InsideLine(line_j, line_i->point[end_i]))
    {
      double g;  // gap between end of line i and line j
      double b;  // smaller bar length
      Vector2 inter = LineIntersection(
          line_i->point[end_i], line_i->tang[end_i],
          line_j->point[START], line_j->dir, &g, &b);
      // if smaller bar length is long enough for a line
      if(min(b, line_j->Length() - b) >= MIN_LINE_LENGTH)
      {
        // note: for now we allow only one T-jct per end
        // TODO: maybe relax this at some later stage
        if(NoTJunctionYet(line_i, end_i))
        {
          Line *j_left = 0, *j_right = 0;
          int end_left, end_right;
          Collinearity *new_c = 0;
          SplitLine(line_j, inter, line_i->tang[end_i], &j_left, &j_right,
              &end_left, &end_right, &new_c);
          LJunction *new_ljct_left = NewL(line_i, j_left, end_i, end_left);
          LJunction *new_ljct_right = NewL(j_right, line_i, end_right, end_i);
          NewT(line_i, j_left, j_right, end_i, end_left, end_right,
              new_c, new_ljct_left, new_ljct_right);
        }
      }
      else // create just an L-junction
      {
        // find out what end of j the L-junction should be
        int end_j = (b < line_j->Length()/2. ? START : END);
        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(line_i->phi, line_j->phi) < M_PI/6)
          NewCollinearity(line_i, line_j, end_i, end_j, true);
        else
          NewL(line_i, line_j, end_i, end_j);
      }
    }
  }
	catch (exception &e)
	{
    // if tangents are parallel -> no intersections
	}
}

/**
 * @brief Create a new L-junction
 * @param line_i First line i
 * @param vtype_i Type of search line (vote line type)
 * @param line_j Second line j
 * @param vtype_j Type of search line (vote line type)
 */
void FormJunctions::CreateL(Line *line_i, int vtype_i, Line *line_j, int vtype_j)
{
  int end_i = VOTE_END(vtype_i);
  int end_j = VOTE_END(vtype_j);

  // if a line is part of a split and we have junction at end, defer
  // creation of junctions to last line in that split
  if(end_i == END)
    while(line_i->next != 0)
      line_i = line_i->next;
  if(end_j == END)
    while(line_j->next != 0)
      line_j = line_j->next;

// printf("FormJunctions::CreateL: lines: %u-%u - votes: %u-%u\n", line_i->ID(), line_j->ID(), vtype_i, vtype_j);

  LJunction *new_l = NewL(line_i, line_j, end_i, end_j);

  // create T-jcts from new L-jct and old collinearities
  // TODO: tidy this up
  if(new_l != 0)
  {
    Line *left = new_l->line[LEFT];
    Line *right = new_l->line[RIGHT];
    int end_l = new_l->near_point[LEFT];
    int end_r = new_l->near_point[RIGHT];
    for(unsigned c = 0; c < right->coll[end_r].Size(); c++)
    {
      // third would be the right arm of a new T-jct
      Line *third = right->coll[end_r][c]->OtherLine(right);
      int end_3 = right->coll[end_r][c]->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != left) // TODO: can be removed
      {
        LJunction *right_l = LBetween(third, end_3, LEFT, left);
        // only if there is also an L-jct between the other two lines
        if(right_l != 0)
          if(NoTJunctionYet(left, end_l))
          {
            NewT(left, right, third, end_l, end_r, end_3,
              right->coll[end_r][c], new_l, right_l);
          }
      }
    }
    for(unsigned c = 0; c < left->coll[end_l].Size(); c++)
    {
      // third would be the left arm of a new T-jct
      Line *third = left->coll[end_l][c]->OtherLine(left);
      int end_3 = left->coll[end_l][c]->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != right) // TODO: can be removed
      {
        LJunction *left_l = LBetween(third, end_3, RIGHT, right);
        // only if there is also an L-jct between the other two lines
        if(left_l != 0)
          if(NoTJunctionYet(right, end_r))
          {
            NewT(right, third, left, end_r, end_3, end_l,
              left->coll[end_l][c], left_l, new_l);
          }
      }
    }
  }
}

/**
 * @brief Create a new collinearity junction.
 * @param line_i First line i
 * @param vtype_i Type of search line (vote line type)
 * @param line_j Second line j
 * @param vtype_j Type of search line (vote line type)
 */
void FormJunctions::CreateC(Line *line_i, int vtype_i, Line *line_j, int vtype_j)
{
  int end_i = VOTE_END(vtype_i);
  int end_j = VOTE_END(vtype_j);

  // if a line is part of a split and we have junction at end, defer
  // creation of junctions to last line in that split
  if(end_i == END)
    while(line_i->next != 0)
      line_i = line_i->next;
  if(end_j == END)
    while(line_j->next != 0)
      line_j = line_j->next;

  Collinearity *new_c = NewCollinearity(line_i, line_j, end_i, end_j, true);

  // create T-jcts from new collinearity and old L-jcts
  if(new_c != 0)
  {
    for(int side = LEFT; side <= RIGHT; side++)
      for(unsigned l = 0; l < line_i->l_jct[end_i][side].Size(); l++)
      {
        LJunction *ljct = line_i->l_jct[end_i][side][l];
        Line *third = ljct->line[Other(side)];
        int end_3 = ljct->near_point[Other(side)];
        // Note that sometimes there is a collinearity AND L-jct between two
        // lines.  Do not create a T-jct for those cases.
        if(third != line_j) // TODO: can be rmoved
        {
          if(NoTJunctionYet(third, end_3))
          {
            TJunction *new_t = 0;
            if(side == RIGHT)  // i is RIGHT side of L-jct
            {
              // then i is left arm of T-jct, ljct is the left L-jct
              LJunction *ljct_right = LBetween(third, end_3, RIGHT, line_j);
              // only if there is also an L-jct between the other two lines
              if(ljct_right != 0)
                new_t = NewT(third, line_i, line_j, end_3, end_i, end_j,
                    new_c, ljct, ljct_right);
            }
            else  // i is LEFT side of L-jct
            {
              // then i is the right arm of T-jct, ljct is the right L-jct
              LJunction *ljct_left = LBetween(third, end_3, LEFT, line_j);
              if(ljct_left != 0)
                new_t = NewT(third, line_j, line_i, end_3, end_j, end_i,
                    new_c, ljct_left, ljct);
            }
          }
        }
      }
  }
}

/**
 * @brief Do some checks and if ok create collinearity.
 * @param line_i First line i
 * @param vtype_i Type of search line (vote line type)
 * @param line_j Second line j
 * @param vtype_j Type of search line (vote line type) 
 * @param inform Inform the system of new L-jct. Normally this will be true.\n
 * But not if coll. is created tue to a line split.
 * @return Collinearity
 */
Collinearity* FormJunctions::NewCollinearity(Line *line_i, Line *line_j, int end_i, int end_j, bool inform)
{
  Line *line[2];
  int near_point[2];
  // line 0 is shorter
  if(line_i->Length() <= line_j->Length())
  {
    line[0] = line_i;
    line[1] = line_j;
    near_point[0] = end_i;
    near_point[1] = end_j;
  }
  else
  {
    line[0] = line_j;
    line[1] = line_i;
    near_point[0] = end_j;
    near_point[1] = end_i;
  }
  // tangents must point at each other
  if(Dot(line_i->tang[end_i], line_j->tang[end_j]) < 0.)
    if(NoCollinearityYet(line_i, end_i, line_j) &&
       NoLJunctionYet(line_i, end_i, line_j))
    {
//       Vector2 v = line[0]->point[near_point[0]] - line[1]->point[near_point[1]];
      Collinearity *new_c = new Collinearity(core, line_i, line_j, end_i, end_j);
      core->NewGestalt(GestaltPrinciple::FORM_JUNCTIONS, new_c, inform);
      return new_c;
    }
  return 0;
}

/**
 * @brief Create new Gestalt L-junction.
 * @param line_i First line i
 * @param vtype_i Type of search line (vote line type)
 * @param line_j Second line j
 * @param vtype_j Type of search line (vote line type)
 * @return L-junction
 */
LJunction* FormJunctions::NewL(Line *line_i, Line *line_j, int end_i, int end_j)
{
  // if lines are parallel, return
  double d = Cross(line_i->tang[end_i], line_j->tang[end_j]);
  if (d == 0.0)
    return 0;
    
  // calculate intersection
  Vector2 p12 = line_j->point[end_j] - line_i->point[end_i];
  double l1 = Cross(line_j->tang[end_j], p12)/d;
  double l2 = Cross(line_i->tang[end_i], p12)/d;
  Vector2 inter = Vector2(line_i->point[end_i].x + l1*line_i->tang[end_i].x, line_i->point[end_i].y + l1*line_i->tang[end_i].y);

  if(NoLJunctionYet(line_i, end_i, line_j) && NoCollinearityYet(line_i, end_i, line_j))
  {
    LJunction *new_l = new LJunction(core, line_i, line_j, end_i, end_j, inter);
    core->NewGestalt(GestaltPrinciple::FORM_JUNCTIONS, new_l);
    return new_l;
  }
  return 0;
}

/**
 * @brief Create new T-junction
 * @param pole 
 * @param left 
 * @param right 
 * @param end_p 
 * @param end_l 
 * @param end_r 
 * @param coll 
 * @param ljct_l 
 * @param ljct_r 
 * @return 
 */
TJunction* FormJunctions::NewT(Line *pole, Line *left, Line *right, int end_p, int end_l, int end_r, Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r)
{
  try
  {
    if(coll != 0 && ljct_l != 0 && ljct_r != 0)
      if(NoTJunctionYet(pole, end_p) && NoTJunctionYet(left, end_l) &&
         NoTJunctionYet(right, end_r))
      {
        double b, g;
        Vector2 inter = LineIntersection(
                pole->point[end_p], pole->tang[end_p],
                left->point[end_l], left->tang[end_l], &g, &b);
        TJunction *new_t = new TJunction(core, pole, left, right,
              end_p, end_l, end_r, coll, ljct_l, ljct_r, g, inter);
        core->NewGestalt(GestaltPrinciple::FORM_JUNCTIONS, new_t);
        return new_t;
      }
  }
	catch (exception &e)
	{
    // if tangents are parallel -> no L-jct
	}
  return 0;
}

/**									/// TODO Description
 * @brief Split a line, when T-junction appears: Create 2 L-junctions and a collinearity from T.
 * @param l 
 * @param inter 
 * @param dir 
 * @param l_left 
 * @param l_right 
 * @param end_left 
 * @param end_right 
 * @param c_new 
 */
void FormJunctions::SplitLine(Line *l, const Vector2 &inter, const Vector2 &dir, Line **l_left, Line **l_right, int *end_left, int *end_right, Collinearity **c_new)
{
  Line *l_new = SplitLine(l, inter, c_new);
  if(LeftOf(dir, l->point[START] - inter))
  {
    *l_left = l;
    *l_right = l_new;
    *end_left = END;
    *end_right = START;
  }
  else
  {
    *l_left = l_new;
    *l_right = l;
    *end_left = START;
    *end_right = END;
  }
}

/**																					/// TODO Description
 * @brief Split 
 * @param l 
 * @param p 
 * @param c_new 
 * @return 
 */
Line* FormJunctions::SplitLine(Line *l, const Vector2 &p, Collinearity **c_new)
{
  // create new line
  Line *l_new = l->Split(p);
  // create new collinearity.
  // Note that if one of the split lines is very short and was near a
  // high-curvature part of the segment (i.e. near an L-jct), this line might
  // change orientation so much that a collinearity is rejected. In that case
  // form an L-jct.
  *c_new = NewCollinearity(l, l_new, END, START, false);
  // TODO: c_new actually can not be UNDEF
  if(*c_new == 0)
    NewL(l, l_new, END, START);
  UpdateClosures(l, l_new, *c_new);
  return l_new;
}

/**
 * @brief Line l1 has been split in to l1 and l2 with a colliinearity between them.
 * Update all closures which l1 is part of accordingly.
 * @param l1 Line, which is now splitted
 * @param l2 New line
 * @param coll New collinearty between line.
 */
void FormJunctions::UpdateClosures(Line *l1, Line *l2, Collinearity *coll)
{
  for(unsigned i = 0; i < l1->closures.Size(); i++)
  {
    Closure *cl = l1->closures[i];
    unsigned j = cl->lines.Find(l1);
    // note: depending on sense of l1, l2 is inserted before or after l2, but
    // the new collinearity is always inserted after the starting junction of l1
    if(cl->senses[j] == SAME)
    {
      cl->lines.InsertAfter(j, l2);
      cl->senses.InsertAfter(j, cl->senses[j]);
      cl->jcts.InsertAfter(j, 0);
      cl->colls.InsertAfter(j, coll);
    }
    else
    {
      cl->lines.InsertBefore(j, l2);
      cl->senses.InsertBefore(j, cl->senses[j]);
      cl->jcts.InsertAfter(j, 0);
      cl->colls.InsertAfter(j, coll);
    }
    l2->closures.PushBack(cl);
  }
}

}

