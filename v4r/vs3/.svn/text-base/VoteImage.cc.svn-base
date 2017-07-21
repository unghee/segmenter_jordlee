/**
 * @file VoteImage.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief The vote image controls the extension of search lines.
 *
 * TODO: Implement other grow methods: grow_equal, grow_weighted
 * TODO: Grow_smart for ellipses and ars is not "so" smart ;-)
 *
 * TODO: avoid duplicate code
 * TODO: deal with very flat intersections (see 28.11.2006)
 * TODO: avoid drawing the first pixel of EDGE, T, NL, NR four times.
 *       e.g. skip first pixel in InitLine
 */

#include <assert.h>
#include <cstdio>
#include <string.h>

#include "VoteImage.hh"
#include "VisionCore.hh"
#include "Arc.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "FormEJunctions.hh"

namespace Z
{

enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};
static GrowMethod grow_method = GROW_SMART;

/**
 * @brief Constructor of vote image
 * @param w Image width
 * @param h Image height
 */
VoteImage::VoteImage(VisionCore *vc, int w, int h)	/// TODO w,h kann später gelöscht werden => vc->IW(), vc->IH()
{
  core = vc;
  width = vc->IW();
  height = vc->IH();
  data = new Elem*[width*height];
  assert(data != 0);
  store_size = 2*width*height;          // TODO: this is an arbitrary choice
  store = new Elem[store_size];
  fill = 0;
  initialized = false;
}

/**
 * @brief Destructor of vote image.
 */
VoteImage::~VoteImage()
{
  delete[] data;
  delete[] store;
}

/**
 * @brief Initialize vote image for processing (extending search lines)
 */
void VoteImage::Initialize()
{
  Clear();                              // clear vote image and initialize 'data' structure
  baseIndex = 18;                       // number of search lines
  arcOffset = 0;
  ellOffset = 0;
  activeJcts = true;                    // TODO configure later via Initialize
  activeAJcts = true;                   // TODO configure later
  activeEJcts = false;                   // TODO configure later
  SetupAdmissibilityMatrix();
  InitSearchLines();
  initialized = true;
}

/**
 * @brief Setup the admissibility matrix for detection of junctions.
 */
void VoteImage::SetupAdmissibilityMatrix()
{
  unsigned i, j;
  for(i = 0; i < baseIndex; i++)
    for(j = 0; j < baseIndex; j++)
      isct_ok[i][j] = 0;

  // note: We fill out the upper right side of the matrix. It is important to
  // keep in mind the numbers corresponding to the symbolic names.
  // I.e. in isct_ok[i][j] i must be <= j.
  if (activeJcts)
  {
    // tangents/edge						=> T-Junctions
    isct_ok[VOTE_E][VOTE_TS] = 1;
    isct_ok[VOTE_E][VOTE_TE] = 1;
    // tangents/tangents					=> L-Junctions or Collinearities
    isct_ok[VOTE_TS][VOTE_TE] = 1;
    isct_ok[VOTE_TS][VOTE_TS] = 1;
    isct_ok[VOTE_TE][VOTE_TE] = 1;
    // tangents/normals					=> L-Junctions or Collinearities
    isct_ok[VOTE_TS][VOTE_NLS] = 1;
    isct_ok[VOTE_TS][VOTE_NLE] = 1;
    isct_ok[VOTE_TS][VOTE_NRS] = 1;
    isct_ok[VOTE_TS][VOTE_NRE] = 1;
    isct_ok[VOTE_TE][VOTE_NLS] = 1;
    isct_ok[VOTE_TE][VOTE_NLE] = 1;
    isct_ok[VOTE_TE][VOTE_NRS] = 1;
    isct_ok[VOTE_TE][VOTE_NRE] = 1;
  }
 
  // arcs => A-Junctions
  if (activeAJcts)
  {
    // basic "daisy chain" constraint: connect only START <-> END of two arcs
    // tangents/tangents
    isct_ok[VOTE_ATS][VOTE_ATE] = 2;
    // tangents/normals
    isct_ok[VOTE_ATS][VOTE_ANLE] = 2;
    isct_ok[VOTE_ATS][VOTE_ANRE] = 2;
    isct_ok[VOTE_ATE][VOTE_ANLS] = 2;
    isct_ok[VOTE_ATE][VOTE_ANRS] = 2;
  }
  
  if (activeEJcts) // ellipses => E-Junctions
  {
    isct_ok[VOTE_E][VOTE_EOTL] = 3; 	// edge itself & ellipse tangents
    isct_ok[VOTE_E][VOTE_EOTR] = 3;	
    isct_ok[VOTE_E][VOTE_EITL] = 3;
    isct_ok[VOTE_E][VOTE_EITR] = 3;

    isct_ok[VOTE_TS][VOTE_EOTL] = 3;	// tangents - outer ellipse tangents
    isct_ok[VOTE_TS][VOTE_EOTR] = 3;
    isct_ok[VOTE_TE][VOTE_EOTL] = 3;
    isct_ok[VOTE_TE][VOTE_EOTR] = 3;

    isct_ok[VOTE_TS][VOTE_EITL] = 3;	// Tangents - inner ellipse tangents
    isct_ok[VOTE_TS][VOTE_EITR] = 3;
    isct_ok[VOTE_TE][VOTE_EITL] = 3;
    isct_ok[VOTE_TE][VOTE_EITR] = 3;
  }
  // now mirror along diagonal to fill lower left side of matrix
  for(i = 0; i < baseIndex; i++)
    for(j = i + 1; j < baseIndex; j++)
      isct_ok[j][i] = isct_ok[i][j];
}

/**
 * @brief Init the search lines.
 * The line- and arc search lines will be initialized at once because their number is
 * static. The initialization of ellipse search lines happens during incremental processing.
 * We call the initialize method of FormEJunctions to initialize already created ellipses.
 */
void VoteImage::InitSearchLines()
{
  // set num lines
  unsigned numLines = 0;
  if (activeJcts) numLines += (core->RankedGestalts(Gestalt::LINE)).Size();
  if (activeAJcts) numLines += (core->RankedGestalts(Gestalt::ARC)).Size();
  SetNumLines(numLines*baseIndex);

  if(activeJcts)						// number of lines is static
  {
    Array<Gestalt*> lines = core->RankedGestalts(Gestalt::LINE);
    for(unsigned i = 0; i < lines.Size(); i++)
      InitLineSearchLines((Line*)lines[i]);	  
    arcOffset = lines.Size();
  }
  else arcOffset = 0;

  if(activeAJcts)						// number of arcs is static
  {
    Array<Gestalt*> arcs = core->RankedGestalts(Gestalt::ARC);
    for(unsigned i=0; i<arcs.Size(); i++)
	    InitArcSearchLines((Arc*)arcs[i]);
    ellOffset = arcOffset + arcs.Size();
  } 
  else ellOffset = arcOffset;

  if(activeEJcts)						// number of ellipses is dynamic
  {
    // Initialize FormEJunctions, if not already done
    ((FormEJunctions*)core->Principles(GestaltPrinciple::FORM_E_JUNCTIONS))->Initialize();
  }
}

/**
 * @brief Init the search lines for the lines
 * @param l Line for search line initialisation.
 */
void VoteImage::InitLineSearchLines(Line *l)
{
  double t_length = l->len*8;                                       // TODO Tangent vote line length
  double n_length = l->len;                                         // TODO Normal vote line length
  unsigned sline = l->ID()*baseIndex;

  // draw edge itself
  DrawLine(l->point[START], l->point[END], sline + VOTE_E);

  // initialise tangents for growing
  InitLine(l->point[START], l->point[START] - t_length*l->dir, sline + VOTE_TS);
  InitLine(l->point[END], l->point[END] + t_length*l->dir, sline + VOTE_TE);
  // initialise normals for growing
  InitLine(l->point[START], l->point[START] + n_length*l->dir.NormalAntiClockwise(), sline + VOTE_NLS);
  InitLine(l->point[END], l->point[END] + n_length*l->dir.NormalAntiClockwise(), sline + VOTE_NLE);
  InitLine(l->point[START], l->point[START] - n_length*l->dir.NormalAntiClockwise(), sline + VOTE_NRS);
  InitLine(l->point[END], l->point[END] - n_length*l->dir.NormalAntiClockwise(), sline + VOTE_NRE);
}

/**
 * @brief Init the search lines for the arcs.
 * @param arc Arc for search line initialisation.
 */
void VoteImage::InitArcSearchLines(Arc *arc)
{
  double length = arc->ArcLength();//1000; 														// TODO: maximum length is arc length
  unsigned sline = (arc->ID() + arcOffset)*baseIndex;

	// TODO we could also draw the arc itself and could use it as search line (see lines)
  // tangent
  InitLine(arc->point[START], arc->point[START] + length*arc->norm[START].NormalClockwise(), sline + VOTE_ATS);
  // radii
  InitLine(arc->point[START], arc->point[START] + length*arc->norm[START], sline + VOTE_ANLS);
  InitLine(arc->point[START], arc->point[START] - length*arc->norm[START], sline + VOTE_ANRS);
  // tangent
  InitLine(arc->point[END], arc->point[END] + length*arc->norm[END].NormalAntiClockwise(), sline + VOTE_ATE);
  // radii
  InitLine(arc->point[END], arc->point[END] + length*arc->norm[END], sline + VOTE_ANLE);
  InitLine(arc->point[END], arc->point[END] - length*arc->norm[END], sline + VOTE_ANRE);
}

/**
 * @brief Init the search lines for the ellipses.
 * The initialisation of ellipses happens during processing and is therefore
 * dynamic. 
 * @param ell Ellipse for search line initialization.
 * @param sl search line number
 * @param is Intersections
 * @return Returns the number of found intersections.
 */
unsigned VoteImage::InitEllipseSearchLines(Ellipse *ell, Array<unsigned> &sl, Array<Elem> &is)
{
// printf("VoteImage::InitEllipseSearchLines: %u\n", ell->ID());

  // extend the number of search lines
  ExtendNumLines((ellOffset+ell->ID()+1) * baseIndex);

  // limit the maximal length of the search lines by majorAxis/4.																								/// TODO Limit search line length
  double majorAxis = (ell->vertex[1] - ell->vertex[0]).Norm();
  unsigned length = majorAxis/4; 

  // search line index for this ellipse
  unsigned sline = (ellOffset + ell->ID())*baseIndex;

	// Check vertex points for intersections with the already drawn lines
	iscts.Clear();
	sl.Clear();
	
	if(ell->vertex[0].x < core->IW() && ell->vertex[0].y < core->IH())
		CheckPixel(ell->vertex[0].x, ell->vertex[0].y, sline, iscts);
	for(unsigned i=0; i<iscts.Size(); i++)
		sl.PushBack(sline + VOTE_EOTL);
	unsigned nrFound = iscts.Size();

	if(ell->vertex[1].x < core->IW() && ell->vertex[1].y < core->IH())
		CheckPixel(ell->vertex[1].x, ell->vertex[1].y, sline, iscts);
	for(unsigned i=nrFound; i<iscts.Size(); i++)
		sl.PushBack(sline + VOTE_EOTR);

  // initialise ellipse search lines
  InitLine(ell->vertex[LEFT], ell->vertex[LEFT] - length*ell->dir, sline + VOTE_EOTL);
  InitLine(ell->vertex[RIGHT], ell->vertex[RIGHT] + length*ell->dir, sline + VOTE_EOTR);
  InitLine(ell->vertex[LEFT], ell->vertex[LEFT] + length*ell->dir, sline + VOTE_EITL);
  InitLine(ell->vertex[RIGHT], ell->vertex[RIGHT] - length*ell->dir, sline + VOTE_EITR);

// printf("VoteImage::InitEllipseSearchLines: end\n");
	is = iscts;
	return is.Size();
}

/**																																										TODO TODO TODO Check addmissibility, before adding to iscts!!!!
 * @brief Process the vote image: extend a search line
 * @param iscts Intersections
 */
bool VoteImage::Extend(unsigned &sline, Array<VoteImage::Elem> &is, bool &_extendedLine)
{
  extendedLine = false;
// printf("VoteImage::Extend start!\n");
	iscts.Clear();
	
	// first decide which Gestalt: line, arc, ellipse TODO Calculate in every processing step??? slow?
	float gest = FRand();  	// random number between 0-1
	float linAm = (float)NumLines(core) / ((float)NumLines(core)+(float)NumEllipses(core)+(float)NumArcs(core));
	float arcAm = ((float)NumArcs(core) / ((float)NumLines(core)+(float)NumEllipses(core)+(float)NumArcs(core))) + linAm;
	
// printf("  VoteImage::Extend: allocation: gest: %4.2f - lin: %4.2f - arc: %4.2f\n", gest, linAm, arcAm);
	
	if (activeJcts && gest < linAm)
	{
// printf("Extend-Line\n");
		int r = ExpSelect(NumLines(core) - 1);
		if(grow_method == GROW_SMART)
			ExtendSmart(Gestalt::LINE, core->RankedGestalts(Gestalt::LINE, r)->ID());

// for(unsigned i=0; i<iscts.Size(); i++)
// printf("  VoteImage::Extend lin: sline: %u-%u\n", sline%baseIndex, iscts[i].id%baseIndex);
	}
	else if(activeAJcts && gest >linAm && gest < arcAm)
	{
		int r = ExpSelect(NumArcs(core) - 1);
		if(grow_method == GROW_SMART)																																		/// TODO TODO TODO Grow smart? Sind Arcs geranked?
			ExtendSmart(Gestalt::ARC, core->RankedGestalts(Gestalt::ARC, r)->ID());
// for(unsigned i=0; i<iscts.Size(); i++)
// printf("  VoteImage::Extend arc: sline: %u-%u\n", sline%baseIndex, iscts[i].id%baseIndex);
	}
	else if(activeEJcts)
	{
		int r = ExpSelect(NumEllipses(core) - 1);
		if(grow_method == GROW_SMART)
			ExtendSmart(Gestalt::ELLIPSE, core->RankedGestalts(Gestalt::ELLIPSE, r)->ID());
// printf("ellipe id: %u\n", core->RankedGestalts(Gestalt::ELLIPSE, r)->ID());
	}
	
	sline = this->sline;
 	is = iscts;

// for(unsigned i=0; i<is.Size(); i++)
// printf("  VoteImage::Extend ell: sline: %u-%u\n", sline%baseIndex, is[i].id%baseIndex);

  _extendedLine = extendedLine;
	if(iscts.Size() > 0) return true;
	else return false;
}

/**
 * @brief Be somewhat smart about extending search lines.
 * Prefer tangential search lines and open line ends, boost neighbouring lines.
 * @param type Type of Gestalt search line to extended
 * @param idx Index of Gestalt
 */
void VoteImage::ExtendSmart(Gestalt::Type type, unsigned idx)
{
// printf("VoteImage::ExtendSmart: start!\n");
  if (type == Gestalt::LINE)
  {
		// Selecting which end to extend is given by the number of junctions already
		// present at that end: p(START)/p(END) = n(END)/n(START).
		// Regarding T-junctions: Each T-jct at one end is accompanied by two L-jcts.
		// Therefore reduce the count of jcts at this side by two.
		// Furthermore add 1 to the jct count on the other end to further favour this
		// end.
		Line *l = Lines(core, idx);
		int n_t[2] = {(l->t_jct[START] != 0 ? 1 : 0), (l->t_jct[END] != 0 ? 1 : 0)};
		int n_s = 1 + l->coll[START].Size() + l->l_jct[START][LEFT].Size() +
				l->l_jct[START][RIGHT].Size() - 2*n_t[START] + n_t[END];
		int n_e = 1 + l->coll[END].Size() + l->l_jct[END][LEFT].Size() +
			l->l_jct[END][RIGHT].Size() - 2*n_t[END] + n_t[START];
		int r = RandInt()%(n_s + n_e);

		unsigned end;
		if(r < n_s) end = END;			// 0 .. n_s - 1   
		else end = START;						// n_s .. n_s + n_e - 1

		ExtendSmartLineEnd(idx, end);
  }

	if (type == Gestalt::ARC)
  {
		int n_l = 1 + Arcs(core, idx)->jct[LEFT].Size();			// AJcts LEFT
		int n_r = 1 + Arcs(core, idx)->jct[RIGHT].Size();			// AJcts RIGHT
		int r = RandInt()%(n_l + n_r);												// decide left or right end
		unsigned end;
		if(r < n_l) end = LEFT;																// 0 .. n_l - 1
		else end = RIGHT; 																		// n_l .. n_l + n_r - 1
		ExtendSmartArcEnd(idx, end);
  }	

  if (type == Gestalt::ELLIPSE)
  {
		int n_l = 1 + Ellipses(core, idx)->ejcts[LEFT].Size();	// E-Jcts LEFT
		int n_r = 1 + Ellipses(core, idx)->ejcts[RIGHT].Size();	// E-Jcts RIGHT

		int r = RandInt()%(n_l + n_r);
		unsigned end;
		if(r < n_l)  // 0 .. n_l - 1
			end = LEFT;
		else         // n_l .. n_l + n_r - 1
			end = RIGHT;
		
// printf("VoteImage::ExtendSmart: extend: idx: %u\n", idx);
		ExtendSmartEllipseEnd(idx, end);	  
  }	
// printf("VoteImage::ExtendSmart: end!\n");
}

/**
 * @brief Be somewhat smart about extending search lines.
 * Follow the line end (maybe split lines) to the search line, we want to extend.
 * Then find intersections.
 * @param idx Line index
 * @param end Line end
 */
void VoteImage::ExtendSmartLineEnd(unsigned idx, unsigned end)
{
// printf("ExtendSmartLineEnd start: idx: %u - end: %u\n", idx, end);
	Line *l = Lines(core, idx);
	FollowEnd(l, end, l);
// printf("ExtendSmartLineEnd end\n");
}

/**
 * @brief Extend an search line of an arc at one end.
 * TODO This function is not smart!
 */
void VoteImage::ExtendSmartArcEnd(unsigned idx, int end)
{
	// which search line: AT, ANL, ANR
	unsigned NorT = RandInt()%2;						// Normal or tangential direction (we prefer tangential search lines: 2:1)
	int side = RandInt()%2;									// left or right normal
	sline = (arcOffset + idx)*baseIndex;		// set sline

  if (end == START)
  {
		if(NorT == 0)
		{
			if (side == LEFT) ExtendLine(sline+=VOTE_ANLS, iscts);
			else if (side == RIGHT) ExtendLine(sline+=VOTE_ANRS, iscts);
		}
		else ExtendLine(sline+=VOTE_ATS, iscts);
	}
	else
	{
		if(NorT == 0)
		{
			if (side == LEFT) ExtendLine(sline+=VOTE_ANLE, iscts);
			else if (side == RIGHT) ExtendLine(sline+=VOTE_ANRE, iscts);
		}
		else ExtendLine(sline+=VOTE_ATE, iscts);
	}
}

/**
 * @brief Extend the end of a search line from an ellipse.
 * @param idx Index of ellipse
 * @param end Ellipse end: LEFT/RIGHT
 */
void VoteImage::ExtendSmartEllipseEnd(unsigned idx, unsigned end)
{
// printf("VoteImage::ExtendSmartEllipseEnd: start!\n");
  unsigned side = RandInt()%2; 			// Inner our outer line?
  sline = (ellOffset + idx)*baseIndex;	

// printf("VoteImage::ExtendSmartEllipseEnd: sline: %u (ellOffset: %u)\n", sline, ellOffset);

  if (end == LEFT)
  {
		if (side == INNER) ExtendLine(sline+=VOTE_EITL, iscts);
		else if (side == OUTER) ExtendLine(sline+=VOTE_EOTL, iscts);
  }
  else 
  {
		if (side == INNER) ExtendLine(sline+=VOTE_EITR, iscts);
		else if (side == OUTER) ExtendLine(sline+=VOTE_EOTR, iscts);
  }
// printf("VoteImage::ExtendSmartEllipseEnd: end!\n");
}


/**
 * @brief Follow the line end: 
 * TODO: maybe following scheme taking into account completed Closures:
 * line itself and T-jct vote for line itself
 * completed closures (of which line is part) vote for any other line
 * L and C vote for hopping
 * P(other) : P(self) : P(hop) = n_clos : 1 + n_t : n_c + n_l + n_r
 * plus: take into accound significances: e.g. a highly significant T-jct votes
 * for tangential extension
 * @param line Line
 * @param end Line end
 * @param stop_line At which line to stop
 */
void VoteImage::FollowEnd(Line *line, int end, Line *stop_line)
{
// printf("FollowEnd: start\n");
  int n_c = line->coll[end].Size();
  int n_l = line->l_jct[end][LEFT].Size();
  int n_r = line->l_jct[end][RIGHT].Size();
  int n_t = (line->t_jct[end] != 0 ? 1 : 0);

  // extend line itself if no junctions at all
  if(n_c + n_l + n_r == 0)
  {
    ExtendEnd(line, end);
  }
  else
  {
    // favour extension of this line if there is a T-jct
    int n_self = 1 + n_t;
    // note: L-jcts belonging to a T-jct do not count -> -2*n_t
    int n_hop = n_c + n_l + n_r - 2*n_t;
    int w = RandInt()%(n_self + n_hop);
    if(w < n_self)
    {
      ExtendEnd(line, end);
    }
    else
    {
      int i = 0;
      Line *fline = 0;
      int fend = UNDEF_ID;

      w = RandInt()%(n_c + n_l + n_r);
      // if one of the collinearities
      if(w < n_c)
      {
        i = w;
        fline = line->coll[end][i]->OtherLine(line);
        fend = Other(line->coll[end][i]->WhichEndIs(fline));
      }
      // if one of the left L-jcts
      else if(w < n_c + n_l)
      {
        i = w - n_c;
        fline = line->l_jct[end][LEFT][i]->line[RIGHT];
        fend = Other(line->l_jct[end][LEFT][i]->near_point[RIGHT]);
      }
      // if on of the right L-jcts
      else
      {
        i = w - n_c - n_l;
        fline = line->l_jct[end][RIGHT][i]->line[LEFT];
        fend = Other(line->l_jct[end][RIGHT][i]->near_point[LEFT]);
      }
      // avoid infinite loops
      if(fline != stop_line)
        FollowEnd(fline, fend, stop_line);
      else
      {
        // select any line at random (ignoring rank)
        fline = Lines(core, RandInt()%NumLines(core));
        fend = RandInt()%2;
        FollowEnd(fline, fend, fline);
      } 
    }
  }
//   printf("FollowEnd: end\n");
}

/**
 * @brief Choose the search line and extend at given line end.
 * @param line Line
 * @param end Line end
 */
void VoteImage::ExtendEnd(Line *line, int end)
{
// printf("ExtendEnd: start\n");
  int r = RandInt();
  // factor for tangent extension: equal chance for open end, favour tangents
  // if T-junction (look for amodal completion)
  int t = 1 + (line->t_jct[end] != 0 ? 1 : 0);
  // factor for normal extension
  int n = 1;
  Line *voteline = line;

  // lines created by splitting defer voting to their original lines
  while(voteline->defer_vote != voteline)
    voteline = voteline->defer_vote;
  sline = voteline->ID()*baseIndex;
  if(r%(t + n) < t) // tangent
  {
    sline += (end == START ? VOTE_TS : VOTE_TE);
  }
  else  // normal
  {
    int side = (r/(t + n))%2;  // equal chance
    if(side == LEFT)
      sline += (end == START ? VOTE_NLS : VOTE_NLE);
    else
      sline += (end == START ? VOTE_NRS : VOTE_NRE);
  }
  ExtendLine(sline, iscts);
//   printf("ExtendEnd: end\n");
}


/**
 * @brief Set the number of search lines
 * @param n Number of lines
 */
void VoteImage::SetNumLines(unsigned n)
{
  lines.Resize(n);
  for(unsigned id = 0; id < lines.Size(); id++)
    lines[id].len = 0;
}

/**
 * @brief Extrend the number of search lines.
 * @param n Number of lines
 */
void VoteImage::ExtendNumLines(unsigned n)
{
// printf("VoteImage::ExtendNumLines: %u to %u\n", lines.Size(), n);
  lines.Resize(n);
  lines[lines.Size()-1].len = 0;
// printf("VoteImage::ExtendNumLines ended\n");
}

/** TODO
 * @brief
 * @
 */
VoteImage::Elem *VoteImage::NewElem(unsigned id)
{
  if(fill < store_size)
  {
    Elem *el = &store[fill++];
    /*el->type = type;*/
    el->id = id;
    el->next = 0;
    return el;
  }
  else
  {
    // TODO: alloc more storage
    printf("VoteImage::NewElem: Error: out of memory!\n");
    return 0;
  }
}

/**
 * @brief Clear the vote image.
 */
void VoteImage::Clear()
{
  memset(data, 0, width*height*sizeof(Elem*));
  fill = 0;
	initialized = false;
}

/** TODO
 * @brief
 * @
 */
inline void VoteImage::SetPixel(int x, int y, unsigned id)
{
  Elem *e = data[y*width + x];
  if(e == 0)
    data[y*width + x] = NewElem(/*type,*/ id);
  else
  {
    while(e->next != 0)
      e = e->next;
    e->next = NewElem(/*type,*/ id);
  }
}

/** 
 * @brief Set new element to the data array and check for intersections. All found intersections
 * will be stored into iscts, independent to already found intersections.
 * @param x x-coordinate in pixel
 * @param y y-coordinate in pixel
 * @param id ID of search line
 * @param iscts Intersections found at this coordinates.
 */
inline void VoteImage::SetAndCheckPixel(int x, int y, unsigned id, Array<Elem> &iscts)
{
  if(x >= width || y >= height) 
  {
    printf("[VoteImage::SetAndCheckPixel] Warning: x,y higher than image width or height!\n");
    return;
  }

  Elem *e = data[y*width + x];
  if(e == 0) {
    data[y*width + x] = NewElem(id);
  }
  else
  {
    Elem *last;
    while(e != 0)
    {
      CreateIntersection(e, id, iscts);
      last = e;
      e = e->next;
    }
    last->next = NewElem(id);
  }
}

/**
 * @brief Check pixel for intersections.
 * @param x x-coordinate of pixel
 * @param y y-coordinate of pixel
 * @param id Index of the Gestalt
 * @param iscts Array with found intersection elements.
 * TODO x,y must be within the image boundaries! => no check!
 */
inline void VoteImage::CheckPixel(int x, int y, unsigned id, Array<Elem> &iscts)
{
  Elem *e = data[y*width + x];
  while(e != 0)
  {
    CreateIntersection(e, id, iscts);
    e = e->next;
  }
}

/**
 * @brief This CheckPixel function is called from FindLineEnd.
 * Lines end only at other visible lines, not tangents or normals.
 * @param x x-coordinate of pixel
 * @param y y-coordinate of pixel
 * @param id Index of the Gestalt
 * @return Returns true, if found
 */
inline bool VoteImage::CheckPixel(int x, int y, unsigned id)
{
  Elem *e = data[y*width + x];
  while(e != 0)
  {
    if(e->id%baseIndex == VOTE_E)
      if(e->id != id)
        return true;
    e = e->next;
  }
  return false;
}

/**
 * @brief Stores the found intersection into iscts-array, after checking.
 * @param e Element to be stored
 * @param id Index of the search line
 * @param iscts Array with intersections
 */
inline void VoteImage::CreateIntersection(Elem *e, unsigned id, Array<Elem> &iscts)
{
// printf("CreateIntersection: start: id: %u - baseIndex: %u\n", id, baseIndex);

// if(e == NULL) printf(" e is EMPTY\n");

// printf("Do we have the element? e->id: %u\n", e->id);

  // don't intersect edge with itself  TODO: why not?
  unsigned first = e->id/baseIndex;
// printf("first: %u\n", first);
  unsigned second = id/baseIndex;
// printf("second: %u\n", second);
  if(first != second) {
//     printf("1\n");
    if(!iscts.ContainsBackwards(*e)) {
//     printf("2\n");
      iscts.PushBack(*e);
//     printf("3\n");
    }
  }
// printf("CreateIntersection: end\n");
}

/**
 * @brief Initialize a new search line.
 * @param x1 x-coordinate of start point
 * @param y1 y-coordinate of start point
 * @param x1 x-coordinate of end point
 * @param y1 y-coordinate of end point
 * @param id Index of the search line
 */
void VoteImage::InitLine(int x1, int y1, int x2, int y2, unsigned id)
{
  LineStub &line = lines[id];
  line.x = line.y = 0;
  line.dx = line.dy = 0;
  line.inc_x = line.inc_y = 0;
  line.len = 0;
  /*line.type = type;*/
  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  line.dx = x2 - x1;
  line.dy = y2 - y1;
  if(line.dx == 0 && line.dy == 0)  // line might be clipped to length 0
    return;
  if(line.dx >= 0)
    line.inc_x = 1;
  else
  {
    line.dx = -line.dx;
    line.inc_x = -1;
  }
  if(line.dy >= 0)
    line.inc_y = 1;
  else
  {
    line.dy = -line.dy;
    line.inc_y = -1;
  }
  line.x = x1;
  line.y = y1;
  if(line.dx >= line.dy)
  {
    // first octant bresenham
    line.err = -line.dx/2;
    line.len = line.dx + 1;
  }
  else
  {
    // second octant bresenham
    line.err = -line.dy/2;
    line.len = line.dy + 1;
  }
}

/**
 * @brief Extends the given search line.
 * Note: We draw dense lines. I.e. whenever increase y, we draw an extra pixel
 * (when drawing line in the first octant).
 *
 * @param id Index of search line to extend.
 * @param iscts  Array contains created intersections after return \n
 * @return Returns the number of created intersections or -1 if the line could not be \n
 * further extended, i.e. its maximum length was reached.
 */
int VoteImage::ExtendLine(unsigned id, Array<Elem> &iscts)
{
// printf("ExtendLine: start\n");
  LineStub &line = lines[id];
  iscts.Clear();
  if(line.len > 0)
  {
    extendedLine = true;
// printf("ExtendLine: 1\n");
    SetAndCheckPixel(line.x, line.y, id, iscts);
// printf("ExtendLine: 2\n");
    line.len--;
    if(line.dx >= line.dy)
    {
      line.err += line.dy;
      if(line.err >= 0)
      {
        line.y += line.inc_y;
        line.err -= line.dx;
// printf("ExtendLine: 3\n");
        if(line.len > 0) { // TODO: this is always true except once
          // make line dense
//  printf("ExtendLine: 4\n");
         SetAndCheckPixel(line.x, line.y, id, iscts);
//  printf("ExtendLine: 5\n");
        }
      }
      line.x += line.inc_x;
    }
    else
    {
//  printf("ExtendLine: 6\n");
      line.err += line.dx;
      if(line.err >= 0)
      {
        line.x += line.inc_x;
        line.err -= line.dy;
        if(line.len > 0)
          // make line dense
//  printf("ExtendLine: 7\n");
          SetAndCheckPixel(line.x, line.y, id, iscts);
//  printf("ExtendLine: 8\n");
      }
      line.y += line.inc_y;
    }
// printf("ExtendLine: end\n");
    return iscts.Size();
  }
  else
  {
// printf("ExtendLine: end -1\n");
    return -1;
  }
}

/**
 * @brief Draw the given search line into the vote image.
 * @param x1 x-coordinate of start point
 * @param y1 y-coordinate of start point
 * @param x1 x-coordinate of end point
 * @param y1 y-coordinate of end point
 * @param id Index of the search line
 */
void VoteImage::DrawLine(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8

  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      SetPixel(x, y, /*type,*/ id);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
	{
          // make line dense
          SetPixel(x, y, /*type,*/ id);
	}
      }
      x += inc_x;
    } while(x != x2); // TODO: x2 is not coloured!
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      SetPixel(x, y, /*type,*/ id);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetPixel(x, y, /*type,*/ id);
      }
      y += inc_y;
    } while(y != y2);
  }
}


/**
 * @brief Check the given search line for intersections.
 * @param x1 x-coordinate of start point
 * @param y1 y-coordinate of start point
 * @param x1 x-coordinate of end point
 * @param y1 y-coordinate of end point
 * @param id Index of the search line
 * @param iscts Array of found intersections
 */
void VoteImage::CheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts)
{
  int dx, dy, inc_x, inc_y, x, y, err;
  unsigned MAX_HYPS = 1000;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      CheckPixel(x, y, /*type,*/ id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          CheckPixel(x, y, /*type,*/ id, iscts);
      }
      x += inc_x;
    } while(x != x2 && iscts.Size() <= MAX_HYPS);
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      CheckPixel(x, y, /*type,*/ id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          CheckPixel(x, y, /*type,*/ id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}

/**
 * @brief Draw and check the given search line for intersections.
 * @param x1 x-coordinate of start point
 * @param y1 y-coordinate of start point
 * @param x1 x-coordinate of end point
 * @param y1 y-coordinate of end point
 * @param id Index of the search line
 * @param iscts Array of found intersections
 */
void VoteImage::DrawAndCheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts)
{
  int dx, dy, inc_x, inc_y, x, y, err;
  unsigned MAX_HYPS = 1000;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      }
      x += inc_x;
    } while(x != x2 && iscts.Size() <= MAX_HYPS);
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}


/**
 * @brief Find the line end.
 * @param x1 x-coordinate of start point
 * @param y1 y-coordinate of start point
 * @param x1 x-coordinate of end point
 * @param y1 y-coordinate of end point
 * @param id Index of the search line
 * @param xe x-coordinate of line end
 * @param ye y-corrdinate of line end
 * @return Returns false if line was clipped to 0 length.
 */
bool VoteImage::FindLineEnd(int x1, int y1, int x2, int y2, unsigned id, int *xe, int *ye)
{
  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return false;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return false;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      if(CheckPixel(x, y, /*type,*/ id))
      {
        *xe = x;
        *ye = y;
        return true;
      }
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          if(CheckPixel(x, y, /*type,*/ id))
          {
            *xe = x;
            *ye = y;
            return true;
          }
      }
      x += inc_x;
    } while(x != x2);
    *xe = x;
    *ye = y;
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      if(CheckPixel(x, y, /*type,*/ id))
      {
        *xe = x;
        *ye = y;
        return true;
      }
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          if(CheckPixel(x, y, /*type,*/ id))
          {
            *xe = x;
            *ye = y;
            return true;
          }
      }
      y += inc_y;
    } while(y != y2);
    *xe = x;
    *ye = y;
  }
  return true;
}

}

