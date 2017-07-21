/**
 * @file VoteImage.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief The vote image controls the extension of search lines.
 *
 * TODO: write some macros or inline methods of Link class for properly handling
 *       search line ids (search_line_id = baseIndex*line_id + end_id)
 */

#ifndef Z_VOTE_IMAGE_HH
#define Z_VOTE_IMAGE_HH

#include <math.h>
#include "Array.hh"
#include "Gestalt.hh"
#include "Vector.hh"

namespace Z
{

class Line;
class Arc;
class Ellipse;
class VisionCore;

/// We have following Gestalts in the vote image:
///   - Arcs			=> Arc-Arc
///   - Lines			=> Line-Line
///   - Ellipses	=> Ellipse-Line
/// to estimate the following Gestalts:
///   - AJunctions (== Arc junctions)
///   - LineJunctions (Cols, T-, L-Junctions)
///   - EllipseJunctions (junctions between ellipse vertices and lines)
///   - 

// types of search (vote) lines for line junctions
#define VOTE_NONE  0x0
#define VOTE_E     0x1   // the edge itself
#define VOTE_TS    0x2   // tangent start
#define VOTE_TE    0x3   // tangent end

#define VOTE_NLS   0x4   // normal left start
#define VOTE_NLE   0x5   // normal left end
#define VOTE_NRS   0x6   // normal right start
#define VOTE_NRE   0x7   // normal right end

// for junctions between ellipses and lines (lines without normal search lines)
#define VOTE_EOTL  0x8   // ellipse outer tangent left
#define VOTE_EOTR  0x9   // ellipse outer tangent right
#define VOTE_EITL  0xa   // ellipse inner tangent left
#define VOTE_EITR  0xb   // ellipse inner tangent right

// for junctions between arcs
#define VOTE_ATS   0xc   // arc tangent start
#define VOTE_ATE   0xd   // arc tangent end
#define VOTE_ANLS  0xe   // arc normal left start
#define VOTE_ANLE  0xf   // arc normal left end
#define VOTE_ANRS  0x10  // arc normal right start
#define VOTE_ANRE  0x11  // arc normal right end

#define VOTE_IS_TANGENT(x) ((x) == VOTE_TS || (x) == VOTE_TE)
#define VOTE_IS_NORMAL(x) ((bool)((x) & 0x4))
// return START or END for a vote type
#define VOTE_END(x) ((x) & 0x1)
#define VOTE_VERTEX(x) ((x) & 0x1)


/** @brief Vote image class. */
class VoteImage
{
public:
  /** @brief Class of a vote image element (. */
  class Elem
  {
  public:
    /* Gestalt::Type type;  // line, arc, ellipse, also end types? */
    unsigned id;                                                ///< search line id
    Elem *next;                                                 ///< next element
    Elem() {}                                                   ///< standard constructor for first element
    Elem(const Elem &e) : id(e.id), next(0) {}                  ///< constructor for next elements.
    bool operator==(const Elem &e) { return id == e.id; }
  };

  /** @brief Class of a search line stub. */
  class LineStub
  {
  public:
    int x, y, dx, dy, inc_x, inc_y, err, len;
  };


private:
  bool extendedLine;                                    // flag, showing that a line gots extended
  bool activeJcts, activeEJcts, activeAJcts;
  bool initialized;                                     // TODO SINNLOS, da NIE verwendet!      true, if already initialized
  unsigned baseIndex;                                   // number of search lines
  unsigned arcOffset;                                   // number of lines (== offset to arcs)
  unsigned ellOffset;                                   // number of lines + arcs (== offset to ellipses)
  unsigned isct_ok[18][18];                             // admissibility matrix
  unsigned sline;                                       // Extended search line (id)
  Array<Elem> iscts;                                    // Found intersections
  VisionCore *core;

  void SetupAdmissibilityMatrix();
  void InitSearchLines();
  void InitLineSearchLines(Line *l);
  void InitArcSearchLines(Arc *arc);
  void ExtendSmart(Gestalt::Type type, unsigned idx);
  void ExtendSmartLineEnd(unsigned idx, unsigned end);
  void ExtendSmartArcEnd(unsigned idx, int end);
  void ExtendSmartEllipseEnd(unsigned idx, unsigned end);
  void FollowEnd(Line *line, int end, Line *stop_line);
  void ExtendEnd(Line *line, int end);

private:
  Elem **data;                                           ///< elements ???
  Elem *store;                                           ///< ????
  int store_size;                                        ///< size of store
  int fill;                                              ///< ???
  Array<LineStub> lines;                                 ///< search lines


public:                                                  /// TODO Alle Funktionen in private verschieben!!!
  int width;                                             ///< image width
  int height;                                            ///< image height

  VoteImage(VisionCore *vc, int w, int h);
  virtual ~VoteImage();
  void Initialize();
  void SetNumLines(unsigned n);
  void ExtendNumLines(unsigned n);
  Elem *NewElem(unsigned id);
  void Clear();
  void SetPixel(int x, int y, unsigned id);
  void CheckPixel(int x, int y, unsigned id, Array<Elem> &iscts);
  bool CheckPixel(int x, int y, unsigned id);
  void SetAndCheckPixel(int x, int y, unsigned id, Array<Elem> &iscts);
  void CreateIntersection(Elem *e, unsigned id, Array<Elem> &iscts);
  Elem *Pixel(int x, int y) {return data[y*width + x];}
  void InitLine(int x1, int y1, int x2, int y2, unsigned id);
  void InitLine(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id)
  {
    InitLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  int ExtendLine(unsigned id, Array<Elem> &iscts);
  void DrawLine(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id)
  {
    DrawLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  void DrawLine(int x1, int y1, int x2, int y2, unsigned id);
  void CheckLine(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id, Array<Elem> &iscts)
  {
    CheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id, iscts);
  }
  void CheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts);
  void DrawAndCheckLine(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id, Array<Elem> &iscts)
  {
    DrawAndCheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id, iscts);
  }
  void DrawAndCheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts);
  bool FindLineEnd(int x1, int y1, int x2, int y2, unsigned id, int *xe, int *ye);

public:
  unsigned InitEllipseSearchLines(Ellipse *ell, Array<unsigned> &sl, Array<Elem> &is);
  bool Extend(unsigned &sline, Array<VoteImage::Elem> &is, bool &_extendedLine);
  unsigned IsctTypeAdmissible(int type_i, int type_j) {return isct_ok[type_i][type_j];}
  unsigned GetBaseIndex() {return baseIndex;}
  unsigned GetArcOffset() {return arcOffset;}
  unsigned GetEllOffset() {return ellOffset;}
  bool IsInitialized() {return initialized;}
};

}

#endif

