/**
 * @file FormLines.cc
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Class file of Gestalt principle FormLines.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Segment.hh"
#include "Line.hh"
#include "FormLines.hh"
#include "rosin_lines.hh"

namespace Z
{

static int CmpLines(const void *a, const void *b)
{
  if( (*(Line**)a)->sig > (*(Line**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormLines::FormLines(VisionCore *vc) : GestaltPrinciple(vc)
{
  done = false;
  next_principles.PushBack(FORM_JUNCTIONS);
}

void FormLines::Reset()
{
  done = false;
}

void FormLines::PreOperate()
{ 
  // note: we only want to run this once for repeated calls to Operate()
  if(!done)
  {
    StartRunTime();
    Create();
    Rank();
    done = true;
    StopRunTime();
  }
}

/**
 * Create straight lines from segments.
 * Each segment is split up into straight line portions.
 */
void FormLines::Create()
{
  LineSegmenter ls(core);
  for(unsigned rank = 0; rank < core->NumGestalts(Gestalt::SEGMENT); rank++)
  {
    Segment *seg = (Segment*)core->RankedGestalts(Gestalt::SEGMENT, rank);
    ls.RosinFitLinesToSegment(seg);
  }
}

void FormLines::Rank()
{
  RankGestalts(Gestalt::LINE, CmpLines);
}

}

