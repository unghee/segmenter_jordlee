/**
 * $Id: FormParallelLineGroups.cc,v 1.11 2006/11/24 13:47:03 mxz Exp mxz $
 * TODO: only group pairs of lines, each lines remembers NUMHYPS parallels.
 */

#include "Line.hh"
#include "ParallelLineGroup.hh"
#include "FormParallelLineGroups.hh"

namespace Z
{

static int CmpFcnAngle(const void *a, const void *b)
{
  if( ScaleAngle_0_pi(Lines(*(unsigned*)a)->phi) <
      ScaleAngle_0_pi(Lines(*(unsigned*)b)->phi) )
    return -1;  // a is first
  else
    return 1;   // b is first
}

static int CmpParallelLines(const void *a, const void *b)
{
  if( ParallelLineGroups(*(unsigned*)a)->sig > ParallelLineGroups(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormParallelLineGroups::FormParallelLineGroups(Config *cfg)
: GestaltPrinciple(cfg)
{
}

/**
 * Strictly speaking angles are not quantised. Practically they are.
 * Considering a horizontal line of maximal length with phi = 0, the next phi
 * is given as atan(1/width).
 */
double FormParallelLineGroups::AngularResolution()
{
  if(core->HaveImage())
    return atan(1./core->GetImage()->width);
  else
    return M_PI;  // some pointless angular resolution
}

void FormParallelLineGroups::PostOperate()
{
	StartRunTime();
  Create();
  Rank();
	StopRunTime();
}

// TODO: avoid double groups
// TODO: use p_ratio
void FormParallelLineGroups::Create()
{
  /*angles.Resize(NumLines());
  for(unsigned i = 0; i < angles.Size(); i++)
    angles[i] = i;
  angles.Sort(CmpFcnAngle);
  FILE *out = fopen("sigpar", "w");  //!!! QUICK HACK !!!
  for(unsigned i = 0; i < angles.Size(); i++)
  {
    double d = fmax((ScaleAngle_0_pi(Lines(angles[i])->phi) -
          ScaleAngle_0_pi(Lines(angles[0])->phi)), AngularResolution());
    double sig = -(double)i*log(d/M_PI);
    fprintf(out, "%f %f\n", ScaleAngle_0_pi(Lines(angles[i])->phi), sig);
  }
  fclose(out);
  return;*/

  angles.Resize(NumLines());
  ranks.Resize(NumLines());
  for(unsigned i = 0; i < angles.Size(); i++)
    angles[i] = i;
  // sort lines according to angle
  angles.Sort(CmpFcnAngle);
  // fill ranks array
  for(unsigned i = 0; i < ranks.Size(); i++)
    ranks[angles[i]] = i;
  // grow parallel groups
  for(unsigned j = 0; j < ranks.Size(); j++)
  {
    unsigned i = ranks[j];
    if(angles[i] == UNDEF_ID)
      continue;
    unsigned n = 1;
    unsigned idx[2] = {i, i};  // START, END
    double phi[2] = {ScaleAngle_0_pi(Lines(angles[idx[START]])->phi),
      ScaleAngle_0_pi(Lines(angles[idx[END]])->phi)};  // START, END
    double sig_max = 0.;
    bool added_some = false;
    do
    {
      added_some = false;
      // tentatively add previous angle
      // TODO: check for endless loop
      unsigned i_prev = angles.CircularPrev(idx[START]);
      if(angles[i_prev] == UNDEF_ID)
        continue;
      double phi_prev = ScaleAngle_0_pi(Lines(angles[i_prev])->phi);
      double sig_prev = -(double)(n + 1)*log(ScaleAngle_0_pi(phi[END] -
            phi_prev)/M_PI);
      //double sig_prev = -LogBinDist(angles.Size(), n+1,
      //    AngleBetweenLines(phi[END], phi_prev)/M_PI_2);
      // tentatively add next angle
      unsigned i_next = angles.CircularNext(idx[END]);
      if(angles[i_next] == UNDEF_ID)
        continue;
      double phi_next = ScaleAngle_0_pi(Lines(angles[i_next])->phi);
      double sig_next = -(double)(n + 1)*log(ScaleAngle_0_pi(phi_next -
            phi[START])/M_PI);
      //double sig_next = -LogBinDist(angles.Size(), n+1,
      //    AngleBetweenLines(phi_next, phi[START])/M_PI_2);
      if(sig_prev >= sig_max || sig_next >= sig_max)
      {
        if(sig_prev > sig_next)
        {
          // add prev element to start
          sig_max = sig_prev;
          idx[START] = i_prev;
          phi[START] = phi_prev;
        }
        else
        {
          // add next element to end
          sig_max = sig_next;
          idx[END] = i_next;
          phi[END] = phi_next;
        }
        n++;
        added_some = true;
      }
    } while(idx[START] != idx[END] && added_some && n < angles.Size());
    // TODO: some cases might still slip through the while condition resulting
    // in a possible endless loop
    // only accept groups with at least 2 lines
    // NOTE: actually this can never happen: any calculated sig will be > 0
    if(n > 1)
    {
      // temp array of line indices
      Array<unsigned> tmp(n);
      for(unsigned j = idx[START], k = 0; k < n; j = angles.CircularNext(j), k++)
      {
        tmp[k] = angles[j];
        //angles[j] = UNDEF_ID;
      }
      NewGestalt(new ParallelLineGroup(tmp, angles[i]));
    }
  }
}

void FormParallelLineGroups::Rank()
{
  RankGestalts(Gestalt::PARALLEL_LINE_GROUP, CmpParallelLines);
}

}

