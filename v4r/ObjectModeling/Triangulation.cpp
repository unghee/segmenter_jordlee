/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author thomas.moerwald
 *
 */

#include "Triangulation.h"

#include "v4r/PCLAddOns/PCLFunctions.h"

#include <stdio.h>
#include "v4r/TomGine/tgShapeCreator.h"
#include "v4r/TomGine/tgRenderModel.h"
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>

using namespace objectmodeling;

// ------------------------------------------------------------------------------------------------
// Interpret two points as being coincident
inline bool
eqPoints (hed::Node*& p1, hed::Node*& p2)
{
  double dx = p1->x () - p2->x ();
  double dy = p1->y () - p2->y ();
  double dist2 = dx * dx + dy * dy;
  const double eps = 1.0e-12;
  if (dist2 < eps)
    return true;

  return false;
}

// ------------------------------------------------------------------------------------------------
// Lexicographically compare two points (2D)
inline bool
ltLexPoint (const hed::Node* p1, const hed::Node* p2)
{
  return (p1->x () < p2->x ()) || (p1->x () == p2->x () && p1->y () < p2->y ());
}

hed::Edge*
getBoundaryEdgeInTriangle (hed::Edge* edge)
{

  if (ttl::isBoundaryEdge (hed::Dart (edge)))
    return edge;
  edge = edge->getNextEdgeInFace ();
  if (ttl::isBoundaryEdge (hed::Dart (edge)))
    return edge;
  edge = edge->getNextEdgeInFace ();
  if (ttl::isBoundaryEdge (hed::Dart (edge)))
    return edge;

  return NULL;
}

void
Triangulation::reverse (ON_NurbsCurve &curve, bool z_negative)
{
  ON_3dPoint p0;
  curve.GetCV (0, p0);

  double z (0.0);

  for (int i = 1; i < curve.CVCount () - 1; i++)
  {
    ON_3dPoint p1, p2;
    curve.GetCV (i, p1);
    curve.GetCV (i + 1, p2);

    z += p1.x * p2.y - p1.y * p2.x;
  }

  if ((z_negative && z <= 0.0) || (!z_negative && z >= 0))
  {
    ON_NurbsCurve curve2 = curve;
    for (int i = 0; i < curve.CVCount (); i++)
    {
      int j = curve.CVCount () - 1 - i;
      ON_3dPoint p;
      curve.GetCV (i, p);
      curve2.SetCV (j, p);
    }
    curve = curve2;
  }
}

void
Triangulation::flip (int dir, ON_NurbsSurface &nurbs)
{
  ON_NurbsSurface result (nurbs);

  for (int i = 0; i < nurbs.CVCount (0); i++)
  {
    for (int j = 0; j < nurbs.CVCount (1); j++)
    {
      ON_3dPoint cp;
      nurbs.GetCV (i, j, cp);
      if (dir == 0)
        result.SetCV (nurbs.CVCount (0) - i - 1, j, cp);
      else if (dir == 1)
        result.SetCV (i, nurbs.CVCount (1) - j - 1, cp);
    }
  }

  nurbs = result;
}

Eigen::Vector2d
Triangulation::intersectNurbsLine (const ON_NurbsSurface &nurbs, Eigen::Vector3d p0, Eigen::Vector3d dir, double hint,
                                   unsigned steps, double accuracy)
{
  Eigen::Vector3d pt = p0 + dir * hint;

  double error;
  Eigen::Vector3d p, tu, tv;
  Eigen::Vector2d params;

  for (unsigned i = 0; i < steps; i++)
  {
    params = pcl::on_nurbs::FittingSurface::findClosestElementMidPoint (nurbs, pt);
    params = pcl::on_nurbs::FittingSurface::inverseMapping (nurbs, pt, params, error, p, tu, tv, 200, 1e-6);

    pt = dir * (dir.dot (p - p0));

    if (error < accuracy)
      return params;
  }

  return params;
}

Eigen::Vector3d
Triangulation::intersectPlaneLine (const std::vector<float> &plane, Eigen::Vector3d p0, Eigen::Vector3d dir)
{
  Eigen::Vector3d n (plane[0], plane[1], plane[2]);
  double norm = n.norm ();
  double d = plane[3] / norm;
  n /= norm;

  // intersect plane
  double z = -(p0.dot (n) + d) / (dir.dot (n));
  return p0 + dir * z;
}

bool
Triangulation::isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance, bool z_negative)
{
  if (curve.KnotCount () < 1)
    return false;

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  double err;
  Eigen::Vector2d p, t;
  double param;
  if (curve.Order () == 2)
    param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, point, err, p, t);
  else
  {
    param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, point);
    param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, point, param, err, p, t, rScale);
  }

  Eigen::Vector2d v (point (0) - p (0), point (1) - p (1));
  double z = v (0) * t (1) - v (1) * t (0);

  return ((z_negative && z <= 0.0) || (!z_negative && z >= 0) || v.squaredNorm () < (tolerance * tolerance));
}

bool
Triangulation::isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance, double rScale,
                              bool z_negative)
{
  if (curve.KnotCount () < 1)
    return false;

  double err;
  Eigen::Vector2d p, t;
  double param;
  if (curve.Order () == 2)
    param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, point, err, p, t);
  else
  {
    param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, point);
    param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, point, param, err, p, t, rScale);
  }

  Eigen::Vector2d v (point (0) - p (0), point (1) - p (1));
  double z = v (0) * t (1) - v (1) * t (0);

  return ((z_negative && z <= 0.0) || (!z_negative && z >= 0) || v.squaredNorm () < (tolerance * tolerance));
}

void
Triangulation::getPlaneBasis (const std::vector<float> &plane, Eigen::Matrix3d &R, Eigen::Vector3d &t)
{
  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d n (plane[0], plane[1], plane[2]);
  n.normalize ();

  if (n (0) == 0.0 || n (2) == 0.0)
    printf ("[ContourRefinement::getPlaneBasis ] Error, plane normal not suitable\n");

  t = intersectPlaneLine (plane, p0, n);

  Eigen::Vector3d x (1.0, 0.0, 0.0);
  x (2) = -x (0) * n (0) / n (2);
  x.normalize ();
  Eigen::Vector3d y = n.cross (x);

  R << x (0), y (0), n (0), x (1), y (1), n (1), x (2), y (2), n (2);
}

void
Triangulation::computeBoundingBox (const pcl::on_nurbs::vector_vec2d &points, Eigen::Vector2d &_min,
                                   Eigen::Vector2d &_max)
{
  _min = Eigen::Vector2d (DBL_MAX, DBL_MAX);
  _max = Eigen::Vector2d (-DBL_MAX, -DBL_MAX);
  for (size_t i = 0; i < points.size (); i++)
  {
    const Eigen::Vector2d &p = points[i];

    if (p (0) < _min (0))
      _min (0) = p (0);
    if (p (1) < _min (1))
      _min (1) = p (1);

    if (p (0) > _max (0))
      _max (0) = p (0);
    if (p (1) > _max (1))
      _max (1) = p (1);
  }
}

void
Triangulation::computeBoundingBox (const pcl::on_nurbs::vector_vec3d &points, Eigen::Vector3d &_min,
                                   Eigen::Vector3d &_max)
{
  _min = Eigen::Vector3d (DBL_MAX, DBL_MAX, DBL_MAX);
  _max = Eigen::Vector3d (-DBL_MAX, -DBL_MAX, -DBL_MAX);
  for (size_t i = 0; i < points.size (); i++)
  {
    const Eigen::Vector3d &p = points[i];

    if (p (0) < _min (0))
      _min (0) = p (0);
    if (p (1) < _min (1))
      _min (1) = p (1);
    if (p (2) < _min (2))
      _min (2) = p (2);

    if (p (0) > _max (0))
      _max (0) = p (0);
    if (p (1) > _max (1))
      _max (1) = p (1);
    if (p (2) > _max (2))
      _max (2) = p (2);
  }
}

void
Triangulation::removeUnusedVertices (TomGine::tgModel &model)
{
  // get vertices in use
  std::vector<bool> v_used (model.m_vertices.size (), false);
  for (size_t i = 0; i < model.m_faces.size (); i++)
  {
    TomGine::tgFace &f = model.m_faces[i];
    for (size_t j = 0; j < f.v.size (); j++)
      v_used[f.v[j]] = true;
  }

  // calculate relation between old and new indices
  std::map<size_t, size_t> idxmap;
  size_t idx (0);
  for (size_t i = 0; i < v_used.size (); i++)
  {
    if (v_used[i] == true)
    {
      idxmap[i] = idx;
      idx++;
    }
  }

  // update indices of faces
  for (size_t i = 0; i < model.m_faces.size (); i++)
  {
    TomGine::tgFace &f = model.m_faces[i];
    for (size_t j = 0; j < f.v.size (); j++)
      f.v[j] = idxmap[f.v[j]];
  }

  // reallocate vertices
  std::vector<TomGine::tgVertex> vertices (idxmap.size ());
  std::map<size_t, size_t>::iterator it;
  for (it = idxmap.begin (); it != idxmap.end (); it++)
    vertices[it->second] = model.m_vertices[it->first];

  model.m_vertices = vertices;
}

void
Triangulation::convertNurbs2Plane (const ON_NurbsSurface &surf, std::vector<float> &coeffs)
{
  if (surf.Order (0) != 2 || surf.Order (1) != 2 || surf.CVCount (0) > 2 || surf.CVCount (1) > 2)
  {
    printf ("[Triangulation::convertNurbs2Plane] Error, surface is not planar\n");
    return;
  }

  ON_3dPoint a, b, c;
  surf.GetCV (0, 0, a);
  surf.GetCV (1, 0, b);
  surf.GetCV (1, 1, c);

  ON_3dVector e1 = b - a;
  ON_3dVector e2 = c - a;

  ON_3dVector n = ON_CrossProduct (e1, e2);

  n /= (-n.Length ()); // normals of Nurbs are pointing in wrong direction (due to image space convention)

  coeffs.clear ();
  coeffs.push_back (n.x);
  coeffs.push_back (n.y);
  coeffs.push_back (n.z);
  coeffs.push_back (-a.x * n.x - a.y * n.y - a.z * n.z);
}

void
Triangulation::convertNurbs2tgModel (const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned resU,
                                     unsigned resV, bool cps)
{
  if (nurbs.IsPeriodic (1))
    convertNurbsCylinder2tgModel (nurbs, model, resU, resV, cps);

  if (!nurbs.IsPeriodic (0) && !nurbs.IsPeriodic (1))
    convertNurbsPatch2tgModel (nurbs, model, resU, resV, cps);
}

void
Triangulation::convertNurbsPatch2tgModel (const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned resU,
                                          unsigned resV, bool cps)
{
  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  model.Clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
  double h = y1 - y0;

  TomGine::tgShapeCreator::CreatePlaneXY (model, x0, y0, 0.0, w, h, resU, resV);

  for (unsigned i = 0; i < model.m_vertices.size (); i++)
  {

    TomGine::tgVertex v = model.m_vertices[i];

    double pointAndTangents[9];
    nurbs.Evaluate (v.pos.x, v.pos.y, 1, 3, pointAndTangents);

    TomGine::vec3 tu, tv;
    model.m_vertices[i].pos.x = pointAndTangents[0];
    model.m_vertices[i].pos.y = pointAndTangents[1];
    model.m_vertices[i].pos.z = pointAndTangents[2];

    tu.x = pointAndTangents[3]; // use tu
    tu.y = pointAndTangents[4];
    tu.z = pointAndTangents[5];
    tv.x = pointAndTangents[6]; // use tv
    tv.y = pointAndTangents[7];
    tv.z = pointAndTangents[8];

    model.m_vertices[i].normal = TomGine::normalize (TomGine::cross (tu, tv));

  }

  if (cps)
  {
    model.m_points.clear ();
    model.m_point_size = 2.0f;
    for (int j = 0; j < nurbs.CVCount (1); j++)
    {
      for (int i = 0; i < nurbs.CVCount (0); i++)
      {
        ON_3dPoint p;
        nurbs.GetCV (i, j, p);
        model.m_points.push_back (TomGine::vec3 (p.x, p.y, p.z));
      }
    }
  }
}

//void
//Triangulation::Convert (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, TomGine::tgModel &model,
//                        unsigned res, bool cps)
//{
//  // copy knots
//  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
//  {
//    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
//    return;
//  }
//
//  model.Clear ();
//
//  double x0 = nurbs.Knot (0, 0);
//  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
//  double w = x1 - x0;
//  double y0 = nurbs.Knot (1, 0);
//  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
//  double h = y1 - y0;
//
//  TomGine::tgShapeCreator::CreatePlaneXY (model, x0, y0, 0.0, w, h, res, res);
//
//  Eigen::Vector3d a0, a1;
//  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
//  double rScale = pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);
//
//  for (unsigned i = 0; i < model.m_vertices.size (); i++)
//  {
//
//    TomGine::tgVertex v = model.m_vertices[i];
//
//    Eigen::Vector2d pcp (v.pos.x, v.pos.y), pt, tc;
//    double err;
//    double param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, pcp);
//    pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, pcp, param, err, pt, tc, rScale);
//    Eigen::Vector3d a (pcp (0) - pt (0), pcp (1) - pt (1), 0.0);
//    Eigen::Vector3d b (tc (0), tc (1), 0.0);
//    Eigen::Vector3d z = a.cross (b);
//    if (z (2) < 0.0)
//    {
//      v.pos.x = FLT_MAX;
//      v.pos.y = FLT_MAX;
//    }
//
//    double pointAndTangents[9];
//    nurbs.Evaluate (v.pos.x, v.pos.y, 1, 3, pointAndTangents);
//
//    TomGine::vec3 tu, tv;
//    model.m_vertices[i].pos.x = pointAndTangents[0];
//    model.m_vertices[i].pos.y = pointAndTangents[1];
//    model.m_vertices[i].pos.z = pointAndTangents[2];
//
//    tu.x = pointAndTangents[3]; // use tu
//    tu.y = pointAndTangents[4];
//    tu.z = pointAndTangents[5];
//    tv.x = pointAndTangents[6]; // use tv
//    tv.y = pointAndTangents[7];
//    tv.z = pointAndTangents[8];
//
//    model.m_vertices[i].normal = TomGine::normalize (TomGine::cross (tu, tv));
//
//  }
//
//  if (cps)
//  {
//    model.m_colorpoints.clear ();
//    model.m_point_size = 2.0f;
//    for (int j = 0; j < nurbs.CVCount (1); j++)
//    {
//      for (int i = 0; i < nurbs.CVCount (0); i++)
//      {
//        ON_3dPoint p;
//        nurbs.GetCV (i, j, p);
//        TomGine::tgColorPoint cp;
//        cp.pos = TomGine::vec3 (p.x, p.y, p.z);
//        cp.color[0] = 0;
//        cp.color[1] = 255;
//        cp.color[2] = 0;
//        model.m_colorpoints.push_back (cp);
//      }
//    }
//  }
//}

void
Triangulation::convertTrimmedSurface2tgModel (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve,
                                              TomGine::tgModel &model, unsigned resU, unsigned resV, bool cps)
{
  // copy knots
  if (nurbs.KnotCount (0) <= 1 || nurbs.KnotCount (1) <= 1 || curve.KnotCount () <= 1)
  {
    printf ("[Triangulation::convertTrimmedSurface2tgModel] Warning: ON knot vector empty.\n");
    return;
  }

  model.Clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.KnotCount (0) - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.KnotCount (1) - 1);
  double h = y1 - y0;

  TomGine::tgShapeCreator::CreatePlaneXY (model, x0, y0, 0.0, w, h, resU, resV);

  pcl::on_nurbs::vector_vec2d points (model.m_vertices.size (), Eigen::Vector2d ());
  std::vector<double> params (model.m_vertices.size (), 0.0);
  std::vector<bool> pt_is_in (model.m_vertices.size (), false);

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  std::vector<uint32_t> out_idx;
  pcl::on_nurbs::vector_vec2d out_pc;

  for (unsigned i = 0; i < model.m_vertices.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[i];
    Eigen::Vector2d vp (v.pos.x, v.pos.y);

    double err, param;
    Eigen::Vector2d pc, tc;
    if (curve.Order () == 2)
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, vp, err, pc, tc);
    else
    {
      param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, vp);
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, vp, param, err, pc, tc, rScale);
    }

    Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
    Eigen::Vector3d b (tc (0), tc (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    points[i] = pc;
    params[i] = param;
    // TODO quick and dirty hack, remove:
    pt_is_in[i] = (z (2) >= 0.0);
  }

  std::vector<TomGine::tgFace> faces;
  for (unsigned i = 0; i < model.m_faces.size (); i++)
  {
    unsigned in (0);
    TomGine::tgFace &face = model.m_faces[i];

    std::vector<uint32_t> out_idx_tmp;
    pcl::on_nurbs::vector_vec2d out_pc_tmp;

    for (std::size_t j = 0; j < face.v.size (); j++)
    {
      unsigned &vi = face.v[j];
      if (pt_is_in[vi])
        in++;
      else
      {
        out_idx_tmp.push_back (vi);
        out_pc_tmp.push_back (points[vi]);
      }
    }

    if (in > 0)
    {
      faces.push_back (face);
      if (in < face.v.size ())
      {
        for (std::size_t j = 0; j < out_idx_tmp.size (); j++)
        {
          out_idx.push_back (out_idx_tmp[j]);
          out_pc.push_back (out_pc_tmp[j]);
        }
      }
    }
  }

  for (std::size_t i = 0; i < out_idx.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[out_idx[i]];
    Eigen::Vector2d &pc = out_pc[i];
    v.pos.x = pc (0);
    v.pos.y = pc (1);
  }

  for (std::size_t i = 0; i < model.m_vertices.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[i];
    Eigen::Vector3d tu, tv, n;

    double point[9];
    nurbs.Evaluate (v.pos.x, v.pos.y, 1, 3, point);

    v.pos.x = point[0];
    v.pos.y = point[1];
    v.pos.z = point[2];
    tu[0] = point[3];
    tu[1] = point[4];
    tu[2] = point[5];
    tv[0] = point[6];
    tv[1] = point[7];
    tv[2] = point[8];

    tu.normalize ();
    tv.normalize ();
    n = tu.cross (tv);

    v.normal = TomGine::vec3 (n (0), n (1), n (2));
  }

  model.m_faces = faces;

  removeUnusedVertices (model);
}

void
Triangulation::convertBrep2tgModel (const ON_Brep &brep, TomGine::tgModel &model, unsigned resF, bool cps)
{
  model.Clear ();

  for (int fi = 0; fi < brep.m_F.Count (); fi++)
  {
    const ON_BrepFace& face = brep.m_F[fi];

    // pSrf = underlying untrimmed surface
    const ON_Surface* pSrf = NULL;
    if (face.m_si < 0 || face.m_si >= brep.m_S.Count ())
    {
      printf ("[Triangulation::convertBrep2tgModel] ERROR: invalid brep.m_F[%d].m_si\n", fi);
      return;
    }
    else
    {
      pSrf = brep.m_S[face.m_si];
      if (!pSrf)
      {
        printf ("[Triangulation::convertBrep2tgModel] ERROR: invalid brep.m_S[%d] is NULL\n", face.m_si);
        return;
      }
    }

    if (pSrf->ClassId () != ON_NurbsSurface ().ClassId ())
    {
      printf ("[Triangulation::convertBrep2tgModel] ERROR: Surface class not supported: %s\n",
              pSrf->ClassId ()->ClassName ());
      return;
    }

    ON_NurbsSurface* pSurf = (ON_NurbsSurface*)pSrf;
    TomGine::tgModel model_face;

    Eigen::Vector2i res;
    for (int i = 0; i < 2; i++)
    {
      res (i) = 2 * pSurf->CVCount (i);
      if (pSurf->Order (i) > 2)
        res (i) = 2 * pSurf->Order (i) * pSurf->CVCount (i);
    }

    //////////////////////////////////////////////////////
    // 2d trimming information

    // loop_count = number of trimming loops on this face (>=1)
    const int loop_count = face.m_li.Count ();

    if (loop_count > 1 || loop_count < 0)
    {
      printf ("[Triangulation::convertBrep2tgModel] ERROR: only 1 trimming loop supported\n");
      return;
    }

    if (loop_count == 0)
    {
      printf ("[Triangulation::convertBrep2tgModel] Warning, no trimming curve available! Triangulating without!\n");
      convertNurbs2tgModel (*pSurf, model_face, res (0), res (1), cps);
    }
    else
    {
      // face's loop index
      const int li = face.m_li[0]; // li = brep loop index
      const ON_BrepLoop& loop = brep.m_L[li];

      // loop_edge_count = number of trimming edges in this loop
      const int loop_trim_count = loop.m_ti.Count ();

      if (loop_trim_count != 1)
      {
        printf ("[Triangulation::convertBrep2tgModel] ERROR: only 1 curve for trimming (closed) supported\n");
        return;
      }

      const int ti = loop.m_ti[0]; // ti = brep trim index
      const ON_BrepTrim& trim = brep.m_T[ti];

      // Each trim has a 2d parameter space curve.
      const ON_Curve* p2dCurve = NULL;
      const int c2i = trim.m_c2i; // c2i = brep 2d curve index
      if (c2i < 0 || c2i >= brep.m_C2.Count ())
      {
        printf ("[Triangulation::convertBrep2tgModel] ERROR: invalid brep.m_T[%d].m_c2i\n", ti);
        return;
      }
      else
      {
        p2dCurve = brep.m_C2[c2i];
        if (!p2dCurve)
        {
          printf ("[Triangulation::convertBrep2tgModel] ERROR: invalid brep.m_C2[%d] is NULL\n", c2i);
          return;
        }
      }

      if (p2dCurve->ClassId () != ON_NurbsCurve ().ClassId ())
      {
        printf ("[Triangulation::convertBrep2tgModel] ERROR: trimming curve not supported: %s\n",
                p2dCurve->ClassId ()->ClassName ());
        return;
      }

      ON_NurbsCurve* pCurve = (ON_NurbsCurve*)p2dCurve;

      convertTrimmedSurface2tgModel (*pSurf, *pCurve, model_face, res (0), res (1), cps);
    }

    model.Merge (model_face);
  }

}

void
Triangulation::convertNurbsCylinder2tgModel (const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned resU,
                                             unsigned resV, bool cps)
{
  // copy knots
  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  model.Clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, nurbs.m_order[1] - 2);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1 - nurbs.m_order[1] + 2);
  double h = y1 - y0;

  TomGine::tgShapeCreator::CreatePlaneXY (model, x0, y0, 0.0, w, h, resU, resV);

  for (unsigned i = 0; i < model.m_vertices.size (); i++)
  {

    TomGine::tgVertex v = model.m_vertices[i];

    double pointAndTangents[9];
    nurbs.Evaluate (v.pos.x, v.pos.y, 1, 3, pointAndTangents);

    TomGine::vec3 tu, tv;
    model.m_vertices[i].pos.x = pointAndTangents[0];
    model.m_vertices[i].pos.y = pointAndTangents[1];
    model.m_vertices[i].pos.z = pointAndTangents[2];

    tu.x = pointAndTangents[3]; // use tu
    tu.y = pointAndTangents[4];
    tu.z = pointAndTangents[5];
    tv.x = pointAndTangents[6]; // use tv
    tv.y = pointAndTangents[7];
    tv.z = pointAndTangents[8];

    model.m_vertices[i].normal = TomGine::normalize (TomGine::cross (tu, tv));

  }

  if (cps)
  {
    model.m_points.clear ();
    model.m_point_size = 2.0f;
    for (int j = 0; j < nurbs.CVCount (1); j++)
    {
      for (int i = 0; i < nurbs.CVCount (0); i++)
      {
        ON_3dPoint p;
        nurbs.GetCV (i, j, p);
        model.m_points.push_back (TomGine::vec3 (p.x, p.y, p.z));
      }
    }
  }
}

void
Triangulation::Convert (const ON_NurbsCurve &nurbs, TomGine::tgModel &model, unsigned res, bool cps)
{
  // copy knots
  if (nurbs.m_knot_capacity <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  if (res < 2)
    res = 2;

  int cp_red = nurbs.Order () - 2;

  model.Clear ();
  for (int i = cp_red; i < nurbs.KnotCount () - 1 - cp_red; i++)
  {
    TomGine::tgLine line;

    double dr = 1.0 / (res - 1);

    double xi0 = nurbs.m_knot[i];
    double xid = (nurbs.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < res - 1; j++)
    {

      double xi1 = (xi0 + j * dr * xid);
      double xi2 = (xi0 + (j + 1) * dr * xid);

      double points[3];

      nurbs.Evaluate (xi1, 0, 3, points);
      line.start.x = points[0];
      line.start.y = points[1];
      line.start.z = points[2];

      nurbs.Evaluate (xi2, 0, 3, points);
      line.end.x = points[0];
      line.end.y = points[1];
      line.end.z = points[2];

      model.m_lines.push_back (line);
    }

  }

  if (cps)
  {
    model.m_points.clear ();
    model.m_point_size = 5.0f;
    for (int i = 0; i < nurbs.CVCount (); i++)
    {
      ON_3dPoint p;
      nurbs.GetCV (i, p);
      TomGine::vec3 tgP = TomGine::vec3 (p.x, p.y, p.z);
      model.m_points.push_back (tgP);
    }
  }
}

void
Triangulation::Convert (const ON_NurbsCurve &curve, const ON_NurbsSurface &nurbs, TomGine::tgModel &model,
                        unsigned res, bool cps)
{
  // copy knots
  if (curve.m_knot_capacity <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  if (res < 2)
    res = 2;

  int cp_red = curve.Order () - 2;

  model.Clear ();
  for (int i = 1; i < curve.KnotCount () - 1 - cp_red; i++)
  {
    TomGine::tgLine line;

    std::vector<double> xi;
    double dr = 1.0 / (res - 1);

    double xi0 = curve.m_knot[i];
    double xid = (curve.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < res; j++)
    {
      xi.push_back (xi0 + j * dr * xid);
    }

    for (unsigned j = 0; j < xi.size () - 1; j++)
    {

      double p[3];
      double pp[3];
      curve.Evaluate (xi[j], 0, 2, pp);
      nurbs.Evaluate (pp[0], pp[1], 0, 3, p);
      line.start.x = p[0];
      line.start.y = p[1];
      line.start.z = p[2];

      curve.Evaluate (xi[j + 1], 0, 2, pp);
      nurbs.Evaluate (pp[0], pp[1], 0, 3, p);
      line.end.x = p[0];
      line.end.y = p[1];
      line.end.z = p[2];

      model.m_lines.push_back (line);
    }

  }

  if (cps)
  {
    model.m_points.clear ();
    model.m_point_size = 5.0f;
    for (int i = 0; i < curve.CVCount (); i++)
    {
      ON_3dPoint p;
      curve.GetCV (i, p);
      double pp[3];
      nurbs.Evaluate (p.x, p.y, 0, 3, pp);
      TomGine::vec3 tgP = TomGine::vec3 (pp[0], pp[1], pp[2]);
      model.m_points.push_back (tgP);
    }
  }
}

void
Triangulation::convertPlane2tgModel (const std::vector<float> &plane, TomGine::tgModel &model,
                                     const Eigen::Vector4d &bb, unsigned res)
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  getPlaneBasis (plane, R, t);

  TomGine::tgShapeCreator::CreatePlaneXY (model, bb (0), bb (2), 0.0, bb (1) - bb (0), bb (3) - bb (2), res, res);

  for (size_t i = 0; i < model.m_vertices.size (); i++)
  {
    TomGine::vec3 &v = model.m_vertices[i].pos;

    Eigen::Vector3d p (v.x, v.y, v.z);

    p = R * p + t;

    v = TomGine::vec3 (p (0), p (1), p (2));

    model.m_vertices[i].normal = TomGine::vec3 (plane[0], plane[1], plane[2]);
  }

}

void
Triangulation::convertPlane2tgModel (const pcl::on_nurbs::vector_vec3d &points, TomGine::tgModel &model, unsigned res)
{
  Eigen::Vector3d mean, eigenvalues;
  Eigen::Matrix3d eigenvectors;

  pcl::on_nurbs::NurbsTools::pca (points, mean, eigenvectors, eigenvalues);

  Eigen::Vector3d z = eigenvectors.col (2);
  z.normalize ();
  double d = -z.dot (mean);

  std::vector<float> coeffs (4);
  coeffs[0] = z (0);
  coeffs[1] = z (1);
  coeffs[2] = z (2);
  coeffs[3] = d;

  Eigen::Matrix3d R, Rt;
  Eigen::Vector3d t;
  objectmodeling::Triangulation::getPlaneBasis (coeffs, R, t);
  Rt = R.transpose ();

  pcl::on_nurbs::vector_vec3d pp;
  for (size_t i = 0; i < points.size (); i++)
  {
    const Eigen::Vector3d &p1 = points[i];
    Eigen::Vector3d p2 = Rt * p1 - Rt * t;
    pp.push_back (p2);
  }

  Eigen::Vector3d a, b;
  objectmodeling::Triangulation::computeBoundingBox (pp, a, b);

  Eigen::Vector4d bb (a (0), b (0), a (1), b (1));
  objectmodeling::Triangulation::convertPlane2tgModel (coeffs, model, bb, res);
}

void
Triangulation::convertCurveOnPlane2tgModel (const ON_NurbsCurve &curve, const std::vector<float> &plane,
                                            TomGine::tgModel &model, unsigned res, bool cps)
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  Eigen::Vector3d pp;

  getPlaneBasis (plane, R, t);

  // copy knots
  if (curve.KnotCount () <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  if (res < 2)
    res = 2;

  int cp_red = curve.Order () - 2;

  model.Clear ();
  for (int i = 1; i < curve.KnotCount () - 1 - cp_red; i++)
  {
    TomGine::tgLine line;

    std::vector<double> xi;
    double dr = 1.0 / (res - 1);
    double xi0 = curve.m_knot[i];
    double xid = (curve.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < res; j++)
    {
      xi.push_back (xi0 + j * dr * xid);
    }

    for (unsigned j = 0; j < xi.size () - 1; j++)
    {
      double p[2];
      curve.Evaluate (xi[j], 0, 2, p);
      pp = Eigen::Vector3d (p[0], p[1], 0.0);
      pp = R * pp + t;
      line.start.x = pp[0];
      line.start.y = pp[1];
      line.start.z = pp[2];

      curve.Evaluate (xi[j + 1], 0, 2, p);
      pp = Eigen::Vector3d (p[0], p[1], 0.0);
      pp = R * pp + t;
      line.end.x = pp[0];
      line.end.y = pp[1];
      line.end.z = pp[2];

      model.m_lines.push_back (line);
    }

  }

  if (cps)
  {
    model.m_points.clear ();
    model.m_point_size = 5.0f;
    for (int i = 0; i < curve.CVCount (); i++)
    {
      ON_3dPoint cp;
      curve.GetCV (i, cp);
      Eigen::Vector3d pp (cp.x, cp.y, cp.z);
      pp = R * pp + t;
      model.m_points.push_back (TomGine::vec3 (pp[0], pp[1], pp[2]));
    }
  }

}

void
Triangulation::convertTrimmedPlane2tgModel (const std::vector<float> &plane, const ON_NurbsCurve &curve,
                                            TomGine::tgModel &model, unsigned res, bool cps)
{
  // copy knots
  if (plane.size () != 4 || curve.KnotCount () <= 1)
  {
    printf ("[Triangulation::convertTrimmedPlane2tgModel] Warning, not a plane / curve.\n");
    return;
  }

  model.Clear ();

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  TomGine::tgShapeCreator::CreatePlaneXY (model, a0 (0), a0 (1), 0.0, a1 (0) - a0 (0), a1 (1) - a0 (1), res, res);

  pcl::on_nurbs::vector_vec2d points (model.m_vertices.size (), Eigen::Vector2d ());
  std::vector<double> params (model.m_vertices.size (), 0.0);
  std::vector<bool> pt_is_in (model.m_vertices.size (), false);

  std::vector<uint32_t> out_idx;
  pcl::on_nurbs::vector_vec2d out_pc;

  for (unsigned i = 0; i < model.m_vertices.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[i];
    Eigen::Vector2d vp (v.pos.x, v.pos.y);

    double err, param;
    Eigen::Vector2d pc, tc;
    if (curve.Order () == 2)
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, vp, err, pc, tc);
    else
    {
      param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, vp);
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, vp, param, err, pc, tc, rScale);
    }

    Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
    Eigen::Vector3d b (tc (0), tc (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    points[i] = pc;
    params[i] = param;
    pt_is_in[i] = (z (2) >= 0.0);
  }

  std::map<size_t, size_t> vi_map;
  std::vector<TomGine::tgVertex> vertices;
  std::vector<TomGine::tgFace> faces;
  for (unsigned i = 0; i < model.m_faces.size (); i++)
  {
    unsigned in (0);
    TomGine::tgFace &face = model.m_faces[i];

    std::vector<uint32_t> out_idx_tmp;
    pcl::on_nurbs::vector_vec2d out_pc_tmp;

    for (std::size_t j = 0; j < face.v.size (); j++)
    {
      unsigned &vi = face.v[j];
      if (pt_is_in[vi])
        in++;
      else
      {
        out_idx_tmp.push_back (vi);
        out_pc_tmp.push_back (points[vi]);
      }
    }

    if (in > 0)
    {

      //      for (size_t j = 0; j < face.v.size (); j++)
      //      {
      //        unsigned &vi = face.v[j];
      //        if (vi_map.count (vi) > 0)
      //        {
      //          face.v[j] = vi_map[vi];
      //        }
      //        else
      //        {
      //          vi_map[vi] = vertices.size();
      //          face.v[j] = vertices.size ();
      //          vertices.push_back (model.m_vertices[vi]);
      //        }
      //      }

      faces.push_back (face);
      if (in < face.v.size ())
      {
        for (std::size_t j = 0; j < out_idx_tmp.size (); j++)
        {
          out_idx.push_back (out_idx_tmp[j]);
          out_pc.push_back (out_pc_tmp[j]);
        }
      }
    }
  }

  for (std::size_t i = 0; i < out_idx.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[out_idx[i]];
    Eigen::Vector2d &pc = out_pc[i];
    v.pos.x = pc (0);
    v.pos.y = pc (1);
  }
  model.m_faces = faces;
  //  model.m_vertices = vertices;

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  getPlaneBasis (plane, R, t);

  for (std::size_t i = 0; i < model.m_vertices.size (); i++)
  {
    TomGine::tgVertex &v = model.m_vertices[i];

    Eigen::Vector3d p (v.pos.x, v.pos.y, v.pos.z);

    p = R * p + t;

    v.pos = TomGine::vec3 (p (0), p (1), p (2));

    v.normal = TomGine::vec3 (plane[0], plane[1], plane[2]);
  }

  removeUnusedVertices (model);
}

//void Triangulation::Convert(const ON_NurbsCurve &curve, const ON_NurbsSurface &nurbs,
//    TomGine::tgModel &model, unsigned res, bool cps)
//{
//  // copy knots
//  if (curve.m_knot_capacity <= 1) {
//    printf("[Triangulation::Convert] Warning: ON knot vector empty.\n");
//    return;
//  }
//
//  model.Clear();
//
//  double x0 = curve.Knot(curve.m_order - 2);
//  double x1 = curve.Knot(curve.m_knot_capacity - 1 - curve.m_order + 2);
//
//  TomGine::tgLine line(TomGine::vec3(x0, 0.0, 0.0), TomGine::vec3(x1, 0.0, 0.0));
//
//  TomGine::tgShapeCreator::CreateLine(model, line, res);
//
//  for (unsigned i = 0; i < model.m_lines.size(); i++) {
//
//    TomGine::tgLine &v = model.m_lines[i];
//
//    double pp[2];
//    double p[3];
//    curve.Evaluate(v.start.x, 0, 2, pp);
//    nurbs.Evaluate(pp[0], pp[1], 0, 3, p);
//    v.start.x = p[0];
//    v.start.y = p[1];
//    v.start.z = p[2];
//
//    curve.Evaluate(v.end.x, 0, 2, pp);
//    nurbs.Evaluate(pp[0], pp[1], 0, 3, p);
//    v.end.x = p[0];
//    v.end.y = p[1];
//    v.end.z = p[2];
//
//  }
//
//  if (cps) {
//    model.m_colorpoints.clear();
//    model.m_point_size = 3.0f;
//    for (int i = 0; i < curve.CVCount(); i++) {
//      ON_3dPoint p;
//      curve.GetCV(i, p);
//      double point[3];
//      nurbs.Evaluate(p.x, p.y, 0, 3, point);
//      TomGine::tgColorPoint cp;
//      cp.pos = TomGine::vec3(point[0], point[1], point[2]);
//      cp.color[0] = 0;
//      cp.color[1] = 255;
//      cp.color[2] = 0;
//      model.m_colorpoints.push_back(cp);
//    }
//  }
//}

void
Triangulation::DelauneyTriangulation (const pcl::on_nurbs::vector_vec2d &points, TomGine::tgModel &model,
                                      double max_concavity)
{
  // Delauney Triangulation
  std::vector<hed::Node*> nodes;
  // create boundary vertices in parameter space
  for (unsigned i = 0; i < points.size (); i++)
  {
    hed::Node* node = new hed::Node (points[i] (0), points[i] (1));
    nodes.push_back (node);
  }
  DelauneyTriangulation (nodes, model, max_concavity);
}

void
Triangulation::DelauneyTriangulation (const pcl::on_nurbs::vector_vec3d &points, TomGine::tgModel &model,
                                      double max_concavity)
{
  // Delauney Triangulation
  std::vector<hed::Node*> nodes;
  // create boundary vertices in parameter space
  for (unsigned i = 0; i < points.size (); i++)
  {
    hed::Node* node = new hed::Node (points[i] (0), points[i] (1), points[i] (2));
    nodes.push_back (node);
  }
  DelauneyTriangulation (nodes, model, max_concavity);
}

void
Triangulation::DelauneyTriangulation (std::vector<hed::Node*> &nodes, TomGine::tgModel &model, double max_concavity)
{
  hed::Triangulation triang;

  // Sort the nodes lexicographically in the plane.
  // This is recommended since the triangulation algorithm will run much faster.
  // (ltLexPoint is defined above)
  std::sort (nodes.begin (), nodes.end (), ltLexPoint);

  // Remove coincident points to avoid degenerate triangles. (eqPoints is defined above)
  std::vector<hed::Node*>::iterator new_end = std::unique (nodes.begin (), nodes.end (), eqPoints);

  // Make the triangulation
  triang.createDelaunay (nodes.begin (), new_end);

  // remove boundary triangles with too long edges (triangulated concave hull)
  triang.removeBoundaryTriangles (max_concavity);

  // copy triangulation to tgModel
  {
    std::list<hed::Node*>* nodelist = triang.getNodes ();
    list<hed::Edge*> leading_edges = triang.getLeadingEdges ();
    list<hed::Node*>::iterator nit;
    list<hed::Edge*>::iterator eit;
    map<int, unsigned> nodemap;
    model.Clear ();

    // copy Nodes to tgModel
    for (nit = nodelist->begin (); nit != nodelist->end (); nit++)
    {

      TomGine::tgVertex v;
      hed::Node* n = (*nit);
      v.pos = TomGine::vec3 (n->x (), n->y (), n->z ());

      model.m_vertices.push_back (v);
      nodemap[n->id ()] = (unsigned)(model.m_vertices.size () - 1);
    }

    // copy triangles to tgModel
    for (eit = leading_edges.begin (); eit != leading_edges.end (); eit++)
    {
      TomGine::tgFace f;
      hed::Edge* edge = (*eit);

      for (unsigned j = 0; j < 3; j++)
      {

        hed::Node* n = edge->getSourceNode ();
        f.v.push_back (nodemap[n->id ()]);

        edge = edge->getNextEdgeInFace ();
      }
      model.m_faces.push_back (f);
    }
  }
}

void
Evaluate (const ON_NurbsSurface &nurbs, TomGine::tgVertex &v)
{
  TomGine::vec3 tu, tv;
  TomGine::vec3 &pos = v.pos;
  TomGine::vec3 &n = v.normal;
  TomGine::vec2 &tc = v.texCoord;

  tc.x = pos.x;
  tc.y = pos.y;

  double pointAndTangents[9];
  nurbs.Evaluate (pos.x, pos.y, 1, 3, pointAndTangents);

  pos.x = pointAndTangents[0];
  pos.y = pointAndTangents[1];
  pos.z = pointAndTangents[2];

  tu.x = pointAndTangents[3];
  tu.y = pointAndTangents[4];
  tu.z = pointAndTangents[5];
  tv.x = pointAndTangents[6];
  tv.y = pointAndTangents[7];
  tv.z = pointAndTangents[8];

  n = TomGine::cross (tu, tv);
  n.normalize ();
}

void
Triangulation::TriangulatNurbs (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, TomGine::tgModel &model,
                                unsigned res, unsigned resBnd)
{
  // create param points for triangulation
  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
  double h = y1 - y0;

  TomGine::tgShapeCreator::CreatePlaneXY (model, x0, y0, 0.0, w, h, res, res);

  pcl::on_nurbs::vector_vec2d points;

  // interior points
  for (unsigned i = 0; i < model.m_vertices.size (); i++)
  {
    Eigen::Vector2d p = Eigen::Vector2d (model.m_vertices[i].pos.x, model.m_vertices[i].pos.y);
    double error;
    Eigen::Vector2d pt, t;
    double param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, p);
    pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, p, param, error, pt, t, 100, 1e-4, true);

    // evaluate if point lies inside or outside the closed curve
    Eigen::Vector3d a (p (0) - pt (0), p (1) - pt (1), 0.0);
    Eigen::Vector3d b (t (0), t (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    if (z (2) >= 0.0)
      points.push_back (p);
  }

  // boundary points
  std::vector<double> elements = pcl::on_nurbs::FittingCurve2dAPDM::getElementVector (curve);
  for (unsigned i = 0; i < elements.size () - 1; i++)
  {
    double &xi0 = elements[i];
    double &xi1 = elements[i + 1];

    double step = (xi1 - xi0) / resBnd;

    for (unsigned j = 0; j < resBnd; j++)
    {
      Eigen::Vector2d params;

      double pt[2];
      curve.Evaluate (xi0 + j * step, 0, 2, pt);
      params (0) = pt[0];
      params (1) = pt[1];

      points.push_back (params);
    }
  }

  TriangulatNurbs (nurbs, model, points, 1.0);
}

void
Triangulation::TriangulatNurbs (const ON_NurbsSurface &nurbs, TomGine::tgModel &model,
                                const pcl::on_nurbs::vector_vec2d &points, double max_concavity)
{
  // Delauney Triangulation
  std::vector<hed::Node*> nodes;
  // create boundary vertices in parameter space
  for (unsigned i = 0; i < points.size (); i++)
  {
    hed::Node* node = new hed::Node (points[i] (0), points[i] (1));
    nodes.push_back (node);
  }

  hed::Triangulation triang;

  // Sort the nodes lexicographically in the plane.
  // This is recommended since the triangulation algorithm will run much faster.
  // (ltLexPoint is defined above)
  std::sort (nodes.begin (), nodes.end (), ltLexPoint);

  // Remove coincident points to avoid degenerate triangles. (eqPoints is defined above)
  std::vector<hed::Node*>::iterator new_end = std::unique (nodes.begin (), nodes.end (), eqPoints);

  // Make the triangulation
  triang.createDelaunay (nodes.begin (), new_end);

  // project Nodes on NURBS and copy to tgModel
  std::list<hed::Node*>* nodelist = triang.getNodes ();
  list<hed::Node*>::iterator nit;
  map<int, unsigned> nodemap;
  model.Clear ();
  for (nit = nodelist->begin (); nit != nodelist->end (); nit++)
  {

    TomGine::tgVertex v;
    hed::Node* n = (*nit);
    v.pos = TomGine::vec3 (n->x (), n->y (), n->z ());

    Evaluate (nurbs, v);

    n->set (v.pos.x, v.pos.y, v.pos.z);

    model.m_vertices.push_back (v);
    nodemap[n->id ()] = (unsigned)(model.m_vertices.size () - 1);
  }

  // remove boundary triangles with too long edges (triangulated concave hull)
  triang.removeBoundaryTriangles (max_concavity);

  // copy triangles to tgModel
  list<hed::Edge*> leading_edges = triang.getLeadingEdges ();
  list<hed::Edge*>::iterator eit;
  for (eit = leading_edges.begin (); eit != leading_edges.end (); eit++)
  {
    TomGine::tgFace f;
    hed::Edge* edge = (*eit);

    for (unsigned j = 0; j < 3; j++)
    {

      hed::Node* n = edge->getSourceNode ();
      f.v.push_back (nodemap[n->id ()]);

      edge = edge->getNextEdgeInFace ();
    }
    model.m_faces.push_back (f);
  }

}

void
Triangulation::RemoveOccludedTriangles (TomGine::tgModel &model, cv::Mat1f &zBuffer, const TomGine::tgCamera &cam,
                                        float zOffset)
{
  TomGine::mat4 extrinsic = cam.GetExtrinsic ();
  TomGine::mat4 intrinsic = cam.GetIntrinsic ();
  std::vector<TomGine::tgFace> faces;

  for (unsigned i = 0; i < model.m_faces.size (); i++)
  {

    TomGine::tgFace &f = model.m_faces[i];

    // Get arithmetic mean of vertices of faces
    TomGine::vec3 u (0.0f, 0.0f, 0.0f);
    for (unsigned j = 0; j < f.v.size (); j++)
    {
      u = u + model.m_vertices[f.v[j]].pos;
    }
    u = u / f.v.size ();

    // Project mean point into camera view
    TomGine::vec4 p1 = extrinsic * TomGine::vec4 (u.x, u.y, u.z, 1.0);
    TomGine::vec4 p2 = intrinsic * p1;
    p2.x = p2.x / p2.w;
    p2.y = p2.y / p2.w;

    // Get x and y position in image space for depth buffer lookup
    int x = int ((p2.x + 1.0f) * 0.5f * zBuffer.cols);
    int y = int ((p2.y + 1.0f) * 0.5f * zBuffer.rows);

    // compare depth with z-buffer
    if (x > 0 && y > 0 && x < zBuffer.cols && y < zBuffer.rows)
    {
      if ((-p1.z) <= (zBuffer (y, x) + zOffset))
      {
        faces.push_back (f);
      }
    }
  }
  model.m_faces = faces;
}

//void Triangulation::Nurbs2TomGine(TomGine::tgTomGineThread *dbgWin, ON_NurbsSurface nurbs, int &surf_id, int res)
//{
//  if (dbgWin) {
//    TomGine::tgNurbsSurfacePatch surf;
//    Triangulation::Convert(nurbs, surf);
//    surf.resU = res;
//    surf.resV = res;
//
//    if (surf_id < 0)
//      surf_id = dbgWin->AddNurbsSurface(surf);
//    else
//      dbgWin->SetNurbsSurface(surf_id, surf);
//
//    dbgWin->Update();
//
//  }
//}

void
Triangulation::GL_GetProjectedNurbs (const ON_NurbsSurface &on_nurbs, cv::Mat_<uchar> &maskNurbs,
                                     TomGine::tgEngine *engine)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  maskNurbs.setTo (0);
  engine->Activate3D ();
  TomGine::tgModel surf;
  Triangulation::convertNurbsPatch2tgModel (on_nurbs, surf, 64);
  glColor3f (1.0, 1.0, 1.0);
  surf.DrawFaces ();
  glReadPixels (0, 0, maskNurbs.cols, maskNurbs.rows, GL_RED, GL_UNSIGNED_BYTE, &maskNurbs (0, 0));
}

void
Triangulation::ProjectPointsToPlane (const pcl::on_nurbs::vector_vec3d &cloud, const std::vector<float> &coeffs,
                                     pcl::on_nurbs::vector_vec3d &points)
{
  if (coeffs.size () != 4)
  {
    printf ("[Triangulation::ProjectPointsToPlane] coeffs.size: %lu\n", coeffs.size ());
    throw std::runtime_error ("[Triangulation::ProjectPointsToPlane] Wrong plane parameters!");
  }

  Eigen::Vector4d mc (coeffs[0], coeffs[1], coeffs[2], 0);
  Eigen::Vector4d p (0, 0, 0, 1), pp;
  float dist;

  mc.normalize ();

  Eigen::Vector4d tmp_mc = mc;
  tmp_mc[3] = coeffs[3];

  points.resize (cloud.size ());
  for (unsigned i = 0; i < cloud.size (); i++)
  {
    const Eigen::Vector3d &pt = cloud[i];

    p (0) = pt (0);
    p (1) = pt (1);
    p (2) = pt (2);

    dist = tmp_mc.dot (p);

    pp = p - mc * dist;

    Eigen::Vector3d &ptOut = points[i];
    ptOut[0] = pp[0];
    ptOut[1] = pp[1];
    ptOut[2] = pp[2];
  }
}

