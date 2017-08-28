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

#ifndef _OBJECT_MODELING_TRIANGULATION_H_
#define _OBJECT_MODELING_TRIANGULATION_H_

#include <vector>
//#include <pcl/surface/on_nurbs/nurbs_data.h>
//#include <pcl/surface/3rdparty/opennurbs/opennurbs.h>

#include "v4r/on_nurbs/nurbs_data.h"
#include "v4r/opennurbs/opennurbs.h"


#include "v4r/TomGine/tgTomGineThread.h"

#include "v4rexternal/TTL-1.1.0/HeTriang.h"
#include "v4rexternal/TTL-1.1.0/HeDart.h"
#include "v4rexternal/TTL-1.1.0/HeTraits.h"

namespace objectmodeling
{

  class Triangulation
  {
  public:

  public:
    static void
    reverse (ON_NurbsCurve &curve, bool z_negative = true);

    static void
    flip (int dir, ON_NurbsSurface &nurbs);

    static Eigen::Vector2d
    intersectNurbsLine (const ON_NurbsSurface &nurbs, Eigen::Vector3d p0, Eigen::Vector3d dir, double hint,
                        unsigned steps, double accuracy);

    static Eigen::Vector3d
    intersectPlaneLine (const std::vector<float> &plane, Eigen::Vector3d p0, Eigen::Vector3d dir);

    static bool
    isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance, bool z_negative = true);

    static bool
    isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance, double rScale,
                   bool z_negative = true);

    static void
    getPlaneBasis (const std::vector<float> &plane, Eigen::Matrix3d &R, Eigen::Vector3d &t);

    static void
    computeBoundingBox (const pcl::on_nurbs::vector_vec2d &points, Eigen::Vector2d &_min, Eigen::Vector2d &_max);

    static void
    computeBoundingBox (const pcl::on_nurbs::vector_vec3d &points, Eigen::Vector3d &_min, Eigen::Vector3d &_max);

    /* @brief Removes vertices that are not indexed by any face */
    static void
    removeUnusedVertices (TomGine::tgModel &model);

    static void
    convertNurbs2Plane (const ON_NurbsSurface &surf, std::vector<float> &plane);

    /** @brief Converts clamped NURBS from openNURBS to TomGine::tgModel. */
    static void
    convertNurbs2tgModel (const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned resU = 16,
                          unsigned resV = 16, bool cps = false);

    static void
    convertNurbsPatch2tgModel (const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned resU = 16,
                               unsigned resV = 16, bool cps = false);

    //    static void
    //    Convert (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, TomGine::tgModel &model, unsigned res, bool cps);

    static void
    convertTrimmedSurface2tgModel (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, TomGine::tgModel &model,
                                   unsigned resU, unsigned resV, bool cps);

    static void
    convertBrep2tgModel (const ON_Brep &brep, TomGine::tgModel &model, unsigned resF = 16, bool cps = false);

    /** @brief Converts peridoc NURBS from openNURBS to TomGine::tgModel. */
    static void
    convertNurbsCylinder2tgModel (const ON_NurbsSurface &on_surf, TomGine::tgModel &model, unsigned resU = 16,
                                  unsigned resV = 16, bool cps = false);

    static void
    Convert (const ON_NurbsCurve &nurbs, TomGine::tgModel &model, unsigned res = 16, bool cps = false);

    static void
    Convert (const ON_NurbsCurve &curve, const ON_NurbsSurface &nurbs, TomGine::tgModel &model, unsigned res, bool cps);

    static void
    convertPlane2tgModel (const std::vector<float> &plane, TomGine::tgModel &model, const Eigen::Vector4d &bb,
                          unsigned res);

    static void
    convertPlane2tgModel (const pcl::on_nurbs::vector_vec3d &points, TomGine::tgModel &model, unsigned res = 4);

    static void
    convertCurveOnPlane2tgModel (const ON_NurbsCurve &curve, const std::vector<float> &plane, TomGine::tgModel &model,
                                 unsigned res, bool cps = false);

    static void
    convertTrimmedPlane2tgModel (const std::vector<float> &plane, const ON_NurbsCurve &curve, TomGine::tgModel &model,
                                 unsigned res, bool cps);

    static void
    DelauneyTriangulation (const pcl::on_nurbs::vector_vec2d &points, TomGine::tgModel &model, double max_concavity);

    static void
    DelauneyTriangulation (const pcl::on_nurbs::vector_vec3d &points, TomGine::tgModel &model, double max_concavity);

    static void
    DelauneyTriangulation (std::vector<hed::Node*> &nodes, TomGine::tgModel &model, double max_concavity);

    static void
    TriangulatNurbs (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, TomGine::tgModel &model, unsigned res,
                     unsigned resBnd);

    /** @brief Triangulates a NURBS surface and calculates the concave hull */
    static void
    TriangulatNurbs (const ON_NurbsSurface &on_surf, TomGine::tgModel &model,
                     const pcl::on_nurbs::vector_vec2d &points, double max_concavity);

    /** @brief Remove occluded triangles by projecting the center of the triangle into the image and
     * compare the depth to a depth buffer */
    static void
    RemoveOccludedTriangles (TomGine::tgModel &model, cv::Mat1f &zBuffer, const TomGine::tgCamera &cam,
                             float zOffset = 0.02);

    /** @brief Display a NURBS in a TomGine::tgTomGineThread */
    //  static void Nurbs2TomGine(TomGine::tgTomGineThread *dbgWin, ON_NurbsSurface on_surf, int &surf_id, int res);

    /** @brief Project a NURBS into the window 'engine' and creates a pixel mask */
    static void
    GL_GetProjectedNurbs (const ON_NurbsSurface &on_nurbs, cv::Mat_<uchar> &maskNurbs, TomGine::tgEngine *engine);

    static void
    ProjectPointsToPlane (const pcl::on_nurbs::vector_vec3d &cloud, const std::vector<float> &coeffs,
                          pcl::on_nurbs::vector_vec3d &points);

  private:

  };

}

#endif // _OBJECT_MODELING_TRIANGULATION_H_
