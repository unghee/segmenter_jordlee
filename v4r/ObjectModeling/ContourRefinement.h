/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
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
 *   * Neither the name of Thomas Mörwald nor the names of its
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

#ifndef _CONTOUR_REFINEMENT_H_
#define _CONTOUR_REFINEMENT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/TomGinePCL/tgTomGineThreadPCL.h"

namespace objectmodeling
{

  class ContourRefinement
  {
  public:
    struct Parameter
    {
      int kernel;
      int curve_order;
      int contour_steps;
      int canny_threshold1;
      int canny_threshold2;
      int canny_apertureSize;
      pcl::on_nurbs::FittingCurve2dAPDM::FitParameter m_curve_params;
      unsigned curve_iterations;
      unsigned surface_iterations;
      int surface_order;
      unsigned surface_refinement;
    };

  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
    surface::View *m_view;
    std::vector<size_t> m_surface_indices;
    ContourRefinement::Parameter m_params;

    TomGine::tgTomGineThreadPCL *m_viewer;

    cv::Mat_<cv::Vec3b> m_image;
    cv::Mat_<uchar> m_canny;
    cv::Mat_<int> m_labels;
    cv::Mat_<double> m_zbuffer;
    Eigen::Vector4i m_bb;
    std::vector<pcl::on_nurbs::NurbsDataCurve2d> m_data;
    std::vector<ON_NurbsCurve> m_curves3D;

    bool m_cloud_set;
    bool m_curves_computed;
    bool m_surfaces_computed;

    bool m_quiet;

    static bool
    isValid (pcl::PointXYZRGB &p);

    /** compute bounding box of cloud in index space */
    Eigen::Vector4i
    computeIndexBoundingBox ();

    /** search for edges along contour and assemble data accordingly */
    void
    assembleData (pcl::on_nurbs::NurbsDataCurve2d &data, const std::vector<int> &contour, const Eigen::Vector4i &bb,
                  int kernel);

    void
    updateContour (const pcl::on_nurbs::NurbsDataCurve2d &data, const ON_NurbsCurve &curve, std::vector<int> &contour);

    void
    getNeighborLabels (const int &px, const int &py, const int &label, std::vector<int> &neighbors, int kernel) const;

    void
    getNeighborPixels (const int &px, const int &py, const int &label, pcl::on_nurbs::vector_vec2i &neighbors,
                       int kernel) const;
    void
    createLabelMatrix ();

    void
    projectPoint (pcl::PointXYZRGB &pc, const ON_NurbsSurface &nurbs);

    void
    projectPoint (pcl::PointXYZRGB &pc, const std::vector<float> &plane);

    bool
    reasignPoint (size_t px, size_t py, pcl::PointXYZRGB &pc, int &label, Eigen::Vector3d &normal,
                  double zthreshold = 0.02, size_t kernel = 40) const;

  public:
    /** Get parameter for contour refinement */
    inline ContourRefinement::Parameter&
    getParams ()
    {
      return m_params;
    }

    inline std::vector<ON_NurbsCurve>&
    getCurves3D ()
    {
      return m_curves3D;
    }

    inline double&
    getScale ()
    {
      return m_params.m_curve_params.param.rScale;
    }

    inline cv::Mat_<cv::Vec3b>&
    getColorImage ()
    {
      return m_image;
    }

    inline cv::Mat_<uchar>&
    getEdgeImage ()
    {
      return m_canny;
    }

    inline cv::Mat_<int>&
    getLabelImage ()
    {
      return m_labels;
    }

    inline void
    setQuiet (bool quiet)
    {
      m_quiet = quiet;
    }

  public:
    ContourRefinement ();

    pcl::PointXYZ
    projectPoint (int x, int y, const ON_NurbsSurface &nurbs, const Eigen::Matrix3d &intrinsic);

    pcl::PointXYZ
    projectPoint (int x, int y, const std::vector<float> &plane, const Eigen::Matrix3d &intrinsic);

    static Eigen::Vector4d
    computeCPBoundingBox (const ON_NurbsCurve &curve);

    /** initialize control points of curve by the mean of 'steps' sequential points */
    static ON_NurbsCurve
    initNurbsCurve (int order, size_t steps, const pcl::on_nurbs::NurbsDataCurve2d &data);

    /** Set input point cloud **/
    void
    setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, surface::View &view);

    void
    setSurfaceIndices (const std::vector<size_t> &indices);

    /** Set visualization tool */
    void
    setViewer (TomGine::tgTomGineThreadPCL &_viewer);

    /** Set parameter for contour refinement */
    void
    setParams (ContourRefinement::Parameter &_params);

    //    static void
    //    computeCurveImage (surface::SurfaceModel::Ptr surface, int curve_order, int contour_steps,
    //                       pcl::on_nurbs::FittingCurve2dPDM::Parameter params, unsigned cloud_width);
    //
    void
    computeCurveImage (surface::SurfaceModel::Ptr surface);

    /** Compute accurate contour curves in image space */
    void
    computeCurvesImage (size_t min_points = 3);

    /** Compute accurate contour curves in image space aligned to the edges of the color image */
    void
    computeCurvesImageEdgeAligned (size_t min_points = 10, bool update_contour = false);

    static ON_NurbsSurface
    computeSurfaceFromPlane (std::vector<float> &plane, Eigen::Matrix3d intrinsic, Eigen::Vector4d bb);

    void
    computeSurfaces ();

    /** Transform curves from image space to parameter space */
    void
    computeCurvesParam ();

    void
    trimSurfacePoints2 (double tolerance);

    void
    trimSurfacePoints (double tolerance = 1.0, bool reasign = false, bool project = false);

    void
    reasignNAN (double zthreshold = 0.02, size_t kernel = 40, bool project = false);

    bool
    isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance = 1.0) const;

    void
    visualizeCurve2D (ON_NurbsCurve curve, double height);

    void
    visualizeCurve3D (ON_NurbsCurve curve, double height, double scale);

  };
}

#endif
