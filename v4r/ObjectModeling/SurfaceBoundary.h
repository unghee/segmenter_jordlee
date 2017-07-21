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

#ifndef _SURFACE_BOUNDARY_H_
#define _SURFACE_BOUNDARY_H_

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include "v4r/TomGine/tgTomGineThread.h"

#include <opencv2/core/core.hpp>

namespace objectmodeling {

class SurfaceBoundary
{
public:
  static void convertIndexMap2ColorMap(cv::Mat_<uchar> &index_map, cv::Mat_<cv::Vec3b> &color_map);
  static void createIndexMap(surface::View &view, cv::Mat_<uchar> &index_map);

  static void createNurbs(surface::View &view, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
      std::vector<pcl::on_nurbs::NurbsDataSurface*> &data);

  static void createCommonBoundaries(surface::View &view,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat_<uchar> &index_map,
      std::vector<pcl::on_nurbs::NurbsDataSurface*> &data);

  static void createSurfaceBoundaries(surface::View &view,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat_<uchar> &index_map,
      std::vector<pcl::on_nurbs::NurbsDataSurface*> &data);

  static void createObjectBoundaries2d(cv::Mat_<uchar> &index_map,
      std::vector<pcl::on_nurbs::NurbsDataCurve2d> &boundaries, bool use_asymetric_weight);

  struct FitBoundaryParameter
  {
    pcl::on_nurbs::FittingCurve2dAPDM::Parameter paramFC;
    pcl::on_nurbs::FittingCurve2dAPDM::Parameter paramFC_TDM;
    pcl::on_nurbs::FittingCurve2dAPDM::Parameter paramFC_SDM;
    int order;
    unsigned refinement;
    double addCPsAccuracy;
    unsigned addCPsIteration;
    unsigned maxCPs;

    double fitAvgError;
    double fitMaxError;
    unsigned fitMaxSteps;

    double invMapAccuracy;
    unsigned invMapMaxSteps;

    double wCommon;
    double wInterior;
  };

  static void createNurbsBoundaries(TomGine::tgTomGineThread &dbgWin, surface::View &view,
      std::vector<pcl::on_nurbs::NurbsDataSurface*> &data, std::vector<ON_NurbsCurve> &nurbs,
      FitBoundaryParameter &param);

  static void fitBoundaryNurbs(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
      pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param);

  static void fitBoundaryNurbs_SDM(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
      pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param);

  static void fitBoundaryNurbs_TDM(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
      pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param);

};

}

#endif
