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

#include "ContourRefinement.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/ObjectModeling/Triangulation.h"
#include <pcl/surface/on_nurbs/fitting_surface_im.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>

using namespace objectmodeling;

bool
ContourRefinement::isValid (pcl::PointXYZRGB &p)
{
  if (isnan (p.x) || isnan (p.y) || isnan (p.z))
    return false;

  if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0)
    return false;

  return true;
}

Eigen::Vector4i
ContourRefinement::computeIndexBoundingBox ()
{
  Eigen::Vector4i bb (INT_MAX, 0, INT_MAX, 0);
  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;

  for (int i = 0; i < (int)cloud_ref.width; i++)
  {
    for (int j = 0; j < (int)cloud_ref.height; j++)
    {
      pcl::PointXYZRGB &point = cloud_ref (i, j);
      if (!isValid (point))
        continue;

      if (i < bb (0))
        bb (0) = i;
      if (i > bb (1))
        bb (1) = i;
      if (j < bb (2))
        bb (2) = j;
      if (j > bb (3))
        bb (3) = j;
    }
  }
  return bb;
}

void
ContourRefinement::assembleData (pcl::on_nurbs::NurbsDataCurve2d &data, const std::vector<int> &contour,
                                 const Eigen::Vector4i &bb, int kernel)
{
  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;
  std::map<int, bool> mymap;
  for (size_t j = 0; j < contour.size (); j++)
  {
    int px = contour[j] % cloud_ref.width;
    int py = contour[j] / cloud_ref.width;

    double min_dist (DBL_MAX);
    Eigen::Vector2i max_pt;
    for (int u = -kernel; u <= kernel; u++)
    {
      for (int v = -kernel; v <= kernel; v++)
      {
        int idx = px + u;
        int idy = py + v;

        if (idx < bb (0) || idx > bb (1) || idy < bb (2) || idy > bb (3))
          continue;

        uchar &w = m_canny (idy, idx);
        double d = (u * u + v * v);
        if (w > 128 && d < min_dist)
        {
          min_dist = d;
          max_pt = Eigen::Vector2i (idx, idy);
        }
      }
    }

    if (min_dist < DBL_MAX)
    {
      int id = max_pt (1) * m_cloud->width + max_pt (0);
      if (mymap.count (id) > 0)
        continue;
      mymap[id] = true;
      data.interior.push_back (Eigen::Vector2d (max_pt (0), max_pt (1)));
    }
    else
    {
      int id = py * m_cloud->width + px;
      if (mymap.count (id) > 0)
        continue;
      mymap[id] = true;
      data.interior.push_back (Eigen::Vector2d (px, py));
    }

  }

}

ON_NurbsCurve
ContourRefinement::initNurbsCurve (int order, size_t steps, const pcl::on_nurbs::NurbsDataCurve2d &data)
{
  if (data.interior.size () < size_t (2 * order) || steps <= 0)
    return pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, data.interior);

  pcl::on_nurbs::vector_vec2d cps;

  for (size_t i = 0; i < data.interior.size (); i += steps)
  {
    pcl::on_nurbs::vector_vec2d tmp;
    for (size_t j = 0; j < steps; j++)
    {
      size_t idx = i + j;
      if (idx >= data.interior.size ())
        break;

      tmp.push_back (data.interior[idx]);
    }

    Eigen::Vector2d u = pcl::on_nurbs::NurbsTools::computeMean (tmp);
    cps.push_back (u);

  }

  ON_NurbsCurve curve;
  while (cps.size () < size_t (2 * order))
    return initNurbsCurve (order, --steps, data);

  return pcl::on_nurbs::FittingCurve2dAPDM::initCPsNurbsCurve2D (order, cps);
}

Eigen::Vector4d
ContourRefinement::computeCPBoundingBox (const ON_NurbsCurve &curve)
{
  Eigen::Vector4d bb (DBL_MAX, 0.0, DBL_MAX, 0.0);

  for (size_t i = 0; i < size_t (curve.CVCount ()); i++)
  {
    ON_3dPoint p;
    curve.GetCV (i, p);

    if (p.x < bb (0))
      bb (0) = p.x;
    if (p.x > bb (1))
      bb (1) = p.x;
    if (p.y < bb (2))
      bb (2) = p.y;
    if (p.y > bb (3))
      bb (3) = p.y;
  }

  return bb;
}

void
ContourRefinement::updateContour (const pcl::on_nurbs::NurbsDataCurve2d &data, const ON_NurbsCurve &curve,
                                  std::vector<int> &contour)
{
  contour.clear ();
  double point[2];
  for (size_t i = 0; i < data.interior_param.size (); i++)
  {
    curve.Evaluate (data.interior_param[i], 0, 2, point);

    int px = int (point[0] + 0.5);
    int py = int (point[1] + 0.5);

    if (px < 0)
      px = 0;
    if (px >= (int)m_cloud->width)
      px = m_cloud->width - 1;
    if (py < 0)
      py = 0;
    if (py >= (int)m_cloud->height)
      py = m_cloud->height - 1;

    contour.push_back (py * m_cloud->width + px);
  }
}

void
ContourRefinement::getNeighborLabels (const int &px, const int &py, const int &label, std::vector<int> &neighbors,
                                      int kernel) const
{
  for (int u = -kernel; u <= kernel; u++)
  {
    for (int v = -kernel; v <= kernel; v++)
    {
      int idx = px + u;
      int idy = py + v;

      if (idx < 0 || idx >= (int)m_cloud->width || idy < 0 || idy >= (int)m_cloud->height)
        continue;

      int label2 = m_labels (idy, idx);
      if (label2 >= 0)
        if (label2 != label)
          if (find (neighbors.begin (), neighbors.end (), label2) == neighbors.end ())
            neighbors.push_back (label2);
    }
  }
}

void
ContourRefinement::getNeighborPixels (const int &px, const int &py, const int &label,
                                      pcl::on_nurbs::vector_vec2i &neighbors, int kernel) const
{
  for (int u = -kernel; u <= kernel; u++)
  {
    for (int v = -kernel; v <= kernel; v++)
    {
      int idx = px + u;
      int idy = py + v;

      if (idx < 0 || idx >= (int)m_cloud->width || idy < 0 || idy >= (int)m_cloud->height)
        continue;

      int label2 = m_labels (idy, idx);
      if (label2 >= 0)
        if (label2 != label)
          neighbors.push_back (Eigen::Vector2i (idx, idy));
    }
  }
}

void
ContourRefinement::createLabelMatrix ()
{
  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;
  m_labels = cv::Mat_<int> (cloud_ref.height, cloud_ref.width);
  m_labels.setTo (-1);
  for (size_t i = 0; i < m_view->surfaces.size (); i++)
  {
    surface::SurfaceModel::Ptr surf = m_view->surfaces[i];
    for (size_t j = 0; j < surf->indices.size (); j++)
    {
      int px = surf->indices[j] % cloud_ref.width;
      int py = surf->indices[j] / cloud_ref.width;

      if (px < 0 || px >= (int)cloud_ref.width || py < 0 || py >= (int)cloud_ref.height)
      {
        printf ("indices: %d %d\n", px, py);
        throw std::runtime_error ("[ContourRefinement::createLabelMatrix] index out of bounds\n");
      }

      m_labels (py, px) = i;
    }
  }
}

void
ContourRefinement::projectPoint (pcl::PointXYZRGB &pc, const ON_NurbsSurface &nurbs)
{
  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d p1 (pc.x, pc.y, pc.z);
  Eigen::Vector3d dir = p1 - p0;
  double hint = dir.norm ();
  dir = dir / hint;

  Eigen::Vector2d param = Triangulation::intersectNurbsLine (nurbs, p0, dir, hint, 20, 1e-4);
  double point[3];
  nurbs.Evaluate (param (0), param (1), 0, 3, point);
  pc.x = point[0];
  pc.y = point[1];
  pc.z = point[2];
}

void
ContourRefinement::projectPoint (pcl::PointXYZRGB &pc, const std::vector<float> &plane)
{
  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d p1 (pc.x, pc.y, pc.z);
  Eigen::Vector3d dir = p1 - p0;
  dir.normalize ();

  Eigen::Vector3d p1i = Triangulation::intersectPlaneLine (plane, p0, dir);
  pc.x = p1i (0);
  pc.y = p1i (1);
  pc.z = p1i (2);
}

pcl::PointXYZ
ContourRefinement::projectPoint (int x, int y, const ON_NurbsSurface &nurbs, const Eigen::Matrix3d &intrinsic)
{
  pcl::PointXYZ pc;

  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d p1;
  p1 (0) = 1.0 * (x - intrinsic (0, 2)) / intrinsic (0, 0);
  p1 (1) = 1.0 * (y - intrinsic (1, 2)) / intrinsic (1, 1);
  p1 (2) = 1.0;
  Eigen::Vector3d dir = p1 - p0;
  dir.normalize ();

  Eigen::Vector2d param = Triangulation::intersectNurbsLine (nurbs, p0, dir, 1.0, 20, 1e-4);
  double point[3];
  nurbs.Evaluate (param (0), param (1), 0, 3, point);
  pc.x = point[2] * (x - intrinsic (0, 2)) / intrinsic (0, 0);
  pc.y = point[2] * (y - intrinsic (1, 2)) / intrinsic (1, 1);
  pc.z = point[2];

  return pc;
}

pcl::PointXYZ
ContourRefinement::projectPoint (int x, int y, const std::vector<float> &plane, const Eigen::Matrix3d &intrinsic)
{
  pcl::PointXYZ pc;

  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d p1;
  p1 (0) = 1.0 * (x - intrinsic (0, 2)) / intrinsic (0, 0);
  p1 (1) = 1.0 * (y - intrinsic (1, 2)) / intrinsic (1, 1);
  p1 (2) = 1.0;
  Eigen::Vector3d dir = p1 - p0;
  dir.normalize ();

  Eigen::Vector3d p1i = Triangulation::intersectPlaneLine (plane, p0, dir);
  pc.x = p1i (2) * (x - intrinsic (0, 2)) / intrinsic (0, 0);
  pc.y = p1i (2) * (y - intrinsic (1, 2)) / intrinsic (1, 1);
  pc.z = p1i (2);

  return pc;
}

bool
ContourRefinement::reasignPoint (size_t px, size_t py, pcl::PointXYZRGB &pc, int &label, Eigen::Vector3d &normal,
                                 double zthreshold, size_t kernel) const
{
  bool left (false);
  bool right (false);
  int idx_left (0);
  int idx_right (0);
  int label_left (-1);
  int label_right (-1);
  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;
  pcl::PointXYZRGB pc_left;
  pcl::PointXYZRGB pc_right;

  // search for valid points in x- direction
  for (size_t j = 1; j < kernel; j++)
  {
    idx_left = px - j;

    if (idx_left < m_bb (0))
      break;

    label_left = m_labels (py, idx_left);
    pc_left = cloud_ref.at (idx_left, py);
    if (label_left >= 0 && isValid (pc_left))
    {
      left = true;
      break;
    }
  }

  // search for valid points in x+ direction
  for (size_t j = 1; j < kernel; j++)
  {
    idx_right = px + j;

    if (idx_right > m_bb (1))
      break;

    label_right = m_labels (py, idx_right);
    pc_right = cloud_ref.at (idx_right, py);
    if (label_right >= 0 && isValid (pc_right))
    {
      right = true;
      break;
    }
  }

  // if valid points on left and right side found
  if (left && right)
  {

    double dist = pc_left.z - pc_right.z;
    if (label_right != label_left && std::abs<double> (dist) < zthreshold)
      return false;

    // choose surface with highest z value
    // nan occur at far surface usually (for Kinect)
    if (dist > 0.0)
      label = label_left;
    else
      label = label_right;

    if (label >= (int)m_view->surfaces.size ())
    {
      printf ("[ContourRefinement::reasignPoint] surface[%i] Warning: not a surface\n", label);
      return false;
    }

    surface::SurfaceModel::Ptr surf = m_view->surfaces[label];

    // project and intersect point with surface
    Eigen::Vector3d p0 (0.0, 0.0, 0.0);
    Eigen::Vector3d p1;
    p1 (2) = 1.0;
    p1 (0) = p1 (2) * (px - m_view->intrinsic (0, 2)) / m_view->intrinsic (0, 0);
    p1 (1) = p1 (2) * (py - m_view->intrinsic (1, 2)) / m_view->intrinsic (1, 1);
    Eigen::Vector3d dir = p1 - p0;

    Eigen::Vector3d p2;
    if (surf->type == pcl::SACMODEL_PLANE && surf->coeffs.size () == 4)
    {
      p2 = Triangulation::intersectPlaneLine (surf->coeffs, p0, dir);
      normal (0) = surf->coeffs[0];
      normal (1) = surf->coeffs[1];
      normal (2) = surf->coeffs[2];
    }
    else if (surf->type == MODEL_NURBS && surf->nurbs.CVCount (0) > 0)
    {
      Eigen::Vector2d params = Triangulation::intersectNurbsLine (surf->nurbs, p0, dir, 1.0, 100, 1e-4);
      double pnt[9];
      surf->nurbs.Evaluate (params (0), params (1), 1, 3, pnt);
      p2 = Eigen::Vector3d (pnt[0], pnt[1], pnt[2]);
      Eigen::Vector3d tu (pnt[3], pnt[4], pnt[5]);
      Eigen::Vector3d tv (pnt[6], pnt[7], pnt[8]);
      tu.normalize ();
      tv.normalize ();
      normal = -tu.cross (tv);
      if (normal.dot (Eigen::Vector3d (0.0, 0.0, 1.0)) > 0.0)
        printf ("[ContourRefinement::reasignPoint] Warning, surface normal pointing in z-direction.\n");
    }
    else
    {
      //      printf ("[ContourRefinement::reasignPoint] surface[%i] Warning: surface model not found\n", label);
      return false;
    }

    pc.x = p2 (2) * (px - m_view->intrinsic (0, 2)) / m_view->intrinsic (0, 0);
    pc.y = p2 (2) * (py - m_view->intrinsic (1, 2)) / m_view->intrinsic (1, 1);
    pc.z = p2 (2);
    return true;
  }

  return false;
}

ContourRefinement::ContourRefinement () :
  m_cloud_set (false), m_curves_computed (false), m_surfaces_computed (false), m_quiet (true)
{
  m_viewer = NULL;

  m_params.kernel = 5;
  m_params.curve_order = 3;
  m_params.contour_steps = 20;
  m_params.canny_threshold1 = 50;
  m_params.canny_threshold2 = 100;
  m_params.canny_apertureSize = 3;

  m_params.m_curve_params.addCPsAccuracy = 1.0;
  m_params.m_curve_params.addCPsIteration = 100;
  m_params.m_curve_params.maxCPs = 100;
  m_params.m_curve_params.accuracy = 1e-4;
  m_params.m_curve_params.iterations = 20;

  m_params.m_curve_params.param.closest_point_resolution = 0;
  m_params.m_curve_params.param.closest_point_weight = 0.0;
  m_params.m_curve_params.param.closest_point_sigma2 = 0.0;
  m_params.m_curve_params.param.interior_sigma2 = 0.0;
  m_params.m_curve_params.param.smooth_concavity = 0.0;
  m_params.m_curve_params.param.smoothness = 0.01;

  m_params.m_curve_params.param.rScale = 640.0;

  m_params.curve_iterations = 3;

  m_params.surface_iterations = 3;
  m_params.surface_order = 3;
  m_params.surface_refinement = 1;
}

void
ContourRefinement::setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, surface::View &view)
{
  m_cloud = _cloud;
  m_view = &view;
  m_cloud_set = true;
  m_curves_computed = false;
  m_surfaces_computed = false;

  m_surface_indices.clear ();
  for (size_t i = 0; i < view.surfaces.size (); i++)
    m_surface_indices.push_back (i);
}

void
ContourRefinement::setSurfaceIndices (const std::vector<size_t> &indices)
{
  m_surface_indices.clear ();
  for (size_t i = 0; i < indices.size (); i++)
  {
    const size_t &idx = indices[i];
    if (idx < 0 || idx >= m_view->surfaces.size ())
    {
      printf ("[ContourRefinement::setSurfaceIndices] Warning, index out of bounds (%lu)\n", idx);
      continue;
    }
    m_surface_indices.push_back (idx);
  }

}

void
ContourRefinement::setViewer (TomGine::tgTomGineThreadPCL &_viewer)
{
  m_viewer = &_viewer;
}

void
ContourRefinement::setParams (ContourRefinement::Parameter &_params)
{
  m_params = _params;
}

void
ContourRefinement::computeCurveImage (surface::SurfaceModel::Ptr surface)
{
  if (!m_cloud_set)
  {
    std::cout << "[ContourRefinement::computeBoundaries] Error, no input cloud set" << endl;
    return;
  }

  pcl::on_nurbs::NurbsDataCurve2d data;

  // create curve data from contour indices
  for (size_t i = 0; i < surface->contours[0].size (); i++)
  {
    const int &c = surface->contours[0][i];

    int px = c % m_cloud->width;
    int py = c / m_cloud->width;

    data.interior.push_back (Eigen::Vector2d (px, py));
  }

  ON_NurbsCurve curve = initNurbsCurve (m_params.curve_order, m_params.contour_steps, data);

  pcl::on_nurbs::FittingCurve2dPDM::Parameter params;
  params.smoothness = m_params.m_curve_params.param.smoothness;
  params.rScale = 1.0 / std::max (m_cloud->width, m_cloud->height);

  pcl::on_nurbs::FittingCurve2dPDM fit (&data, curve);
  for (unsigned i = 0; i < m_params.curve_iterations; i++)
  {
    fit.assemble (params);
    fit.solve ();
  }

  curve = fit.m_nurbs;
  Triangulation::reverse (curve, false);
  surface->curves_image.assign (1, curve);
}

void
ContourRefinement::computeCurvesImage (size_t min_points)
{
  if (!m_cloud_set)
  {
    std::cout << "[ContourRefinement::computeBoundaries] Error, no input cloud set" << endl;
    return;
  }

  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = m_view->surfaces[m_surface_indices[i]];

    if(surface->contours.empty())
    {
      printf("[ContourRefinement::computeBoundaries] surface[%lu] Warning, no contours given\n", m_surface_indices[i]);
      continue;
    }

    // project contour into image space and search for edges along contour
    if (surface->contours[0].size () < min_points)
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeBoundaries] TOO SMALL, skipping (limit = %lu)\n", min_points);
      continue;
    }

    computeCurveImage (surface);
  }

  m_curves_computed = true;
}

void
ContourRefinement::computeCurvesImageEdgeAligned (size_t min_points, bool update_contour)
{
  if (!m_cloud_set)
  {
    std::cout << "[ContourRefinement::computeBoundaries] Error, no input cloud set" << endl;
    return;
  }

  // edge detection
  pclA::ConvertPCLCloud2Image (m_cloud, m_image);
  cv::Canny (m_image, m_canny, m_params.canny_threshold1, m_params.canny_threshold2, m_params.canny_apertureSize, false);

  // compute index bounding box of cloud
  m_bb = computeIndexBoundingBox ();
  if (!m_quiet)
    printf ("[ContourRefinement::computeBoundaries] bb: %d %d %d %d\n", m_bb (0), m_bb (1), m_bb (2), m_bb (3));

  // define scale for fitting data
  m_params.m_curve_params.param.rScale = 1.0 / std::max (m_cloud->width, m_cloud->height);
  if (!m_quiet)
    printf ("[ContourRefinement::computeBoundaries] scale: %f\n", m_params.m_curve_params.param.rScale);

  // iterate through segments to be fitted (surfaces)
  m_data.assign (m_surface_indices.size (), pcl::on_nurbs::NurbsDataCurve2d ());

  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    // contour detection
    //    pclA::ContourDetection contour_detection;
    //    contour_detection.setInputCloud (m_cloud);
    //    contour_detection.compute (m_view->surfaces[i]->indices, m_contour);
    surface::SurfaceModel::Ptr surface = m_view->surfaces[m_surface_indices[i]];

    // project contour into image space and search for edges along contour
    pcl::on_nurbs::NurbsDataCurve2d &data = m_data[i];
    assembleData (data, surface->contours[0], m_bb, m_params.kernel);
    if (!m_quiet)
    {
      printf ("[%lu] boundary size: %lu | ", i, data.interior.size ());
      printf ("contour size: %lu | ", surface->contours[0].size());
    }

    if (data.interior.size () < min_points)
    {
      if (!m_quiet)
        printf ("TOO SMALL, skipping (limit = %lu)\n", min_points);
      continue;
    }

    // initialize curve
    surface->curves_image.assign (1, initNurbsCurve (m_params.curve_order, m_params.contour_steps, data));
    ON_NurbsCurve &curve = surface->curves_image[0];

    // fit curve;
    data.interior_weight_function.push_back (false);
    pcl::on_nurbs::FittingCurve2dAPDM curve_fit (&data, curve);

    curve_fit.fitting (m_params.m_curve_params);
    curve = curve_fit.m_nurbs;
    if (!m_quiet)
      printf ("control points: %d\n", curve.CVCount ());

    Triangulation::reverse (curve, false);

    if (update_contour)
      updateContour (data, curve, surface->contours[0]);
  }

  m_curves_computed = true;
}

ON_NurbsSurface
ContourRefinement::computeSurfaceFromPlane (std::vector<float> &plane, Eigen::Matrix3d intrinsic, Eigen::Vector4d bb)
{
  ON_NurbsSurface nurbs (3, false, 2, 2, 2, 2);
  double dx = bb (1) - bb (0);
  double dy = bb (3) - bb (2);
  nurbs.MakeClampedUniformKnotVector (0, dx);
  nurbs.MakeClampedUniformKnotVector (1, dy);

  Eigen::Vector3d p0 (0.0, 0.0, 0.0);
  Eigen::Vector3d pt, p1;
  Eigen::Vector3d dir;

  nurbs.SetKnot (0, 0, bb (0));
  nurbs.SetKnot (0, 1, bb (1));
  nurbs.SetKnot (1, 0, bb (2));
  nurbs.SetKnot (1, 1, bb (3));

  // warning, normals are pointing along view-vector of camera
  pt (2) = 1.0;
  pt (0) = pt (2) * (bb (0) - intrinsic (0, 2)) / intrinsic (0, 0);
  pt (1) = pt (2) * (bb (2) - intrinsic (1, 2)) / intrinsic (1, 1);
  dir = pt - p0;
  dir.normalize ();
  p1 = objectmodeling::Triangulation::intersectPlaneLine (plane, p0, dir);
  nurbs.SetCV (0, 0, ON_3dPoint (p1 (0), p1 (1), p1 (2)));

  pt (2) = 1.0;
  pt (0) = pt (2) * (bb (1) - intrinsic (0, 2)) / intrinsic (0, 0);
  pt (1) = pt (2) * (bb (2) - intrinsic (1, 2)) / intrinsic (1, 1);
  dir = pt - p0;
  dir.normalize ();
  p1 = objectmodeling::Triangulation::intersectPlaneLine (plane, p0, dir);
  nurbs.SetCV (1, 0, ON_3dPoint (p1 (0), p1 (1), p1 (2)));

  pt (2) = 1.0;
  pt (0) = pt (2) * (bb (1) - intrinsic (0, 2)) / intrinsic (0, 0);
  pt (1) = pt (2) * (bb (3) - intrinsic (1, 2)) / intrinsic (1, 1);
  dir = pt - p0;
  dir.normalize ();
  p1 = objectmodeling::Triangulation::intersectPlaneLine (plane, p0, dir);
  nurbs.SetCV (1, 1, ON_3dPoint (p1 (0), p1 (1), p1 (2)));

  pt (2) = 1.0;
  pt (0) = pt (2) * (bb (0) - intrinsic (0, 2)) / intrinsic (0, 0);
  pt (1) = pt (2) * (bb (3) - intrinsic (1, 2)) / intrinsic (1, 1);
  dir = pt - p0;
  dir.normalize ();
  p1 = objectmodeling::Triangulation::intersectPlaneLine (plane, p0, dir);
  nurbs.SetCV (0, 1, ON_3dPoint (p1 (0), p1 (1), p1 (2)));

  return nurbs;
}

void
ContourRefinement::computeSurfaces ()
{
  if (!m_curves_computed)
  {
    std::cout << "[ContourRefinement::computeSurfaces] Error, curves not computed." << endl;
    return;
  }

  pcl::on_nurbs::FittingSurfaceIM fit;
  fit.setInputCloud (m_cloud);
  fit.setCamera (m_view->intrinsic);
  Eigen::Matrix3d intrinsic = m_view->intrinsic;

  // #pragma omp parallel for private (fit)
  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = m_view->surfaces[m_surface_indices[i]];

    if (surface->curves_image.empty ())
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeSurfaces] surface[%lu] Warning: No curve in surface.\n", i);
      continue;
    }

    ON_NurbsCurve &curve2D = surface->curves_image[0];
    if (curve2D.KnotCount () < 1)
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeSurfaces] surface[%lu] Warning: ON knot vector empty. Skipping.\n", i);
      continue;
    }

    Eigen::Vector4d bb = computeCPBoundingBox (curve2D);

    if (surface->type == pcl::SACMODEL_PLANE)
    {
      // create ON_NurbsSurface from plane equation (surface->coeffs)
      surface->nurbs = computeSurfaceFromPlane (surface->coeffs, intrinsic, bb);
    }
    else if (surface->type == MODEL_NURBS)
    {
      // create ON_NurbsSurface by fitting to point cloud
      fit.setIndices (surface->indices);

      fit.initSurface (m_params.surface_order, bb);

      for (size_t j = 0; j < m_params.surface_refinement; j++)
        fit.refine ();

      fit.assemble ();
      fit.solve ();

      for (size_t j = 0; j < m_params.surface_iterations - 1; j++)
      {
        fit.assemble (true);
        fit.solve ();
      }

      printf ("[ContourRefinement::computeSurfaces] Warning, normals of surface might not be correct (surface[%lu])\n",
              i);

      // #pragma omp critical
      surface->nurbs = fit.getSurface ();
    }
  }

  m_surfaces_computed = true;
}

void
ContourRefinement::computeCurvesParam ()
{
  if (!m_surfaces_computed)
  {
    std::cout << "[ContourRefinement::computeSurfaces] Error, surfaces not computed." << endl;
    return;
  }

  Eigen::Matrix3d intrinsic = m_view->intrinsic;

  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = m_view->surfaces[m_surface_indices[i]];

    if (surface->curves_image.empty ())
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeCurvesParam] [%lu] Warning no curve_image available\n");
      continue;
    }

    ON_NurbsCurve &curve_image = surface->curves_image[0];
    surface->curves_param.assign (1, curve_image);
    ON_NurbsCurve &curve_param = surface->curves_param[0];

    if (curve_image.KnotCount () < 1)
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeCurvesParam] surface[%lu] Warning: curve knot vector empty. Skipping.\n", i);
      continue;
    }

    if (surface->nurbs.KnotCount(0) < 1 || surface->nurbs.KnotCount(1) < 1)
    {
      if (!m_quiet)
        printf ("[ContourRefinement::computeCurvesParam] surface[%lu] Warning: surface knot vector empty. Skipping.\n", i);
      continue;
    }

    //    if (surface->type == pcl::SACMODEL_PLANE)
    //    {
    //      Eigen::Matrix3d R, Rt;
    //      Eigen::Vector3d t;
    //
    //      Triangulation::getPlaneBasis (surface->coeffs, R, t);
    //      Rt = R.transpose ();
    //
    //      // project curve2D on nurbs surface
    //      for (int j = 0; j < curve_image.CVCount (); j++)
    //      {
    //        ON_3dPoint cp;
    //        curve_image.GetCV (j, cp);
    //
    //        Eigen::Vector3d p0 (0.0, 0.0, 0.0);
    //        Eigen::Vector3d pt;
    //        pt (2) = 1.0;
    //        pt (0) = pt (2) * (cp.x - intrinsic (0, 2)) / intrinsic (0, 0);
    //        pt (1) = pt (2) * (cp.y - intrinsic (1, 2)) / intrinsic (1, 1);
    //        Eigen::Vector3d dir = pt - p0;
    //        dir.normalize ();
    //
    //        Eigen::Vector3d p1 = Triangulation::intersectPlaneLine (surface->coeffs, p0, dir);
    //
    //        Eigen::Vector3d p1t = Rt * (p1 - t);
    //
    //        cp.x = p1t (0);
    //        cp.y = p1t (1);
    //        curve_param.SetCV (j, cp);
    //      }
    //    }
    //    else
    //    {
    // project curve2D on nurbs surface
    for (int j = 0; j < curve_image.CVCount (); j++)
    {
      ON_3dPoint cp;
      curve_image.GetCV (j, cp);

      Eigen::Vector3d p0 (0.0, 0.0, 0.0);
      Eigen::Vector3d pt;
      pt (2) = 1.0;
      pt (0) = pt (2) * (cp.x - intrinsic (0, 2)) / intrinsic (0, 0);
      pt (1) = pt (2) * (cp.y - intrinsic (1, 2)) / intrinsic (1, 1);
      Eigen::Vector3d dir = pt - p0;
      dir.normalize ();

      Eigen::Vector2d params = Triangulation::intersectNurbsLine (surface->nurbs, p0, dir, 1.0, 100, 1e-4);

      //        double points[3];
      //        m_surfaces[i].Evaluate (params (0), params (1), 0, 3, points);
      //        Eigen::Vector3d p1 (points[0], points[1], points[2]);
      cp.x = params (0);
      cp.y = params (1);

      curve_param.SetCV (j, cp);
    }
    //    }
  }
}

void
ContourRefinement::trimSurfacePoints2 (double tolerance)
{
  if (!m_curves_computed)
  {
    std::cout << "[ContourRefinement::trimSurfacePoints] Error, curves2D not computed." << endl;
    return;
  }
  if (!m_surfaces_computed)
  {
    std::cout << "[ContourRefinement::trimSurfacePoints] Error, surfaces not computed, cannot project points." << endl;
  }

  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;

  // fill label matrix
  createLabelMatrix ();
  m_zbuffer = cv::Mat_<double>::zeros (cloud_ref.height, cloud_ref.width);
  m_zbuffer.setTo (DBL_MAX);
  cv::Mat_<int> labels;
  m_labels.copyTo (labels);

  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    surface::SurfaceModel::Ptr surf1 = m_view->surfaces[m_surface_indices[i]];

    if (surf1->curves_image.empty ())
      continue;

    ON_NurbsCurve &curve2D = surf1->curves_image[0];

    if (curve2D.KnotCount () < 1)
      continue;

    unsigned reasigned (0);
    cv::Mat_<bool> visited = cv::Mat_<bool>::zeros (cloud_ref.height, cloud_ref.width);

    for (size_t j = 0; j < surf1->contours[0].size (); j++)
    {
      int &idx = surf1->contours[0][j];
      int px = idx % cloud_ref.width;
      int py = idx / cloud_ref.width;

      for (int u = -m_params.kernel; u <= m_params.kernel; u++)
      {
        for (int v = -m_params.kernel; v <= m_params.kernel; v++)
        {
          int x = px + u;
          int y = py + v;

          if (x < m_bb (0) || x > m_bb (1) || y < m_bb (2) || y > m_bb (3))
            continue;

          if (visited (y, x))
            continue;
          else
            visited (y, x) = true;

          if (!isInsideCurve (curve2D, Eigen::Vector2d (x, y), tolerance))
          {
            // search for neighbors
            int other_curve (-1);

            std::vector<int> neighbors;
            getNeighborLabels (x, y, i, neighbors, m_params.kernel);

            // check if point is inside a boundary of a neighbor
            for (size_t k = 0; k < neighbors.size (); k++)
            {
              if (neighbors[k] != (int)i)
              {
                surface::SurfaceModel::Ptr s2 = m_view->surfaces[neighbors[k]];
                if (s2->curves_image.empty ())
                  continue;

                if (isInsideCurve (s2->curves_image[0], Eigen::Vector2d (x, y), tolerance))
                {
                  other_curve = neighbors[k];
                  break;
                }
              }
            }

            if (other_curve >= 0)
            {

              surface::SurfaceModel::Ptr s2 = m_view->surfaces[other_curve];

              // project points
              pcl::PointXYZ pt;
              if (s2->type == pcl::SACMODEL_PLANE)
                pt = projectPoint (x, y, s2->coeffs, m_view->intrinsic);
              else if (s2->nurbs.KnotCount (0) >= 1 && s2->nurbs.KnotCount (1) >= 1)
                pt = projectPoint (x, y, s2->nurbs, m_view->intrinsic);

              if (pt.z < m_zbuffer (y, x))
              {
                if (labels (y, x) != other_curve)
                {
                  pcl::PointXYZRGB &pc = cloud_ref.at (x, y);
                  pc.x = pt.x;
                  pc.y = pt.y;
                  pc.z = pt.z;
                }
                labels (y, x) = other_curve;
                m_zbuffer (y, x) = pt.z;
                reasigned++;
              }

              //              cv::Vec3b &col = m_image (y, x);
              //              pc.r = col.val[2];
              //              pc.g = col.val[1];
              //              pc.b = col.val[0];

            }
            else
            {
              labels (y, x) = -1;
              pcl::PointXYZRGB &pc = cloud_ref.at (x, y);
              pc.x = 0.0;
              pc.y = 0.0;
              pc.z = 0.0;
            } // if (other_curve >= 0)


          } // if (!isInsideCurve (curve2D, Eigen::Vector2d (px, py), tolerance))
          else
          {
            // project points
            pcl::PointXYZ pt;
            if (surf1->type == pcl::SACMODEL_PLANE)
              pt = projectPoint (x, y, surf1->coeffs, m_view->intrinsic);
            else if (surf1->nurbs.KnotCount (0) >= 1 && surf1->nurbs.KnotCount (1) >= 1)
              pt = projectPoint (x, y, surf1->nurbs, m_view->intrinsic);

            if (pt.z <= m_zbuffer (y, x))
            {
              if (labels (y, x) != i)
              {
                pcl::PointXYZRGB &pc = cloud_ref.at (x, y);
                pc.x = pt.x;
                pc.y = pt.y;
                pc.z = pt.z;
              }
              labels (y, x) = i;
              m_zbuffer (y, x) = pt.z;
            }
            //            cv::Vec3b &col = m_image (y, x);
            //            pc.r = col.val[2];
            //            pc.g = col.val[1];
            //            pc.b = col.val[0];

          } // if (!isInsideCurve (curve2D, Eigen::Vector2d (px, py), tolerance))

        } // for (int v = -kernel; v <= kernel; v++)
      } // for (int u = -kernel; u <= kernel; u++)

    } // for (size_t j = 0; j < surface->indices.size (); j++)

    if (!m_quiet)
      printf ("[%lu] reasigned: %d\n", i, reasigned);

    surf1->indices.clear ();
    surf1->error.clear ();
    surf1->probs.clear ();
    surf1->normals.clear ();
  }

  labels.copyTo (m_labels);

  for (int y = 0; y < m_labels.rows; y++)
  {
    for (int x = 0; x < m_labels.cols; x++)
    {
      int &label = m_labels (y, x);

      if (label >= 0 && label < m_view->surfaces.size ())
      {
        surface::SurfaceModel::Ptr s1 = m_view->surfaces[label];
        s1->indices.push_back (y * cloud_ref.width + x);
        s1->error.push_back (0.0);
        s1->probs.push_back (0.0);
        s1->normals.push_back (Eigen::Vector3d (0.0, 0.0, 0.0));
      }

    }
  }

}

void
ContourRefinement::trimSurfacePoints (double tolerance, bool reasign, bool project)
{
  if (!m_curves_computed)
  {
    std::cout << "[ContourRefinement::trimSurfacePoints] Error, curves2D not computed." << endl;
    return;
  }
  if (project && !m_surfaces_computed)
  {
    std::cout << "[ContourRefinement::trimSurfacePoints] Error, surfaces not computed, cannot project points." << endl;
    project = false;
  }

  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;

  // fill label matrix
  createLabelMatrix ();

  for (size_t i = 0; i < m_surface_indices.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = m_view->surfaces[m_surface_indices[i]];

    if (surface->curves_image.empty ())
      continue;

    ON_NurbsCurve &curve2D = surface->curves_image[0];

    if (curve2D.KnotCount () < 1)
      continue;

    unsigned reasigned (0);

    std::vector<int> indices;
    std::vector<double> error;
    std::vector<double> probs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > normals;

    for (size_t j = 0; j < surface->indices.size (); j++)
    {
      int &idx = surface->indices[j];
      int px = idx % cloud_ref.width;
      int py = idx / cloud_ref.width;

      if (!isInsideCurve (curve2D, Eigen::Vector2d (px, py), tolerance))
      {
        if (reasign)
        {
          // search for neighbors
          int other_curve (-1);
          std::vector<int> neighbors;
          getNeighborLabels (px, py, i, neighbors, m_params.kernel);

          // check if point is inside a boundary of a neighbor
          for (size_t k = 0; k < neighbors.size (); k++)
          {
            if (neighbors[k] != (int)i)
            {
              surface::SurfaceModel::Ptr s2 = m_view->surfaces[neighbors[k]];
              if (s2->curves_image.empty ())
                continue;

              if (isInsideCurve (s2->curves_image[0], Eigen::Vector2d (px, py), tolerance))
              {
                other_curve = neighbors[k];
                break;
              }
            }
          }

          if (other_curve >= 0)
          {
            surface::SurfaceModel::Ptr s2 = m_view->surfaces[other_curve];
            s2->indices.push_back (idx);
            s2->error.push_back (surface->error[j]);
            s2->probs.push_back (surface->probs[j]);
            s2->normals.push_back (surface->normals[j]);
            reasigned++;

            if (project)
            {
              if (s2->type == pcl::SACMODEL_PLANE)
                projectPoint (cloud_ref.at (idx), s2->coeffs);
              else
                projectPoint (cloud_ref.at (idx), s2->nurbs);
            }

          }
          else
          {
            indices.push_back (idx);
            error.push_back (surface->error[j]);
            probs.push_back (surface->probs[j]);
            normals.push_back (surface->normals[j]);
          } // if (other_curve >= 0)

        } // if (reasign)

      } // if (!isInsideCurve (curve2D, Eigen::Vector2d (px, py), tolerance))
      else
      {
        indices.push_back (idx);
        error.push_back (surface->error[j]);
        probs.push_back (surface->probs[j]);
        normals.push_back (surface->normals[j]);
      } // if (!isInsideCurve (curve2D, Eigen::Vector2d (px, py), tolerance))

    } // for (size_t j = 0; j < surface->indices.size (); j++)

    if (!m_quiet)
    {
      printf ("[%lu] indices: %lu | ", i, surface->indices.size ());
      printf ("reasigned: %d\n", reasigned);
    }

    surface->indices = indices;
    surface->error = error;
    surface->probs = probs;
    surface->normals = normals;
  }
}

void
ContourRefinement::reasignNAN (double zthreshold, size_t kernel, bool project)
{
  if (!m_curves_computed)
  {
    std::cout << "[ContourRefinement::reasignNAN] Error, curves2D not computed." << endl;
    return;
  }
  if (!m_surfaces_computed)
  {
    std::cout << "[ContourRefinement::reasignNAN] Error, surfaces not computed, cannot project points." << endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;

  // fill label matrix
  createLabelMatrix ();
  unsigned nan (0);
  unsigned reasigned (0);

  std::map<size_t, std::map<size_t, pcl::PointXYZRGB> > new_points;

  for (size_t y = 0; y < cloud_ref.height; y++)
  {
    for (size_t x = 0; x < cloud_ref.width; x++)
    {
      pcl::PointXYZRGB pc = cloud_ref.at (x, y);

      if (!isValid (pc))
      {

        nan++;

        if (x < m_bb (0) || x > m_bb (1) || y < m_bb (2) || y > m_bb (3))
          continue;

        Eigen::Vector3d n;
        int nl;
        if (reasignPoint (x, y, pc, nl, n, zthreshold, kernel))
        {
          surface::SurfaceModel::Ptr surf = m_view->surfaces[nl];
          int idx = y * cloud_ref.width + x;

          surf->indices.push_back (idx);
          surf->error.push_back (0.0);
          surf->probs.push_back (1.0);
          surf->normals.push_back (n);
          reasigned++;

          new_points[x][y] = pc;
        }

      } // if (if (!isValid (pc))
    }
  }

  if (!m_quiet)
    printf ("[ContourRefinement::reasignNAN] reasigned: %d / %d\n", reasigned, nan);

  if (project)
  {
    std::map<size_t, std::map<size_t, pcl::PointXYZRGB> >::iterator itx;
    std::map<size_t, pcl::PointXYZRGB>::iterator ity;
    for (itx = new_points.begin (); itx != new_points.end (); itx++)
    {
      size_t x = itx->first;
      for (ity = itx->second.begin (); ity != itx->second.end (); ity++)
      {
        size_t y = ity->first;
        m_cloud->at (x, y) = ity->second;
      }
    }
  }

}

bool
ContourRefinement::isInsideCurve (const ON_NurbsCurve &curve, Eigen::Vector2d point, double tolerance) const
{
  if (!m_curves_computed)
  {
    std::cout << "[ContourRefinement::computeSurfaces] Error, curves not computed." << endl;
    return false;
  }

  if (curve.KnotCount () < 1)
    return false;

  double err;
  Eigen::Vector2d p, t;
  double scale = m_params.m_curve_params.param.rScale;
  double xi = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, point);
  pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, point, xi, err, p, t, scale, 200, 1e-4, true);

  Eigen::Vector2d v (point (0) - p (0), point (1) - p (1));
  double z = v (0) * t (1) - v (1) * t (0);

  return (z < 0.0 || v.squaredNorm () < (tolerance * tolerance));
}

void
ContourRefinement::visualizeCurve2D (ON_NurbsCurve curve, double height)
{

  for (int i = 0; i < curve.CVCount (); i++)
  {
    ON_3dPoint p;
    curve.GetCV (i, p);
    p.y = height - p.y;
    curve.SetCV (i, p);
  }
  TomGine::tgModel model;
  objectmodeling::Triangulation::Convert (curve, model, 8, false);
  model.m_point_color = TomGine::vec3 (0.0, 0.8, 0.0);
  model.m_line_width = 1.0;
  if (m_viewer != NULL)
  {
    m_viewer->ClearModels2D ();
    m_viewer->AddModel2D (model);
  }
}

void
ContourRefinement::visualizeCurve3D (ON_NurbsCurve curve, double height, double scale)
{

  for (int i = 0; i < curve.CVCount (); i++)
  {
    ON_3dPoint p;
    curve.GetCV (i, p);
    p = p * scale;
    curve.SetCV (i, p);
  }
  TomGine::tgModel model;
  objectmodeling::Triangulation::Convert (curve, model, 8, true);
  model.m_point_color = TomGine::vec3 (0.0, 0.8, 0.0);
  model.m_point_size = 5.0;
  model.m_line_width = 1.0;

  //  std::vector<double> elements = pcl::on_nurbs::FittingCurve2dAPDM::getElementVector (curve);
  //  double points[2];
  //  double seg = 1.0 / (curve.Order () - 1);
  //  for (unsigned i = 0; i < elements.size () - 1; i++)
  //  {
  //    double &xi0 = elements[i];
  //    double &xi1 = elements[i + 1];
  //    double dxi = xi1 - xi0;
  //
  //    for (unsigned j = 0; j < curve.Order (); j++)
  //    {
  //      double xi = xi0 + (seg * j) * dxi;
  //
  //      curve.Evaluate (xi, 0, 2, points);
  //
  //      model.m_points.push_back (TomGine::vec3 (points[0], points[1], 0.0));
  //    }
  //  }
  if (m_viewer != NULL)
  {
    m_viewer->ClearModels3D ();
    m_viewer->AddModel3D (model);
  }
}

