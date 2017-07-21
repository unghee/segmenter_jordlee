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

#include "SurfaceBoundary.h"
#include <pcl/sample_consensus/model_types.h>

#include <pcl/surface/on_nurbs/closing_boundary.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_atdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>

// TODO
#include "Triangulation.h"

using namespace surface;
using namespace objectmodeling;

void SurfaceBoundary::convertIndexMap2ColorMap(cv::Mat_<uchar> &index_map,
    cv::Mat_<cv::Vec3b> &color_map)
{
  std::vector<cv::Vec3b> colors;

  color_map = cv::Mat_<cv::Vec3b>(index_map.rows, index_map.cols);

  for (int j = 0; j < index_map.cols; j++) {
    for (int i = 0; i < index_map.rows; i++) {
      int ind = index_map(i, j);

      if (ind == 255) {
        color_map(i, j) = cv::Vec3b(255, 255, 255);
        continue;
      }

      while (ind >= (int) colors.size()) {
        cv::Vec3b col;
        col[0] = uchar(100 + 155 * double(rand()) / RAND_MAX);
        col[1] = uchar(100 + 155 * double(rand()) / RAND_MAX);
        col[2] = uchar(100 + 155 * double(rand()) / RAND_MAX);
        colors.push_back(col);
      }

      color_map(i, j) = colors[ind];
    }
  }

}

void SurfaceBoundary::createIndexMap(surface::View &view, cv::Mat_<uchar> &index_map)
{
  if (view.surfaces.empty())
    return;

  unsigned width = view.width;
  unsigned height = view.height;
  index_map = cv::Mat_<uchar>(height, width);
  index_map.setTo(255);

  for (unsigned i = 0; i < view.surfaces.size(); i++) {
    for (unsigned j = 0; j < view.surfaces[i]->indices.size(); j++) {
      int ind = view.surfaces[i]->indices[j];

      int r = ind / width;
      int c = ind % width;
      index_map(r, c) = i;
    }
  }
}

void SurfaceBoundary::createNurbs(surface::View &view,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    std::vector<pcl::on_nurbs::NurbsDataSurface*> &data)
{
  if (view.surfaces.empty())
    return;

  unsigned width = view.width;
  unsigned height = view.height;
  double dsw = double(cloud->width) / width;
  double dsh = double(cloud->height) / height;

  data.assign(view.surfaces.size(), NULL);
  for (unsigned i = 0; i < view.surfaces.size(); i++) {

    data[i] = new pcl::on_nurbs::NurbsDataSurface;

    for (unsigned j = 0; j < view.surfaces[i]->indices.size(); j++) {
      int ind = view.surfaces[i]->indices[j];

      int r = ind / width;
      int c = ind % width;
      int x = c * dsw;
      int y = r * dsh;
      pcl::PointXYZRGB &p = cloud->at(x, y);
      if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
        data[i]->interior.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    // create nurbs from plane
    if (!data[i]->interior.empty()) {
      if (view.surfaces[i]->type == pcl::SACMODEL_PLANE) {
        int order = 2;
        view.surfaces[i]->nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order,
            data[i]);
      } else {
        data[i]->mean = pcl::on_nurbs::NurbsTools::computeMean(data[i]->interior);
        data[i]->interior_param = view.surfaces[i]->nurbs_params;
      }
    }else{
      printf("[SurfaceBoundary::createNurbs] Warning, view.surfaces[%d] contains no data\n", i);
    }

  }
}

void SurfaceBoundary::createCommonBoundaries(surface::View &view,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat_<uchar> &index_map,
    std::vector<pcl::on_nurbs::NurbsDataSurface*> &data)
{
  if (view.surfaces.empty())
    return;

  unsigned width = view.width;
  unsigned height = view.height;
  double dsw = double(cloud->width) / width;
  double dsh = double(cloud->height) / height;

  for (int r = 0; r < index_map.rows - 1; r++) {
    for (int c = 0; c < index_map.cols - 1; c++) {
      uchar a;
      uchar m = index_map(r, c);
      if (m == 255)
        continue;

      a = index_map(r + 1, c);
      if (a != 255 && a != m && view.surfaces[a]->label == view.surfaces[m]->label) {
        pcl::PointXYZRGB &p1 = cloud->at(c * dsw, r * dsh);
        pcl::PointXYZRGB &p2 = cloud->at(c * dsw, (r - 1) * dsh);
        if (!isnan(p1.x) && !isnan(p1.y) && !isnan(p1.z) && !isnan(p2.x) && !isnan(p2.y) && !isnan(
            p2.z)) {
          Eigen::Vector3d ep1(p1.x, p1.y, p1.z);
          Eigen::Vector3d ep2(p2.x, p2.y, p2.z);
          //          p1.r = p2.r = 255;
          //          p1.g = p2.g = 0;
          //          p1.b = p2.b = 0;
          data[m]->common_boundary_point.push_back((ep1 + ep2) * 0.5);
          data[m]->common_boundary_idx.push_back(a);
          data[a]->common_boundary_point.push_back((ep1 + ep2) * 0.5);
          data[a]->common_boundary_idx.push_back(m);
        }
      }

      a = index_map(r, c + 1);
      if (a != 255 && a != m && view.surfaces[a]->label == view.surfaces[m]->label) {
        pcl::PointXYZRGB &p1 = cloud->at(c * dsw, r * dsh);
        pcl::PointXYZRGB &p2 = cloud->at(c * dsw, (r - 1) * dsh);
        if (!isnan(p1.x) && !isnan(p1.y) && !isnan(p1.z) && !isnan(p2.x) && !isnan(p2.y) && !isnan(
            p2.z)) {
          Eigen::Vector3d ep1(p1.x, p1.y, p1.z);
          Eigen::Vector3d ep2(p2.x, p2.y, p2.z);
          //          p1.r = p2.r = 255;
          //          p1.g = p2.g = 0;
          //          p1.b = p2.b = 0;
          data[m]->common_boundary_point.push_back((ep1 + ep2) * 0.5);
          data[m]->common_boundary_idx.push_back(a);
          data[a]->common_boundary_point.push_back((ep1 + ep2) * 0.5);
          data[a]->common_boundary_idx.push_back(m);
        }
      }

    }
  }

}

void SurfaceBoundary::createSurfaceBoundaries(surface::View &view,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat_<uchar> &index_map,
    std::vector<pcl::on_nurbs::NurbsDataSurface*> &data)
{
  if (view.surfaces.empty())
    return;

  unsigned width = view.width;
  unsigned height = view.height;
  double dsw = double(cloud->width) / width;
  double dsh = double(cloud->height) / height;

  for (int r = 1; r < index_map.rows - 1; r++) {
    for (int c = 1; c < index_map.cols - 1; c++) {
      uchar m = index_map(r, c);
      if (m == 255)
        continue;

      uchar a1 = index_map(r + 1, c);
      uchar a2 = index_map(r - 1, c);
      uchar a3 = index_map(r, c + 1);
      uchar a4 = index_map(r, c - 1);
      if (a1 != m || a2 != m || a3 != m || a4 != m) {
        pcl::PointXYZRGB &p1 = cloud->at(c * dsw, r * dsh);
        if (!isnan(p1.x) && !isnan(p1.y) && !isnan(p1.z)) {
          Eigen::Vector3d ep1(p1.x, p1.y, p1.z);
          //          p1.r = 0;
          //          p1.g = 255;
          //          p1.b = 0;
          data[m]->boundary.push_back(ep1);
        }
      }

    }
  }

}

void SurfaceBoundary::createObjectBoundaries2d(cv::Mat_<uchar> &index_map,
    std::vector<pcl::on_nurbs::NurbsDataCurve2d> &boundaries, bool use_asymetric_weight)
{

  double sx = 1.0 / index_map.cols;
  double sy = 1.0 / index_map.rows;
  double scale = std::min(sx, sy);

  for (int i = 1; i < index_map.rows - 1; i++) {
    for (int j = 1; j < index_map.cols - 1; j++) {
      uchar a, b, c, d;
      uchar m = index_map(i, j);

      if (m == 255)
        continue;

      a = index_map(i - 1, j);
      b = index_map(i + 1, j);

      c = index_map(i, j - 1);
      d = index_map(i, j + 1);

      if (m == a && m == b && m == c && m == d)
        continue;
      else {
        while (m >= boundaries.size())
          boundaries.push_back(pcl::on_nurbs::NurbsDataCurve2d());

        boundaries[m].interior.push_back(Eigen::Vector2d(j * scale, (index_map.rows - i) * scale));
        boundaries[m].interior_weight.push_back(1.0);
        boundaries[m].interior_weight_function.push_back(use_asymetric_weight);
      }

    }
  }

}

void SurfaceBoundary::createNurbsBoundaries(TomGine::tgTomGineThread &dbgWin, surface::View &view,
    std::vector<pcl::on_nurbs::NurbsDataSurface*> &data, std::vector<ON_NurbsCurve> &nurbs,
    FitBoundaryParameter &param)
{
  unsigned s = view.surfaces.size();

  if (data.size() < s)
    throw std::runtime_error(
        "[SurfaceBoundary::createNurbsBoundaries] Surfaces and data do not match in size.");

  for (unsigned i = 0; i < s && !dbgWin.Stopped(); i++) {

    dbgWin.ClearPoints3D();
    dbgWin.ClearModels();
    dbgWin.ClearLabels();

    pcl::on_nurbs::NurbsDataCurve2d data_bnd;
    unsigned nInt = data[i]->boundary.size();
    for (unsigned p = 0; p < nInt; p++) {
      Eigen::Vector3d &pcp = data[i]->boundary[p];
      Eigen::Vector2d xi;
      Eigen::Vector3d pt, tu, tv, n;
      double error;
      xi = pcl::on_nurbs::FittingSurface::findClosestElementMidPoint(view.surfaces[i]->nurbs, pcp);
      xi = pcl::on_nurbs::FittingSurface::inverseMapping(view.surfaces[i]->nurbs, pcp, xi, error,
          pt, tu, tv, param.invMapMaxSteps, param.invMapAccuracy);
      data_bnd.interior.push_back(xi);
      data_bnd.interior_weight.push_back(param.wInterior);
      data_bnd.interior_weight_function.push_back(true);
      dbgWin.AddPoint3D(xi(0), xi(1), 0.0, 0, 0, 0, 1.0f);
    }

    for (unsigned j = 0; j < data[i]->common_boundary_idx.size(); j++) {
      unsigned &idx = data[i]->common_boundary_idx[j];
      Eigen::Vector3d &start = data[i]->common_boundary_point[j];

      view.surfaces[idx]->nurbs;

      Eigen::Vector2d params1, params2;
      Eigen::Vector3d pt;
      unsigned nsteps(20);
      double error;
      double accuracy(1e-3);

      pt = pcl::on_nurbs::ClosingBoundary::commonBoundaryPoint1(view.surfaces[i]->nurbs,
          view.surfaces[idx]->nurbs, params1, params2, start, nsteps, error, accuracy);

      data_bnd.interior.push_back(params1);
      data_bnd.interior_weight.push_back(param.wCommon);
      data_bnd.interior_weight_function.push_back(false);
      dbgWin.AddPoint3D(params1(0), params1(1), 0.0, 0, 100, 100, 2.0f);
    }

    //    dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Space);

    ON_NurbsCurve nc = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(param.order,
        data_bnd.interior);
    fitBoundaryNurbs(dbgWin, nc, data_bnd, param);
    nurbs.push_back(nc);

    TomGine::tgRenderModel model;
    Triangulation::Convert(nurbs[i], view.surfaces[i]->nurbs, model, 2048, true);
    dbgWin.AddModel(model);

    //    dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Space);

  }
}

void SurfaceBoundary::fitBoundaryNurbs(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
    pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param)
{
  //  ON_NurbsCurve nurbs = FittingCurve2dAPDM::initNurbsCurve2D(param.order, data.interior);
  pcl::on_nurbs::FittingCurve2dAPDM fit(&data, nurbs);

  int id;
  int txt_id[3];
  ostringstream os;
  os.str("");
  int h = dbgWin.GetCamera().GetHeight();
  txt_id[0] = dbgWin.AddLabel2D("", 10, 5, h - 20, 0, 0, 0);
  txt_id[1] = dbgWin.AddLabel2D("", 10, 5, h - 40, 0, 0, 0);
  txt_id[2] = dbgWin.AddLabel2D("", 10, 5, h - 60, 0, 0, 0);

  double avgerr(DBL_MAX);
  double maxerr(DBL_MAX);
  bool stop(false);
  for (unsigned j = 0; j < param.fitMaxSteps && !stop && !dbgWin.Stopped(); j++) {
    if (2 * fit.m_nurbs.CVCount() > (int) data.interior.size()) {
      break;
    } else if (j > 0 && j <= param.refinement) {
      fit.refine();
    } else if (j > param.refinement + 1) {
      if (!(j % param.addCPsIteration) && ((unsigned) fit.m_nurbs.CVCount() < param.maxCPs))
        fit.addCPsOnClosestPointViolation(param.addCPsAccuracy);
    }

    data.interior_param.clear();

    fit.assemble(param.paramFC);

    unsigned s = data.closest_points_error.size();
    avgerr = 0.0;
    maxerr = 0.0;
    for (unsigned i = 0; i < s; i++) {
      double &e = data.closest_points_error[i];
      avgerr += (e / s);
      if (e > maxerr) {
        maxerr = e;
      }
    }
    maxerr = sqrt(maxerr);

    if (j > param.refinement)
      stop = (param.fitMaxError > maxerr && param.fitAvgError > avgerr);

    // Visualisation
    {
      TomGine::tgRenderModel model;
      Triangulation::Convert(fit.m_nurbs, model, 8, true);
      model.m_line_width = 3.0;
      if (j == 0)
        id = dbgWin.AddModel(model);
      else
        dbgWin.SetModel(id, model);

      dbgWin.ClearLines3D();
      for (unsigned i = 0; i < data.interior_line_start.size() && i < data.interior_line_end.size(); i++) {
        Eigen::Vector2d &a = data.interior_line_start[i];
        Eigen::Vector2d &b = data.interior_line_end[i];
        dbgWin.AddLine3D(a(0), a(1), 0.0f, b(0), b(1), 0.0f, 0, 100, 50, 2.0f);
      }

      os.str("");
      os << "iteration: " << j << "  ncps: " << fit.m_nurbs.CVCount();
      dbgWin.SetLabel2D(txt_id[0], os.str(), 10, 5, h - 20, 0, 0, 0);
      os.str("");
      os << "avg error: " << avgerr << " / " << param.fitAvgError;
      dbgWin.SetLabel2D(txt_id[1], os.str(), 10, 5, h - 40, 0, 0, 0);
      os.str("");
      os << "max error: " << maxerr << " / " << param.fitMaxError;
      dbgWin.SetLabel2D(txt_id[2], os.str(), 10, 5, h - 60, 0, 0, 0);
      dbgWin.Update();

      //      dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Space);
    }

    fit.solve();
  }

  //  dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Return);

  // clamp to parametric domain of NURBS surface
  for (int i = 0; i < fit.m_nurbs.CVCount(); i++) {
    ON_3dPoint cv;
    fit.m_nurbs.GetCV(i, cv);

    if (cv.x < 0.0)
      cv.x = 0.0;
    if (cv.y < 0.0)
      cv.y = 0.0;
    if (cv.x > 1.0)
      cv.x = 1.0;
    if (cv.y > 1.0)
      cv.y = 1.0;
    cv.z = 0.0;

    fit.m_nurbs.SetCV(i, cv);
  }

  nurbs = fit.m_nurbs;
}

void SurfaceBoundary::fitBoundaryNurbs_SDM(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
    pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param)
{
  //  ON_NurbsCurve nurbs = FittingCurve2dAPDM::initNurbsCurve2D(param.order, data.interior);
  pcl::on_nurbs::FittingCurve2dASDM fit(&data, nurbs);

  int id;
  int txt_id[3];
  ostringstream os;
  os.str("");
  int h = dbgWin.GetCamera().GetHeight();
  dbgWin.ClearLabels();
  txt_id[0] = dbgWin.AddLabel2D("", 10, 5, h - 20, 0, 0, 0);
  txt_id[1] = dbgWin.AddLabel2D("", 10, 5, h - 40, 0, 0, 0);
  txt_id[2] = dbgWin.AddLabel2D("", 10, 5, h - 60, 0, 0, 0);

  double avgerr(DBL_MAX);
  double maxerr(DBL_MAX);
  bool stop(false);
  for (unsigned j = 0; j < param.fitMaxSteps && !stop && !dbgWin.Stopped(); j++) {
    if (2 * fit.m_nurbs.CVCount() > (int) data.interior.size()) {
      ; //break;
    } else if (j > 0 && j <= param.refinement) {
      fit.refine();
    } else if (j > param.refinement + 1) {
      if (!(j % param.addCPsIteration) && ((unsigned) fit.m_nurbs.CVCount() < param.maxCPs))
        fit.addCPsOnClosestPointViolation(param.addCPsAccuracy);
    }

    data.interior_param.clear();

    fit.assemble(param.paramFC);

    unsigned s = data.closest_points_error.size();
    avgerr = 0.0;
    maxerr = 0.0;
    for (unsigned i = 0; i < s; i++) {
      double &e = data.closest_points_error[i];
      avgerr += (e / s);
      if (e > maxerr) {
        maxerr = e;
      }
    }
    maxerr = sqrt(maxerr);

    if (j > param.refinement)
      stop = (param.fitMaxError > maxerr && param.fitAvgError > avgerr);

    // Visualisation
    {
      TomGine::tgRenderModel model;
      Triangulation::Convert(fit.m_nurbs, model, 8, true);
      model.m_line_width = 3.0;
      if (j == 0)
        id = dbgWin.AddModel(model);
      else
        dbgWin.SetModel(id, model);

      dbgWin.ClearLines3D();
      for (unsigned i = 0; i < data.interior_line_start.size() && i < data.interior_line_end.size(); i++) {
        Eigen::Vector2d &a = data.interior_line_start[i];
        Eigen::Vector2d &b = data.interior_line_end[i];
        dbgWin.AddLine3D(a(0), a(1), 0.0f, b(0), b(1), 0.0f, 0, 100, 50, 2.0f);
      }

      os.str("");
      os << "iteration: " << j << "  ncps: " << fit.m_nurbs.CVCount();
      dbgWin.SetLabel2D(txt_id[0], os.str(), 10, 5, h - 20, 0, 0, 0);
      os.str("");
      os << "avg error: " << avgerr << " / " << param.fitAvgError;
      dbgWin.SetLabel2D(txt_id[1], os.str(), 10, 5, h - 40, 0, 0, 0);
      os.str("");
      os << "max error: " << maxerr << " / " << param.fitMaxError;
      dbgWin.SetLabel2D(txt_id[2], os.str(), 10, 5, h - 60, 0, 0, 0);
      dbgWin.Update();

      //      dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Space);
    }

    fit.solve();
  }

  //  dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Return);

  // clamp to parametric domain of NURBS surface
  for (int i = 0; i < fit.m_nurbs.CVCount(); i++) {
    ON_3dPoint cv;
    fit.m_nurbs.GetCV(i, cv);

    if (cv.x < 0.0)
      cv.x = 0.0;
    if (cv.y < 0.0)
      cv.y = 0.0;
    if (cv.x > 1.0)
      cv.x = 1.0;
    if (cv.y > 1.0)
      cv.y = 1.0;
    cv.z = 0.0;

    fit.m_nurbs.SetCV(i, cv);
  }

  nurbs = fit.m_nurbs;
}

// ------------------------------- TDM ----------------------------------

void SurfaceBoundary::fitBoundaryNurbs_TDM(TomGine::tgTomGineThread &dbgWin, ON_NurbsCurve &nurbs,
    pcl::on_nurbs::NurbsDataCurve2d &data, FitBoundaryParameter &param)
{
  pcl::on_nurbs::NurbsDataCurve2d data_TDM = data;
  pcl::on_nurbs::NurbsDataCurve2d data_SDM = data;

  //  ON_NurbsCurve nurbs = FittingCurve2dAPDM::initNurbsCurve2D(param.order, data.interior);
  pcl::on_nurbs::FittingCurve2dAPDM fit(&data, nurbs);
  pcl::on_nurbs::FittingCurve2dATDM fit_TDM(&data_TDM, nurbs);
  pcl::on_nurbs::FittingCurve2dASDM fit_SDM(&data_SDM, nurbs);

  int id[3];
  int txt_id[3];
  ostringstream os;
  os.str("");
  int h = dbgWin.GetCamera().GetHeight();
  int w = dbgWin.GetCamera().GetWidth();
  dbgWin.ClearLabels();
  txt_id[0] = dbgWin.AddLabel2D(" ", 10, 5, h - 20, 0, 0, 0);
  txt_id[1] = dbgWin.AddLabel2D(" ", 10, 5, h - 40, 0, 0, 0);
  txt_id[2] = dbgWin.AddLabel2D(" ", 10, 5, h - 60, 0, 0, 0);

  dbgWin.AddLabel2D("PDM", 10, w - 60, h - 20, 1.0, 0, 0);
  dbgWin.AddLabel2D("TDM", 10, w - 60, h - 40, 0, 0.8, 0);
  dbgWin.AddLabel2D("SDM", 10, w - 60, h - 60, 0, 0, 0.8);

  double avgerr(DBL_MAX);
  double maxerr(DBL_MAX);
  bool stop(false);
  for (unsigned j = 0; j < param.fitMaxSteps && !stop && !dbgWin.Stopped(); j++) {
    if (j > 0 && j <= param.refinement) {
      fit.refine();
      fit_TDM.refine();
      fit_SDM.refine();
    } else if (j > param.refinement + 1) {
      if (!(j % param.addCPsIteration) && ((unsigned) fit.m_nurbs.CVCount() < param.maxCPs))
        fit.addCPsOnClosestPointViolation(param.addCPsAccuracy);
      if (!(j % param.addCPsIteration) && ((unsigned) fit_TDM.m_nurbs.CVCount() < param.maxCPs))
        fit_TDM.addCPsOnClosestPointViolation(param.addCPsAccuracy);
      if (!(j % param.addCPsIteration) && ((unsigned) fit_SDM.m_nurbs.CVCount() < param.maxCPs))
        fit_SDM.addCPsOnClosestPointViolation(param.addCPsAccuracy);
    }

    data.interior_param.clear();
    data_TDM.interior_param.clear();
    data_SDM.interior_param.clear();

    //    fit.assemble(param.paramFC);
    //    fit_TDM.assemble(param.paramFC_TDM);
    fit_SDM.assemble(param.paramFC_SDM);

    unsigned s = data_SDM.closest_points_error.size();
    avgerr = 0.0;
    maxerr = 0.0;
    for (unsigned i = 0; i < s; i++) {
      double &e = data_SDM.closest_points_error[i];
      avgerr += (e / s);
      if (e > maxerr) {
        maxerr = e;
      }
    }
    maxerr = sqrt(maxerr);

    if (j > param.refinement)
      stop = (param.fitMaxError > maxerr && param.fitAvgError > avgerr);

    // Visualisation
    {
      TomGine::tgRenderModel model[3];
      //      Triangulation::Convert(fit.m_nurbs, model[0], 8, true);
      //      Triangulation::Convert(fit_TDM.m_nurbs, model[1], 8, true);
      Triangulation::Convert(fit_SDM.m_nurbs, model[2], 8, true);
      //      model[0].m_line_width = 3.0;
      //      model[0].m_line_color = TomGine::vec3(1.0, 0.0, 0.0);
      //      model[0].m_point_color = TomGine::vec3(1.0, 0.0, 0.0);
      //      model[1].m_line_width = 3.0;
      //      model[1].m_line_color = TomGine::vec3(0.0, 0.8, 0.0);
      //      model[1].m_point_color = TomGine::vec3(0.0, 0.8, 0.0);
      model[2].m_line_width = 3.0;
      model[2].m_line_color = TomGine::vec3(0.0, 0.0, 0.8);
      model[2].m_point_color = TomGine::vec3(0.0, 0.0, 0.8);
      if (j == 0) {
        //        id[0] = dbgWin.AddModel(model[0]);
        //        id[1] = dbgWin.AddModel(model[1]);
        id[2] = dbgWin.AddModel(model[2]);
      } else {
        //        dbgWin.SetModel(id[0], model[0]);
        //        dbgWin.SetModel(id[1], model[1]);
        dbgWin.SetModel(id[2], model[2]);
      }

      dbgWin.ClearLines3D();
      for (unsigned i = 0; i < data_SDM.interior_line_start.size() && i
          < data_SDM.interior_line_end.size(); i++) {
        Eigen::Vector2d &a = data_SDM.interior_line_start[i];
        Eigen::Vector2d &b = data_SDM.interior_line_end[i];
        dbgWin.AddLine3D(a(0), a(1), 0.0f, b(0), b(1), 0.0f, 0, 100, 0, 2.0f);
      }

      os.str("");
      os << "iteration: " << j << "  ncps: " << fit.m_nurbs.CVCount() << " "
          << fit_TDM.m_nurbs.CVCount() << " " << fit_SDM.m_nurbs.CVCount();
      dbgWin.SetLabel2D(txt_id[0], os.str(), 10, 5, h - 20, 0, 0, 0);
      os.str("");
      os << "avg error: " << avgerr << " / " << param.fitAvgError;
      dbgWin.SetLabel2D(txt_id[1], os.str(), 10, 5, h - 40, 0, 0, 0);
      os.str("");
      os << "max error: " << maxerr << " / " << param.fitMaxError;
      dbgWin.SetLabel2D(txt_id[2], os.str(), 10, 5, h - 60, 0, 0, 0);
      dbgWin.Update();

//      printf("[SurfaceBoundary::fitBoundaryNurbs_TDM] iteration: %d\n", j);
      //      dbgWin.WaitForEvent(TomGine::TMGL_Press, TomGine::TMGL_Space);
    }

    //    fit.solve();
    //    fit_TDM.solve();
    fit_SDM.solve();
  }

  // clamp to parametric domain of NURBS surface
  for (int i = 0; i < fit.m_nurbs.CVCount(); i++) {
    ON_3dPoint cv;
    fit.m_nurbs.GetCV(i, cv);

    if (cv.x < 0.0)
      cv.x = 0.0;
    if (cv.y < 0.0)
      cv.y = 0.0;
    if (cv.x > 1.0)
      cv.x = 1.0;
    if (cv.y > 1.0)
      cv.y = 1.0;
    cv.z = 0.0;

    fit.m_nurbs.SetCV(i, cv);
  }

  nurbs = fit_SDM.m_nurbs;
}
