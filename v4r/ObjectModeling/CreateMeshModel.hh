/**
 * $Id$
 */

#ifndef P_CREATE_MESH_MODEL_HH
#define P_CREATE_MESH_MODEL_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceUtils/Utils.hh"
#include "v4r/PCLAddOns/PCLUtils.h"

#include "Triangulation.h"

namespace objectmodeling
{

  /**
   * Create a mesh model from planes and nurbs for visualization
   */
  class CreateMeshModel
  {
  public:
    class Parameter
    {
    public:
      double max_concavity;
      unsigned resolution;
      Parameter (double _max_concavity = 1., unsigned _resolution = 8) :
        max_concavity (_max_concavity), resolution (_resolution)
      {
      }
    };

  private:
    int width, height;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    Parameter param;

    void
    ProjectPointsToPlane (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<int> &indices,
                          const std::vector<float> &coeffs,
                          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &points);

    void
    ProjectPointsToPlane (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<int> &indices,
                          const std::vector<float> &coeffs,
                          std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &points);

    inline int
    GetIdx (short x, short y);
    inline short
    X (int idx);
    inline short
    Y (int idx);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CreateMeshModel (Parameter p = Parameter ());
    ~CreateMeshModel ();

    void
    setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

    void
    compute (std::vector<surface::SurfaceModel::Ptr> &_models);

    void
    computeUnTrimmed (surface::SurfaceModel::Ptr surface);

    void
    computeUntrimmed (surface::View &view);

    static void
    computeTrimmed (surface::SurfaceModel::Ptr surface, unsigned resolution=8);

    void
    computeTrimmed (surface::View &view);

  };

  /*********************** INLINE METHODES **************************/
  inline int
  CreateMeshModel::GetIdx (short x, short y)
  {
    return y * width + x;
  }

  inline short
  CreateMeshModel::X (int idx)
  {
    return idx % width;
  }

  inline short
  CreateMeshModel::Y (int idx)
  {
    return idx / width;
  }

}

#endif

