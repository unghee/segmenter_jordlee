/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file Segmenter.h
 * @author Andreas Richtsfeld
 * @date July 2012
 * @version 0.1
 * @brief Segment images
 */


#ifndef SEGMENT_SEGMENTER_H
#define SEGMENT_SEGMENTER_H

#include <stdio.h>

#include "v4r/KinectInterface/KinectData.h"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceClustering/ClusterNormalsToPlanes.hh"
#include "v4r/SurfaceClustering/ZAdaptiveNormals.hh"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"
#include "v4r/SurfaceRelations/PatchRelations.h"
#include "v4r/svm/SVMPredictorSingle.h"
#include "v4r/svm/SVMFileCreator.h"
#include "v4r/GraphCut/GraphCut.h"
#include "v4r/ObjectModeling/ContourRefinement.h"
#include "v4r/ObjectModeling/CreateMeshModel.hh"

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>

//ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//ROS srv
#include "segmenter_jordlee/SegmentObject.h"
namespace segment
{
  
/**
 * @class Segmenter
 */
class Segmenter
{
private:
  
  bool useStructuralLevel;                                ///< Use structural level svm
  bool useAssemblyLevel;                                  ///< Use assembly
  
  double z_min, z_max;                                    ///< Minimum and maximum z-values

  std::string database_path;                              ///< database path
  std::string rgbd_filename;                              ///< depth image filename
  std::string kinect_config;                              ///< kinect configuration file
  std::string model_path;                                 ///< path to the svm model and scaling files

  unsigned startIdx, endIdx;                              ///< Start and end index of images
  bool data_live;                                         ///< load data live from Kinect
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;       ///< original pcl point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;              ///< Normals of the point cloud
  std::vector<surface::SurfaceModel::Ptr > surfaces;      ///< Surfaces container (for Planes, NURBS)
  std::vector< std::vector<unsigned> > graphCutGroups;    ///< Graph cut groups of surface patch models

  KinectData *kinect;                                     ///< Load kinect data from file or live from Kinect
  surface::ClusterNormalsToPlanes *clusterNormals;        ///< Cluster normals to planes
  surface::SurfaceModeling *surfModeling;                 ///< Surface modeling (plane, NURBS, model selection)
  surface::PatchRelations *patchRelations;                ///< Calculatie the relations between surface patches
  svm::SVMPredictorSingle *svm1st;                        ///< SVM-predictor for structural level
  svm::SVMPredictorSingle *svm2nd;                        ///< SVM-predictor for assembly level
  gc::GraphCut *graphCut;                                 ///< Graph cut
  surface::SaveFileSequence *resultSaver;                 ///< Save segmented object models to sfv file


//  boost::mutex pc_mutex_;                                 ///< Mutex for locking on the point cloud and messages
  ros::NodeHandle nh;                                  ///< ROS Node handles
  ros::Subscriber sub;                                    ///< ROS subscirber
  ros::ServiceServer segment_srv_;
  ros::Publisher pub,segmented_objects_pub_, markers_pub_;
  /*! Latest point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_;
  visualization_msgs::MarkerArray markers_;
  rail_manipulation_msgs::SegmentedObjectList object_list_;
  /*! List of segmentation zones. */
//  std::vector<SegmentationZone> zones_;
  /*! The transform tree buffer for the tf2 listener. */
//  tf2_ros::Buffer tf_buffer_;
  /*! Main transform listener. */
 // tf::TransformListener tf_;

public:
    static const float DOWNSAMPLE_LEAF_SIZE = 0.01;
    /*! Size of the marker visualization scale factor. */
    static const double MARKER_SCALE = 0.01;




private:
  void init();
  void process();
  
public:
  Segmenter(std::string _db = "/media/Daten/OSD-0.2/",
            std::string _rgbd = "pcd/test%1d.pcd",
            std::string _model = "model/",
            bool _live = false);
  ~Segmenter();
  
  /** Use assembly level for processing **/
  void setAssemblyLevel(bool _on) {useAssemblyLevel = _on;}
  
  /** Set minimum and maximum depth values **/
  void setMinMaxDepth(double _z_min, double _z_max) {z_min = _z_min; z_max = _z_max;}
  
  /** Process a point cloud and return labeled cloud **/
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
    processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Process a point cloud and return vector of segment indices **/
  std::vector<pcl::PointIndices> 
    processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Run the segmenter **/
//  void run(std::string _rgbd_filename, std::string _model_path, int _startIdx, int _endIdx);
    void run(std::string _model_path);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    rgbVis(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud,  std::vector<pcl::PointIndices> label);



  /** Ros related **/
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    bool SegmentObjectCallback(segmenter_jordlee::SegmentObject::Request &req, segmenter_jordlee::SegmentObject::Response &res);
    visualization_msgs::Marker createMarker(const pcl::PCLPointCloud2::ConstPtr &pc) const;
};

}

#endif
