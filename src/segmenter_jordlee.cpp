
//pcl
//#include <iostream>
//#include <pcl/point_types.h>

#include "Segmenter1.h"

//#include <pcl/segmentation/region_growing.h>

/* --------------- Segmenter --------------- */

//Segmenter::Segmenter segmenter;
namespace segment
{

Segmenter::Segmenter(std::string _db, std::string _rgbd, std::string _model, bool _live)
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = _db;
  rgbd_filename = _rgbd;
  //model_path = _model;
  model_path = "/home/fetch/catkin_ws/src/segmenter_jordlee/model/";
  data_live = _live;
  startIdx = 0;
  endIdx = 65;
  useStructuralLevel = true;
  useAssemblyLevel = false;

  //ros subscribing-
  string point_cloud_topic("/head_camera/depth_registered/points");


  sub = nh.subscribe(point_cloud_topic, 1, &Segmenter::pointCloudCallback,this);
  segment_srv_ = nh.advertiseService("segment_object", &Segmenter::SegmentObjectCallback, this);
  pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("pointstestinginput", 1,true);
  markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("markers_jordlee", 1, true);
 // segmented_objects_pub_ = nh.advertise<std_msgs::String>("segmented_objects", 1, true);
 // segmented_objects_pub_ = nh.advertise<rail_manipulation_msgs::SegmentedObjectList>("segmented_objects", 1, true);
 // segmented_objects_pub_ = nh.advertise<rail_manipulation_msgs::SegmentedObjectList>("segmented_objects", 1, true);
//  segmented_objects_pub_ = nh.advertise<rail_manipulation_msgs::SegmentedObject>("segmented_objects", 1, true);

  ROS_INFO("Ready to segment.");
 //ros::spin();

}
Segmenter::~Segmenter()
{
  delete kinect;
  delete surfModeling;
  delete patchRelations;
  delete svm1st;
  delete svm2nd;
  delete graphCut;
  delete resultSaver;
}

void Segmenter::init()
{

  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  std::string svmStructuralModel = model_path + "PP-Trainingsset.txt.scaled.model";
  std::string svmStructuralScaling = model_path + "param.txt";
  std::string svmAssemblyModel = model_path + "PP2-Trainingsset.txt.scaled.model";
  std::string svmAssemblyScaling = model_path + "param2.txt";


  surface::ClusterNormalsToPlanes::Parameter param;
  param.adaptive = true;         // use adaptive thresholds
  clusterNormals = new surface::ClusterNormalsToPlanes(param);

  // init nurbsfitting & model-selection

  pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16;
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.planePointsFixation = 8000;       // 8000
  sfmParams.z_max = 0.01;
  surfModeling = new surface::SurfaceModeling(sfmParams);
  surfModeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling->setExtrinsic(pose);

  // init patch relation class
  patchRelations = new surface::PatchRelations();
  patchRelations->setStructuralLevel(useStructuralLevel);
  patchRelations->setAssemblyLevel(useAssemblyLevel);

  // init svm-predictor
  svm1st = new svm::SVMPredictorSingle(svmStructuralModel.c_str());
  svm2nd = new svm::SVMPredictorSingle(svmAssemblyModel.c_str());
  svm1st->setScaling(true, svmStructuralScaling.c_str());
  svm2nd->setScaling(true, svmAssemblyScaling.c_str());

  // init graph cutter
  graphCut = new gc::GraphCut();

  // save results to sfv file
  resultSaver = new surface::SaveFileSequence();
  resultSaver->InitFileSequence("result_%1d.sfv", 0, 1000);

}

void Segmenter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);
  std::cout<< "subscribing pointcloud "<< std::endl;
  pc_=cloud;

}


std::vector<pcl::PointIndices>
Segmenter::processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
 // pclA::FilterZ(pcl_cloud, z_min, z_max);

  // calcuate normals
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter param;
  param.adaptive = true;
  pclA::ZAdaptiveNormals nor(param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);

  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);

  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(surfaces, false);
  printf("surfmodeling\n");
  std::vector<surface::Relation> relation_vector;
  patchRelations->setInputCloud(pcl_cloud);
  printf("setinputcloud\n");
  patchRelations->setSurfaceModels(surfaces);
  printf("setSurfaceModels\n");
  patchRelations->setOptimalPatchModels(true);
  printf("OptimalPatchModel\n");
  patchRelations->computeSegmentRelations();
  printf("computeSegmentRelation\n");
  patchRelations->getRelations(relation_vector);
  printf("getrelation\n");
  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st->getResult(relation_vector[i].type,
                                                        relation_vector[i].rel_value,
                                                        relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd->getResult(relation_vector[i].type,
                                                        relation_vector[i].rel_value,
                                                        relation_vector[i].rel_probability);
  }
//githubtest
  graphCutGroups.clear();
  graphCut->init(surfaces.size(), relation_vector);
  graphCut->process();
  graphCut->getResults(surfaces.size(), graphCutGroups);
//  graphCut->printResults();

  std::vector<pcl::PointIndices> results;
  results.resize(graphCutGroups.size());
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
      for(unsigned k=0; k<surfaces[graphCutGroups[i][j]]->indices.size(); k++)
        results[i].indices.push_back(surfaces[graphCutGroups[i][j]]->indices[k]);
  return results;

}


//  void Segmenter::run(std::string _rgbd_filename,std::string _model_path, int _startIdx, int _endIdx)
//  void Segmenter::run(std::string _model_path)

bool Segmenter::SegmentObjectCallback(segmenter_jordlee::SegmentObject::Request &req, segmenter_jordlee::SegmentObject::Response &res)
{

  printf("init.\n");
  init();


  std::cout<<" processing pointcloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZRGB>);

/*
  // ######################## Setup TomGine ########################
  int width = 640;
  int height = 480;
  surface::View view;

  TomGine::tgTomGineThread dbgWin(width, height, "TomGine Render Engine");
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Vec3d rotCenter(0, 0, 1.0);

  cv::Mat intrinsic;
  intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  view.intrinsic = Eigen::Matrix3d::Zero();
  intrinsic.at<double> (0, 0) = intrinsic.at<double> (1, 1) = view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
  intrinsic.at<double> (0, 2) = view.intrinsic(0, 2) = 320;
  intrinsic.at<double> (1, 2) = view.intrinsic(1, 2) = 240;
  intrinsic.at<double> (2, 2) = view.intrinsic(2, 2) = 1.;

  dbgWin.SetClearColor(0.5, 0.5, 0.5);
  dbgWin.SetCoordinateFrame();
  dbgWin.SetCamera(intrinsic);
  dbgWin.SetCamera(R, t);
  dbgWin.SetRotationCenter(rotCenter);
  dbgWin.Update();
  cv::Mat_<cv::Vec3b> kImage = cv::Mat_<cv::Vec3b>::zeros(480, 640);

  pcl::copyPointCloud(*pc_, *cloud_input);
  pclA::ConvertPCLCloud2Image(cloud_input, kImage);
  cv::imshow("Debug image", kImage);
  dbgWin.SetImage(kImage);
  dbgWin.Update();
*/

  //loading pointcloud from pcd
  // pcl::io::loadPCDFile (rgbd_filename, *cloud_input);
  //save pointcloud to pcd
  //pcl::io::savePCDFileASCII("/home/fetch/catkin_ws/src/segmenter_jordlee/testsegment.pcd",*cloud_input);

  std::cout<<"copying pointcloud"<<std::endl;
  pcl::copyPointCloud(*pc_, *cloud_input);

  std::cout<<"after copying pointcloud"<<std::endl;
  std::vector<pcl::PointIndices> label_indices;
  label_indices = processPointCloudV(cloud_input);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud_output;
  std::cout<<"before"<<std::endl;
  for(int i =0;i<label_indices.size();i++)
  {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_input, label_indices[i], *(cloud_temp));
    cloud_output.push_back(cloud_temp);

  }
  std::cout<<"after"<<std::endl;

  
//rviz

  cloud_input->header.frame_id = "head_camera_depth_optical_frame";
//  cloud_output[1]->header.stamp = ros::Time::now().toNSec();

  pub.publish (cloud_input);
  ros::spinOnce ();

  for(int i =0;i<label_indices.size();i++)
  {
    pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_output[i], *converted);

    rail_manipulation_msgs::SegmentedObject segmented_object;
    pcl_conversions::fromPCL(*converted, segmented_object.point_cloud);

    segmented_object.point_cloud.header.stamp = ros::Time::now();
    segmented_object.marker = this->createMarker(converted);
    segmented_object.marker.id = i;

    markers_.markers.push_back(segmented_object.marker);

    object_list_.objects.push_back(segmented_object);
 //   segmented_objects_pub_.publish(segmented_object);
  }

  //publish markers
  markers_pub_.publish(markers_);

  // save into object_list
  object_list_.header.seq++;
  object_list_.header.stamp = ros::Time::now();
 // object_list_.header.frame_id = zone.getSegmentationFrameID();
  object_list_.header.frame_id = "test";
  object_list_.cleared = false;

  //save it into response
  res.object_list_ = object_list_;
  ROS_INFO("sending back response: ");

  return true;


//pcl viewer
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  segment::Segmenter::rgbVis(cloud_output,label_indices);


  printf("[Segmenter::run] Done.\n");


  ROS_INFO("sending back response");
  return true;

}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
Segmenter::rgbVis (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud, std::vector<pcl::PointIndices> label)
  {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    std::vector<string> idname;
    idname.reserve(label.size());
    viewer->setBackgroundColor (0, 0, 0);

    for(int i=0;i<label.size();i++)
    {

      string s = boost::lexical_cast<string>(i);
      idname.push_back(s);
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> rgb(cloud[i]);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud[i], rgb, idname[i]);

    }
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (viewer);
  }

visualization_msgs::Marker Segmenter::createMarker(const pcl::PCLPointCloud2::ConstPtr &pc) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = pc->header.frame_id;

  // default position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // default scale
  marker.scale.x = MARKER_SCALE;
  marker.scale.y = MARKER_SCALE;
  marker.scale.z = MARKER_SCALE;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color.a = 1.0;

  // downsample point cloud for visualization
  pcl::PCLPointCloud2 downsampled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  voxel_grid.setInputCloud(pc);
  voxel_grid.setLeafSize(DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE);
  voxel_grid.filter(downsampled);

  // convert to an easy to use point cloud message
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(downsampled, pc2_msg);
  sensor_msgs::PointCloud pc_msg;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);

  // place in the marker message
  marker.points.resize(pc_msg.points.size());
  int r = 0, g = 0, b = 0;
  for (size_t j = 0; j < pc_msg.points.size(); j++)
  {
    marker.points[j].x = pc_msg.points[j].x;
    marker.points[j].y = pc_msg.points[j].y;
    marker.points[j].z = pc_msg.points[j].z;

    // use average RGB
    uint32_t rgb = *reinterpret_cast<int *>(&pc_msg.channels[0].values[j]);
    r += (int) ((rgb >> 16) & 0x0000ff);
    g += (int) ((rgb >> 8) & 0x0000ff);
    b += (int) ((rgb) & 0x0000ff);
  }

  // set average RGB
  marker.color.r = ((float) r / (float) pc_msg.points.size()) / 255.0;
  marker.color.g = ((float) g / (float) pc_msg.points.size()) / 255.0;
  marker.color.b = ((float) b / (float) pc_msg.points.size()) / 255.0;
  marker.color.a = 1.0;

  return marker;
}


}

void printUsage(char *av)
{
    printf("Usage: %s [options] \n"
               " Options:\n"
               "   [-h] ... show this help.\n"
               "   [-f rgbd_filename] ... specify rgbd-image path and filename\n"
               "   [-m model_path] ... specify model path\n"
               "   [-idx start end] ... start and end index of files\n"
               "   [-l] ... live image from kinect\n"
               "   [-as usage] ... use assembly level: 0/1\n", av);
    std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -m model/ -idx 0 10" << std::endl;
}

int main(int argc, char **argv)
{

  std::string rgbd_filename = "points2/test%1d.pcd";
  std::string model_path = "/home/fetch/catkin_ws/src/segmenter_jordlee/model/";

  ros::init(argc, argv, "segmenter");

  segment::Segmenter seg;

  ros::spin();
  return 0;

}
