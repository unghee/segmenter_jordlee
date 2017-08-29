# Segmenter

ROS wrapper for RGBD segmentation.[![Build Status](https://travis-ci.com/JordLee/segmenter_jordlee.svg?token=yq1ky2pprZ86pCPy1qX5&branch=master)](https://travis-ci.com/JordLee/segmenter_jordlee)

## Description
This package is a ROS wrapper that performs segmentation based on 
[RGBD segmentation](http://www.acin.tuwien.ac.at/forschung/v4r/software-tools/rgb-d-segmentation/) by Richtsfeld et al. 
This package can be used in junction with existing grasping system, i.e., [fetch_grasp_suggestion](https://github.com/fetchrobotics/fetch_grasp_suggestion).
This package also includes functionality to train Support Vector Machine model using libsvm. 

## Installation
This package was tested on Ubuntu 14.04 
1. Install ROS-indigo. Installation instructions can be found [here](http://wiki.ros.org/indigo/Installation/Ubuntu). Full Desktop installation is recommended.
2. Install the package by cloning the `segmenter_jordlee` repository into your catkin workspace and building it as follows:
   ```bash
   cd (your catkin workspace)/src
   git clone https://github.com/fetchrobotics/sandbox/segmenter_jordlee.git
   cd ..
   catkin_make install
   ```   
## Usage
### segmenter
1. change model_path directory and string point_cloud_topic into your local directory in 'Segmenter::Segmenter' in src/segmenter_jordlee.cpp
  
2. ```bash
    cd (your catkin workspace)
    source devel/setup.bash 
    export ROS_MASTER_URI=http://<robot_name_or_ip>:11311
    ```
    
run rosnode  
3. ```bash
    rosrun segmenter_jordlee segmenter_jordlee
   ```  
   
open a new terminal. call rosservice. (repeat step 1, 2 before rosservice call)   
4. ```bash
   rosservice call /segment_object "{}" > /dev/null
   ```  
   
5. Visualization in Rviz
    ```bash
   rosrun rviz rviz
   ```  
   
   change frame to base_link. Pointcloud before segmenting will be under topic sensor_msgs/PointCloud2/pointtestinginput.  
   Segmented objects will be under topic visualization_msgs::MarkerArray/markers_jordlee
   
### svm_model_creator
Currently, the segmenter uses 'PP-Trainingsset.txt.scaled.model' as a default SVM model. This is trained by [Object Segmentation Database](http://users.acin.tuwien.ac.at/arichtsfeld/?site=4).
  User can change this by using their own dataset. [mOSD](https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD) trained data set is provided as an example in the src/svm_model_creator.cpp
1. prepare own object data set 
2. ```bash
       cd (your catkin workspace)/devel/lib/segmenter_jordlee
       ./svm_model creator -f ~/path_to_your_objectdatasets/datasets%1d.pcd -idx 0(number of your current train set)
   ```   
   
3. press F10 once TomGine window pops up. 
4. Go to src/svm_model_creator.cpp
5. go to 'Segmenter::annotator' function.
   - anno[number of surfacepatch of interest].pushback(numbering of surface patch next to it that is within same object)
   - repeat this for all surfaces except the floor and surfaces which are extremely small
   - move to next surface 
   This annotation handles double counting so you do not need to count it twice (i.e., anno[1].pushback(2), no need to anno[2].pushback(1))
6. when you finished the annotation for all the datasets, 

    ```bash
   ./svm_model creator -f ~/path_to_your_objectdatasets/datasets%1d.pcd -idx 0 10(total count of your training data set)
   ``` 
   
   this will generate 'model.txt' in '/devel/lib/segmenter_jordlee'
7. move this model.txt to model folder in your src/segmenter_jordlee/model
8. change of variable name is segmenter_jordlee.cpp for using this model. 

## Primary ROS Nodes
* **Subscribers**
  * `/head_camera/depth_registered/points`([pcl/PointCloud<pcl/PointXYZRGB>](http://wiki.ros.org/pcl_ros))  
  Point cloud stream used for updating the scene
  for segmentation.  The point cloud subscribed to can be changed by setting the `string point_cloud_topic` in constructor of 'Segmenter::Segmenter'.
* **Publishers**
  * `/segmented_objects`([rail_manipulation_msgs/SegmentedObjectList](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/SegmentedObjectList.msg))  
  segmented object list
  * `/pointstestinginput`([pcl/PointCloud<pcl/PointXYZRGB>](http://wiki.ros.org/pcl_ros))  
  pointcloud before segmenting. Used for visualization in rviz.
  * `/segmented_markers`([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))  
  visualization markers in rviz
* **Service Clients**
   `/segment_object([rail_manipulation_msgs/SegmentedObjectList](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/SegmentedObjectList.msg))  
  segmented object list. This has a same type with the segmented object list in the publisher. So the same message types are both published and saved in the service response.
  Users can remove one of these for their own need. 
  
## Connecting with grasping system
This version currently works with [fetch_grasp_suggestion](https://github.com/fetchrobotics/fetch_grasp_suggestion),
 which subscribes [rail_segmentation/SegmentedObjectList](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/SegmentedObjectList.msg). 
 Users may change the type of the message for their grasping system.
Please refer to [connecting with alternative segmenter](https://github.com/fetchrobotics/fetch_grasp_suggestion/tree/master/fetch_grasp_suggestion#connecting-alternative-object-segmentation)
 for detailed instruction.


## Authors
### ROS Wrapper
Ung Hee(Jordan) Lee  
Mechanical Engineering  
University of Michigan  
unghee@umich.edu

### RGB-D Segmentation
Dipl.-Ing. Andreas Richtsfeld  
Automation and Control Institute (ACIN)  
Vienna University of Technology  
Gusshausstraße 25-29
1040 Vienna  
ari(at)acin.tuwien.ac.at

## Citation
1. Richtsfeld A., Mörwald T., Prankl J., Zillich M. and Vincze M. - Segmentation of Unknown Objects in Indoor Environments.  IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012 (pdf)
2. Chih-Chung Chang and Chih-Jen Lin, LIBSVM : a library for support vector machines.
ACM Transactions on Intelligent Systems and Technology, 2:27:1--27:27, 2011. Software available at http://www.csie.ntu.edu.tw/~cjlin/libsvm
3. [strands-project v4r library](https://github.com/strands-project/v4r)
4. [Modified Object Segmented Datasets](https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD)
## Developer note
The speed of running the package decreased after including and building from source file on_nurbs opennurbs from [strands-project v4r library](https://github.com/strands-project/v4r)
instead of using pcl 1.7.2. For now, easy solution would be install the pcl 1.7.2. and connect the header files to the pcl instead of this stand alone on_nurbs opennurbs. 