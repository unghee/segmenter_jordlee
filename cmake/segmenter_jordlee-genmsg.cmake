# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "segmenter_jordlee: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Irail_manipulation_msgs:/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/indigo/share/visualization_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(segmenter_jordlee_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv" NAME_WE)
add_custom_target(_segmenter_jordlee_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmenter_jordlee" "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv" "rail_manipulation_msgs/SegmentedObjectList:geometry_msgs/Vector3:geometry_msgs/Quaternion:sensor_msgs/PointField:rail_manipulation_msgs/Grasp:rail_manipulation_msgs/SegmentedObject:sensor_msgs/PointCloud2:visualization_msgs/Marker:std_msgs/Header:sensor_msgs/Image:std_msgs/ColorRGBA:geometry_msgs/Pose:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(segmenter_jordlee
  "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObjectList.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/Grasp.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObject.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmenter_jordlee
)

### Generating Module File
_generate_module_cpp(segmenter_jordlee
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmenter_jordlee
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(segmenter_jordlee_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(segmenter_jordlee_generate_messages segmenter_jordlee_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv" NAME_WE)
add_dependencies(segmenter_jordlee_generate_messages_cpp _segmenter_jordlee_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmenter_jordlee_gencpp)
add_dependencies(segmenter_jordlee_gencpp segmenter_jordlee_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmenter_jordlee_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(segmenter_jordlee
  "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObjectList.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/Grasp.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObject.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmenter_jordlee
)

### Generating Module File
_generate_module_lisp(segmenter_jordlee
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmenter_jordlee
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(segmenter_jordlee_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(segmenter_jordlee_generate_messages segmenter_jordlee_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv" NAME_WE)
add_dependencies(segmenter_jordlee_generate_messages_lisp _segmenter_jordlee_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmenter_jordlee_genlisp)
add_dependencies(segmenter_jordlee_genlisp segmenter_jordlee_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmenter_jordlee_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(segmenter_jordlee
  "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObjectList.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/Grasp.msg;/opt/ros/indigo/share/rail_manipulation_msgs/cmake/../msg/SegmentedObject.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmenter_jordlee
)

### Generating Module File
_generate_module_py(segmenter_jordlee
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmenter_jordlee
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(segmenter_jordlee_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(segmenter_jordlee_generate_messages segmenter_jordlee_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fetch/catkin_ws/src/segmenter_jordlee/srv/SegmentObject.srv" NAME_WE)
add_dependencies(segmenter_jordlee_generate_messages_py _segmenter_jordlee_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmenter_jordlee_genpy)
add_dependencies(segmenter_jordlee_genpy segmenter_jordlee_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmenter_jordlee_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmenter_jordlee)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmenter_jordlee
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(segmenter_jordlee_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET rail_manipulation_msgs_generate_messages_cpp)
  add_dependencies(segmenter_jordlee_generate_messages_cpp rail_manipulation_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmenter_jordlee)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmenter_jordlee
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(segmenter_jordlee_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET rail_manipulation_msgs_generate_messages_lisp)
  add_dependencies(segmenter_jordlee_generate_messages_lisp rail_manipulation_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmenter_jordlee)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmenter_jordlee\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmenter_jordlee
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(segmenter_jordlee_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET rail_manipulation_msgs_generate_messages_py)
  add_dependencies(segmenter_jordlee_generate_messages_py rail_manipulation_msgs_generate_messages_py)
endif()
