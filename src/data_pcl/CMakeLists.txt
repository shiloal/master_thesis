cmake_minimum_required(VERSION 2.8.3)
project(data_pcl)

find_package(catkin REQUIRED
    COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs)

find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES data_pcl
  CATKIN_DEPENDS pcl_ros
    		roscpp
    		sensor_msgs 
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#add_executable(octree pc.cpp)
add_executable(pc_kdtree pckdtree.cpp)
add_executable(ransac_plane ransac_plane.cpp)
add_executable(normal_plane normal_plane.cpp)
add_executable(region_grow_segm region_grow_segm.cpp)
add_executable(normals normals.cpp)
add_executable(normals_smoothing normals_smoothing.cpp)

#target_link_libraries(octree ${catkin_LIBRARIES})
target_link_libraries(pc_kdtree ${catkin_LIBRARIES})
target_link_libraries(ransac_plane ${catkin_LIBRARIES})
target_link_libraries(normal_plane ${catkin_LIBRARIES})
target_link_libraries(region_grow_segm ${catkin_LIBRARIES})
target_link_libraries(normals ${catkin_LIBRARIES})
target_link_libraries(normals_smoothing ${catkin_LIBRARIES})


