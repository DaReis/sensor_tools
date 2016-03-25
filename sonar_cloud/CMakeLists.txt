cmake_minimum_required(VERSION 2.8.3)
project(sonar_cloud)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	tf 
	laser_geometry 
	pcl_ros
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp

)

###########
## Build ##
###########

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(sonar_cloud_assembler src/sonar_cloud_assembler.cpp)
target_link_libraries(sonar_cloud_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sonar_plant_filter src/sonar_plant_filter.cpp)
target_link_libraries(sonar_plant_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

