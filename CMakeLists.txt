cmake_minimum_required( VERSION 2.8 )
project(pcl_tutorial)

set( CMAKE_CXX_FLAGS "-std=c++11" )

# opencv
#set (OpenCV_DIR "/usr/local/share/OpenCV")
find_package( OpenCV 2.4.13 REQUIRED  HINTS /home/zc/tools/opencv-2.4.13/ )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

#boost
find_package(Boost REQUIRED  )
include_directories(${Boost_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED COMPONENT common io visualization kdtree )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions(${PCL_DEFINITIONS})


add_executable( select src/select.cpp )
target_link_libraries( select ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( point_viewer src/point_viewer.cpp )
target_link_libraries( point_viewer ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
