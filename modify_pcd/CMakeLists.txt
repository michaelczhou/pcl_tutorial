cmake_minimum_required( VERSION 2.8 )
project(pointSelection)

set( CMAKE_CXX_FLAGS "-std=c++11" )

# opencv
set (OpenCV_DIR "/usr/local/share/OpenCV")
find_package( OpenCV 3.2 REQUIRED )
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

add_executable( modify_pcd modify_pcd.cpp )
target_link_libraries( modify_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )

