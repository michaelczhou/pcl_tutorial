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
find_package(PCL REQUIRED )#COMPONENT common io visualization kdtree filters )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable( select src/select.cpp )
target_link_libraries( select  ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( point_viewer src/point_viewer.cpp )
target_link_libraries( point_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( read_pcd src/read_pcd.cpp )
target_link_libraries( read_pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( write_pcd src/write_pcd.cpp )
target_link_libraries( write_pcd  ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( modify_pcd src/modify_pcd.cpp )
target_link_libraries( modify_pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
add_executable( ndt src/normal_distributions_transform.cpp )
target_link_libraries( ndt  ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} )
add_executable( icp src/iterative_closest_point.cpp )
target_link_libraries( icp  ${PCL_LIBRARIES}  )
add_executable( passthrough src/passthrough.cpp )
target_link_libraries( passthrough ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} )
add_executable( remove_outliers src/remove_outliers.cpp)
target_link_libraries( remove_outliers ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} )
add_executable (pir src/pairwise_incremental_registration.cpp)
target_link_libraries (pir ${PCL_LIBRARIES})

