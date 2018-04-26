#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>           //PCD读写类相关的头文件
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <boost/shared_ptr.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("viewer"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer("viewer1"));

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5000; //设置点云宽度或数量，这里为数量
  cloud->height = 1; //设置点云高度或标准其为无序点云
  cloud->points.resize (cloud->width * cloud->height);
  //为点云填充数据
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("random.pcd", *cloud);

  //打印所有点到标准错误输出
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ>pass;     //设置滤波器对象
  pass.setInputCloud(cloud);                //设置输入点云
  pass.setFilterFieldName("z");             //设置过滤时所需要点云类型的z字段
  pass.setFilterLimits(0.0,10.0);           //设置在过滤字段上的范围
  //pass.setFilterLimitsNegative (true);     //设置保留范围内的还是过滤掉范围内的
  pass.filter(*cloud_filtered);              //执行滤波，保存过滤结果在cloud_filtered

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  viewer->addPointCloud(cloud, "map");
  viewer->setCameraPosition(0, 0, -2, 0, -1, 1, 0);

  while (!viewer->wasStopped())
  {
     viewer->spinOnce(100);
     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  viewer1->addPointCloud(cloud_filtered, "map1");
  viewer1->setCameraPosition(0, 0, -2, 0, -1, 1, 0);

  while (!viewer1->wasStopped())
  {
     viewer1->spinOnce(100);
     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return (0);
}
