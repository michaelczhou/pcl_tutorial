#if 1
#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <boost/shared_ptr.hpp>

//using namespace std;
//using namespace pcl;

int main(int argc,char* argv[] )
{
    //cloud_mutex.lock();    // for not overwriting the point cloud
    // 创建点云（指针）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //* 读入PCD格式的文件，如果文件不存在，返回-1
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zc/Desktop/shandian/map.pcd", *cloud) == -1)
    {
       //文件不存在时，返回错误，终止程序。
       PCL_ERROR("Couldn't read file test_pcd.pcd \n");
       return -1;
    }
    //直接创造一个显示窗口
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    //再这个窗口显示点云
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
       //viewer.spinOnce(100);
       //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}
#endif
