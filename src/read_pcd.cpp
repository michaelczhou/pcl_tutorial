#include <iostream>           //标准C++库中的输入输出类相关头文件。
#include <pcl/io/pcd_io.h>   //pcd 读写类相关的头文件。
#include <pcl/point_types.h> //PCL中支持的点类型头文件。

//创建一个PointCloud<pcl::PointXYZ>    boost共享指针并进行实例化
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

int main()
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zc/project/pcl_tutorial/parking_filtered.pcd",*cloud)==-1)//打开点云文件
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    //默认就是而二进制块读取转换为模块化的PointCLoud格式里pcl::PointXYZ作为点类型  然后打印出来
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
//    sensor_msgs::PointCloud2 cloud_blob;  //PointCloud2适合版本低的点云文件
//    pcl::io::loadPCDFile("/home/zc/project/pcl_tutorial/parking_filtered.pcd",cloud_blob);
//    pcl::fromROSMsg(cloud_blob,*cloud);
//    //* sensor_msgs/PointCloud2 转换为 pcl::PointCloud<T>

    for(size_t i=0; i < cloud->points.size(); ++i)
        std::cout<<"    "<<cloud->points[i].x
                 <<" "<<cloud->points[i].y
                 <<" "<<cloud->points[i].z << std::endl;
    std::cout << "point_num = " << cloud->points.size() << std::endl;
    return(0);
}

