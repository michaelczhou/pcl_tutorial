#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;
std::vector< int> totalIndices;
bool myfind(int temp1,const vector<int>& temp)
{

for (int i = 0; i < temp.size(); i++)
{
if (temp1 == temp[i])
return true;
}

return false;
}
void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
std::vector< int > indices;
if (event.getPointsIndices(indices) == -1)
return;


for (int i = 0; i < indices.size(); ++i)
{
clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
totalIndices.push_back(indices[i]);
}
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
std::stringstream ss;
std::string cloudName;
ss << num++;
ss >> cloudName;
cloudName += "_cloudName";
viewer->addPointCloud(clicked_points_3d, red, cloudName);
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

int main()
{
if (pcl::io::loadPCDFile("/home/zc/aaa_temp/parking_filtered.pcd", *cloud))
{
std::cerr << "ERROR: Cannot open file " << std::endl;
return 0;
}
viewer->addPointCloud(cloud, "test");
viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
int i = 0;
while (!viewer->wasStopped())
{
if (i < 60)
{
viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(100000));
i++;
//print();
}
else
break;
//print();
}
////////////////////////////////②框选（要移除）的点云存在另外一个窗口viewer2中：
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
//pcl::PointCloud<pcl::PointXYZ>::Ptr OUTcloud(new pcl::PointCloud<pcl::PointXYZ>());
//pcl::copyPointCloud(*cloud, totalIndices, *OUTcloud);
//viewer2->addPointCloud(OUTcloud, "test2");
//viewer2->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
//while (!viewer2->wasStopped())
//{


// viewer2->spinOnce(100);
// boost::this_thread::sleep(boost::posix_time::microseconds(100000));


//}
////////////////////////////////③移除后剩余的点云存在另外一个窗口viewer3中:
vector<int>FINALIndices;
for (int i = 0; i < 100000; i++)
{
if (myfind(i, totalIndices) == false)
FINALIndices.push_back(i);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("viewer3"));
pcl::PointCloud<pcl::PointXYZ>::Ptr FINALcloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::copyPointCloud(*cloud, FINALIndices, *FINALcloud);
viewer3->addPointCloud(FINALcloud, "test3");
viewer3->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
while (!viewer3->wasStopped())
{
viewer3->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(100000));

}

return 0;
}
