#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace pcl;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr lastcloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> new_viewer(new pcl::visualization::PCLVisualizer("new_viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);

int num = 0;
std::vector< int > indices;
vector<string> redpoint;



void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    std::vector< int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
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

void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args=NULL  )
{
    static int count=0;
    static int last_character_press=0;
    int character_press = event.getKeyCode();
    if(last_character_press != character_press && character_press != (int)'x')
    {
        cout<<"chara "<<character_press<<endl;
        cout<<(count++)<<endl;
        last_character_press = character_press;

        if(character_press==(int)'b')
        {
            pcl::copyPointCloud(*lastcloud,*cloud);
            viewer->removePointCloud("map");
            viewer->addPointCloud(cloud,"map");
        }

        else if(character_press==(int)'d')
        {
            cout<<"d"<<endl;

            sort(indices.begin(),indices.end());
            for (int i=0;i<indices.size();i++){
                cout<<indices[i]<<" ";
            }
            cout<<"ok"<<endl;
            pcl::copyPointCloud(*cloud,*lastcloud);
            pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
            pcl::PointCloud<pcl::PointXYZ>::iterator tmpindex;
            int deletecount=0;

            cout<<"delete done"<<endl;
            indices.clear();
            std::stringstream ss;
            std::string cloudName;
            cloudName += "_cloudName";
            cout<<"num"<<num<<endl;
            for(int i=0;i<redpoint.size();i++ ){
                viewer->removePointCloud(redpoint[i]);
                //cloudName="";
            }
            redpoint.clear();
            //num=0;
            viewer->removePointCloud("map");
            viewer->addPointCloud(cloud,"map");
            //viewer->removePointCloud("");


        }

        else if(character_press==(int)'s')
        {
            pcl::io::savePCDFileBinary("culledmap.pcd",*cloud);
            cout<<"save done"<<endl;
        }
    }
}

int main(int argc,char** argv)
{
	if (pcl::io::loadPCDFile("/home/zc/aaa_temp/parking_filtered.pcd", *cloud))
    {
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return 0;
	}

    viewer->addPointCloud(cloud, "map");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
    viewer->registerKeyboardCallback(kb_callback,NULL);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}
