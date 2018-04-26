#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace pcl;

#define MAXCLUSTER 10000
#define K 32


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr lastcloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;
int num1 = 0;
static double radius=0.01;
std::vector< int > indices;
vector<string> redpoint;
boost::mutex cloud_mutex;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct callback_args{
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args );
void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);
void radius_search(pcl::PointXYZ searchPoint );



int main(int argc,char** argv)
{

    if (pcl::io::loadPCDFile("/home/zc/aaa_temp/parking_filtered.pcd", *cloud))//*intensitycloud))
    {
        std::cerr << "ERROR: Cannot open file (intensity)" << std::endl;
        return 0;
    }


    viewer->addPointCloud(cloud, "map");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    viewer->registerAreaPickingCallback(ap_callback, (void*)&cloud);
    viewer->registerKeyboardCallback(kb_callback,NULL);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    viewer->spin();

    return 1;
}

void radius_search(pcl::PointXYZ searchPoint )
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    cout<<"ra"<<radius<<endl;
    if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0){
        cout<<"search no result"<<endl;
        return ;
    }
    for(int i=0;i<pointIdxRadiusSearch.size();i++){
        cout<<"point: "<<cloud->points[pointIdxRadiusSearch[i]].x<<" "<<cloud->points[pointIdxRadiusSearch[i]].y<<" "<<cloud->points[pointIdxRadiusSearch[i]].z<<endl;
    }
    indices.insert(indices.end(),pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end());

    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(pointIdxRadiusSearch[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

    std::stringstream ss;
    std::string cloudName;
    ss << num++;
    ss >> cloudName;
    cloudName += "_cloudName";
    cout<<cloudName<<endl;
    redpoint.push_back(cloudName);

    viewer->addPointCloud(clicked_points_3d, red, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
    clicked_points_3d->clear();

}


void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args=NULL  )
{
    static int count=0;
    static int last_character_press=0;
    int character_press = event.getKeyCode();
    if(character_press != (int)'x')
    {
        cout<<"chara"<<character_press<<endl;
        cout<<(count++)<<endl;
        last_character_press=character_press;
        
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

            pcl::copyPointCloud(*cloud,*lastcloud);
            pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
            pcl::PointCloud<pcl::PointXYZ>::iterator tmpindex;
            int deletecount=0;
            for(int i=0;i<indices.size();i++){
                tmpindex=index+indices[i]-deletecount;

                cloud->erase(tmpindex);
                deletecount++;


            }
            cout<<"delete done"<<endl;
            indices.clear();
            std::stringstream ss;
            std::string cloudName;
            cloudName += "_cloudName";

            for(int i=0;i<redpoint.size();i++ )
            {
                viewer->removePointCloud(redpoint[i]);
            }
            redpoint.clear();

            viewer->removePointCloud("map");
            viewer->addPointCloud(cloud,"map");


        }
        else if(character_press==(int)'a')
        {
            cout<<"enter the radius:";
            cin>>radius;
            cout<<endl<<endl<<"radius:"<<radius<<endl;
        }

        else if(character_press==(int)'s')
        {
            pcl::io::savePCDFileBinary("culledmap.pcd",*cloud);
            cout<<"save done"<<endl;
        }
    }
}

void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    std::vector< int > getindices;

    if (event.getPointsIndices(getindices)==-1)
        return;
    indices.insert(indices.end(),getindices.begin(),getindices.end() );


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
    cout<<cloudName<<endl;
    redpoint.push_back(cloudName);

    viewer->addPointCloud(clicked_points_3d, red, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
    clicked_points_3d->clear();

}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    int getIndex=event.getPointIndex();
    if (getIndex == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    radius_search(current_point);
    return ;
}
