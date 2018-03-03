#if 0
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace pcl;

#define MAXCLUSTER 10000
#define K 32


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr intensitycloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr lastcloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;
int num1=0;
static double radius=0.01;
std::vector< int > indices;
vector<string> redpoint;
boost::mutex cloud_mutex;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args );
void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);
void radius_search(pcl::PointXYZ searchPoint );

//vector<int> cluster(int getIndex);

int main1(int argc,char** argv)
{
//    if (pcl::io::loadPCDFile("/home/zc/Desktop/shandian/map.pcd", *cloud))
//    {
//        std::cerr << "ERROR: Cannot open file (no intensity)" << std::endl;
//        return 0;
//    }

    if (pcl::io::loadPCDFile("/home/zc/Desktop/shandian/map.pcd", *intensitycloud))
    {
        std::cerr << "ERROR: Cannot open file (intensity)" << std::endl;
        return 0;
    }
    PointXYZ tmp;
    for(int i=0;i< intensitycloud->points.size();i++){
        tmp.x=intensitycloud->points[i].x;
        tmp.y=intensitycloud->points[i].y;
        tmp.z=intensitycloud->points[i].z;
        cloud->push_back(tmp);
    }

    cloud_mutex.lock();    // for not overwriting the point cloud

    viewer->addPointCloud(cloud, "map");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerAreaPickingCallback(ap_callback, (void*)&cloud);
    viewer->registerKeyboardCallback(kb_callback,NULL);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    viewer->spin();
    cloud_mutex.unlock();


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}

void radius_search(pcl::PointXYZ searchPoint ){
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


/*
vector<int> cluster(int getIndex){

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*tmpcloud);
    pcl::PointCloud<pcl::PointXYZ>::iterator index = tmpcloud->begin();
    index+=getIndex;
    tmpcloud->erase(index);//删除最初搜索点

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tmpcloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x=cloud->points[getIndex].x;
    searchPoint.y=cloud->points[getIndex].y;
    searchPoint.z=cloud->points[getIndex].z;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<int> resultIndices;
    int searchCount=1;


    double sumDistance=0;
    double aveDistance;
    if(kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)<=0){
        return resultIndices;
    }
    searchCount+=pointIdxNKNSearch.size();

    sort(pointIdxNKNSearch.begin(),pointIdxNKNSearch.end());
    index = tmpcloud->begin();
    pcl::PointCloud<pcl::PointXYZ>::iterator tmpindex;
    vector<pcl::PointXYZ> searchResult;
    for(int i=0;i<pointIdxNKNSearch.size();i++){
        tmpindex=index+pointIdxNKNSearch[i]-i;
        //searchResult.push_back(  );
        tmpcloud->erase(tmpindex);//删除第一次K近邻搜索得到的点


    }

    resultIndices.insert(resultIndices.begin(),pointIdxNKNSearch.begin(),pointIdxNKNSearch.end());
    for(int i=0;i<pointNKNSquaredDistance.size();i++){
        sumDistance+=pointNKNSquaredDistance[i];
    }
    aveDistance=sumDistance/pointNKNSquaredDistance.size();
    double radius=aveDistance;//K个K近邻搜索的平均距离为范围搜索的半径
    int vectorIndex=1;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    vector<pcl::PointXYZ> vSearchPoint;
    while(resultIndices.size()<MAXCLUSTER){
        for(int i=0;i<pointIdxNKNSearch.size();i++){
            //searchPoint.x=tmpcloud->points
        }
        searchPoint.x=cloud->points[resultIndices[vectorIndex]].x;
        searchPoint.y=cloud->points[resultIndices[vectorIndex]].y;
        searchPoint.z=cloud->points[resultIndices[vectorIndex]].z;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=16 )
        {
        }

    }
}
*/



void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args=NULL  ){
    static int count=0;
    static int last_character_press=0;
    int character_press=event.getKeyCode();
    if(last_character_press!=character_press && character_press!=(int)'x'){
        cout<<"chara"<<character_press<<endl;
        cout<<(count++)<<endl;
        last_character_press=character_press;
        if(character_press==(int)'b'){
            pcl::copyPointCloud(*lastcloud,*cloud);
            viewer->removePointCloud("map");
            viewer->addPointCloud(cloud,"map");
        }else if(character_press==(int)'d'){
            cout<<"b"<<endl;

            sort(indices.begin(),indices.end());
            for (int i=0;i<indices.size();i++){
                cout<<indices[i]<<" ";
            }
            cout<<"ok"<<endl;
            pcl::copyPointCloud(*cloud,*lastcloud);
            pcl::PointCloud<pcl::PointXYZI>::iterator index = intensitycloud->begin();
            pcl::PointCloud<pcl::PointXYZI>::iterator tmpindex;
            int deletecount=0;
            for(int i=0;i<indices.size();i++){
                tmpindex=index+indices[i]-deletecount;
                if(tmpindex->intensity<0.11){
                    intensitycloud->erase(tmpindex);
                    deletecount++;
                }

            }
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


        }else if(character_press==(int)'a'){
            cout<<"enter the radius:";
            cin>>radius;
            cout<<endl<<endl<<"radius:"<<radius<<endl;
        }else if(character_press==(int)'s'){
            pcl::io::savePCDFileBinary("culledmap.pcd",*intensitycloud);
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
    /*
    for (int i=0;i<indices.size();i++){
        cout<<indices[i]<<" ";
    }

    cout<<endl<<"sort"<<endl;
    sort(indices.begin(),indices.end());
    for (int i=0;i<indices.size();i++){
        cout<<indices[i]<<" ";
    }
    cout<<"ok"<<indices.size()<<endl;
    pcl::copyPointCloud(*cloud,*lastcloud);
    pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
    pcl::PointCloud<pcl::PointXYZ>::iterator tmpindex;
    for(int i=0;i<indices.size();i++){
        tmpindex=index+indices[i]-i;
        cloud->erase(tmpindex);
    }
    viewer->removePointCloud("map");
    viewer->addPointCloud(cloud,"map");
    */

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
    cout<<"intensity::"<<intensitycloud->points[getIndex].intensity<<endl;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    radius_search(current_point);
    return ;




#if 0
    pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
    index+=getIndex;
    cout<<"size:"<<cloud->width<<"  "<<cloud->height<<endl;
    cloud->erase(index);
    cout<<"size:"<<cloud->width<<"  "<<cloud->height<<endl;
    //cout<<"index  "<<index<<endl;
    //PointIndicesConstPtr indices;
    //IndicesPtr indice;


    //indices->indices.push_back(index);


    //vector<int> searchIndices=cluster(getIndex);
    //indices.insert(indices.end(),searchIndices.begin(),searchIndices.end());
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    bool found=false;
    for(int i=0;i<redpoint.size();i++){
        if(redpoint[i]=="clicked_points"){
            found=true;
            break;
        }
    }
    if(!found)
        redpoint.push_back("clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    //cout<<"size:"<<cloud->width<<"  "<<cloud->height<<endl;
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
#endif

}

//*/


/* 3
typedef pcl::PointXYZRGBA PointT_XYZ;
typedef pcl::PointCloud<PointT_XYZ> PointCloudT_XYZ;


// Mutex: //
boost::mutex cloud_mutex;

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT_XYZ::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};



void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{

    struct callback_args* data = (struct callback_args *)args;
    std::cout << "Picking event active" << std::endl;
    PointT_XYZ current_point;
    if (event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(current_point.x, current_point.y, current_point.z);
        //std::cout << x << ";" << y << ";" << z << std::endl;
        data->clicked_points_3d->points.push_back(current_point);

    }
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT_XYZ> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main()
{

    //visualizer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

    pcl::io::loadPCDFile("/home/ywl/桌面/point_culling/map.pcd", *cloud);

    //viewer->addPointCloud(cloud, "bunny");

    cloud_mutex.lock();    // for not overwriting the point cloud

    // Display pointcloud:
    viewer->addPointCloud(cloud, "map");

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT_XYZ::Ptr clicked_points_3d(new PointCloudT_XYZ);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);

    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer->spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}
*/
#endif
