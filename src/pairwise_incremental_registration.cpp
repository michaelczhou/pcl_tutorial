#include <boost/make_shared.hpp>   //boost指针相关头文件
#include <pcl/point_types.h>    //点类型定义相关头文件
#include <pcl/point_cloud.h>   //点云类相关头文件
#include <pcl/point_representation.h>   //点表示相关头文件

#include <pcl/io/pcd_io.h>   //pcd文件打开存储类相关头文件

#include <pcl/filters/voxel_grid.h>   //基于体素网格化的滤波类相关头文件
#include <pcl/filters/filter.h>   //滤波相关头文件

#include <pcl/features/normal_3d.h>     //法线特征相关头文件

#include <pcl/registration/icp.h>   //icp类相关头文件
#include <pcl/registration/icp_nl.h>   //非线性icp类相关头文件
#include <pcl/registration/transforms.h>   //变换矩阵类相关头文件

#include <pcl/visualization/pcl_visualizer.h>   //可视化类相关头文件

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
//our visualizer/创建可视化对象
pcl::visualization::PCLVisualizer *p;
//its left and right viewports定义存储左右视点ID
int vp_1, vp_2;

//convenient structure to handle our pointclouds声明一个结构体，方便对点云以文件名和点云对象进行成对处理管理，在配准过程中，可以同时接受多个点云文件输入，程序从第一个文件开始，连续的两两配准处理，然后存储配准后的点云文件。
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud(new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
//以< x, y, z, curvature >形式定义一个新的点表示

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:

MyPointRepresentation ()
{
    nr_dimensions_ =4;//定义点的维度
}

//重载copyToFloatArray方法来将点转化为4维数组
virtual void copyToFloatArray (const PointNormalT &p, float* out) const
{
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
}

};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *配准前的源和目标点云
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *配准后
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  * 加载数据非常简单，我们迭代其他程序的参数，检查每一个参数是否指向一个pcd文件，如果是，创建一个添加到点云矢量data中的PCD对象。
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)   //第一个参数为命令本身，所以从第二个参数开始解析
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file检查参数是否为一个pcd后缀的文件
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models加载点云并保存在总体的点云列表中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud从点云中移除NAN点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  * 实际匹配，由子函数pairAlign具体实现，其中参数有输入一组需要配准的点云，以及是否进行下采样的设置项，其他参数输出配准后的点云及变换矩阵。
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);  //存储滤波后的源点云
  PointCloud::Ptr tgt (new PointCloud);  //存储滤波后的目标点云
  pcl::VoxelGrid<PointT> grid;    //滤波处理对象
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);   //设置滤波处理时采用的体素大小
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature//计算点云法线
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);    //设置估计对象采用的搜索对象
  norm_est.setKSearch (30);   //设置估计时进行搜索用的k数

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);   //下面分别估计源和目标点云法线
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align配准
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;   //配准对象
  reg.setTransformationEpsilon (1e-6);   //设置收敛判断条件，越小精度越大，收敛也越慢
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  //将两个点云中的对应点对之间的(src<->tgt)最大距离设置为10厘米，大于此值的点对不考虑。
  //注意：根据你的数据集大小来调整
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation设置点表示
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);  //设置源点云
  reg.setInputTarget (points_with_normals_tgt);  //设置目标点云



  //指导实例，我们希望显示匹配过程的迭代过程，为了这个目的，ICP在内部进行计算时，限制其最大的迭代次数为2，即每迭代两次就认为收敛，停止内部迭代。
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);   //设置最大迭代次数

  //手动迭代（本例中是30次），每手动迭代一次，在配准结果视口对迭代的最新的结果进行刷新显示。
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

        //accumulate transformation between each Iteration记录并积累由ICP返回的变换：
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    //如果迭代N次找到的变换和迭代N-1中找到的变换之间的差异小于传给ICP的变换收敛阈值，我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程。
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  //一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，我们求逆变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
  // Get the transformation from target to source
  targetToSource = Ti.inverse();  // 得到目标点云到源点云的变换

  //
  // Transform target back in source frame把目标点云变换到源点云坐标系下
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
 }


int main(int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;  //  存储管理所有打开的点云
  loadData (argc, argv, data);   // 加载所有点云文件到data

  // Check user input检查用户输入
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());
  
  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);    //用左半窗口创建视口vp_1
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);   //用右半窗口创建视口vp_2

	PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
  for (size_t i = 1; i < data.size (); ++i)   //循环处理所有点云
  {
    source = data[i-1].cloud;   //连续配准
    target = data[i].cloud;   //相邻两组点云

    // Add visualization data可视化为配准的源和目标点云
    showCloudsLeft(source, target);

    //调用子函数完成一组点云的配准，temp返回配准后两组点云在第一组点云坐标下的点云，pairTransform返回从目标点云target到源点云source的变换矩阵。
    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform把当前的两两配对后的点云temp转换到全局坐标系下（第一个输入点云的坐标系）返回result
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform用当前的两组点云之间的变换更新全局变换
    GlobalTransform = GlobalTransform * pairTransform;

    //save aligned pair, transformed into the first cloud's frame保存转换到第一个点云坐标下的当前配准后的两组点云result到文件i.pcd
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);

  }
}

