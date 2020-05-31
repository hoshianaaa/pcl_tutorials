#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>


void centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

    for(size_t i = 0; i < cloud->points.size(); i++){
        cloud->points[i].x = cloud->points[i].x - xyz_centroid[0];//X座標の移動
        cloud->points[i].y = cloud->points[i].y - xyz_centroid[1];//Y座標の移動
        cloud->points[i].z = cloud->points[i].z - xyz_centroid[2];//Z座標の移動
    }
}

pcl::PointCloud<pcl::PointXYZ> rotation_x(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theta)///rotate point cloud by Y axiz
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_centroid = cloud;

    centroid(cloud_centroid);//点群の重心位置求めを（0,0,0）に移動する

    Eigen::Matrix4f rot_x;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out = *cloud;
    double theta_x = theta * 3.1415926 / 180.0;//角度の変換
    rot_x(0,0) = 1.0;
    rot_x(1,0) = 0.0;
    rot_x(2,0) = 0.0;
    rot_x(3,0) = 0.0;
    rot_x(0,1) = 0.0;
    rot_x(1,1) = cos(theta_x);
    rot_x(2,1) = -sin(theta_x);
    rot_x(3,1) = 0.0;
    rot_x(0,2) = 0.0;
    rot_x(1,2) = sin(theta_x);
    rot_x(2,2) = cos(theta_x);
    rot_x(3,2) = 0.0;
    rot_x(0,3) = 0.0;
    rot_x(1,3) = 0.0;
    rot_x(2,3) = 0.0;
    rot_x(3,3) = 1.0;

    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud_out.points[i].x = rot_x(0,0) * cloud->points[i].x + rot_x(0,1) * cloud->points[i].y + rot_x(0,2) * cloud->points[i].z + rot_x(0,3) * 1;
        cloud_out.points[i].y = rot_x(1,0) * cloud->points[i].x + rot_x(1,1) * cloud->points[i].y + rot_x(1,2) * cloud->points[i].z + rot_x(1,3) * 1;
        cloud_out.points[i].z = rot_x(2,0) * cloud->points[i].x + rot_x(2,1) * cloud->points[i].y + rot_x(2,2) * cloud->points[i].z + rot_x(2,3) * 1;
    }

    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ> rotation_y(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theta)///rotate point cloud by Y axiz
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_centroid = cloud;

    centroid(cloud_centroid);//点群の重心位置求めを（0,0,0）に移動する

    Eigen::Matrix4f rot_y;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out = *cloud;
    double theta_y = theta * 3.1415926 / 180.0;//角度の変換
    rot_y(0,0) = cos(theta_y);
    rot_y(1,0) = 0.0;
    rot_y(2,0) = sin(theta_y);
    rot_y(3,0) = 0.0;
    rot_y(0,1) = 0.0;
    rot_y(1,1) = 1.0;
    rot_y(2,1) = 0.0;
    rot_y(3,1) = 0.0;
    rot_y(0,2) = -sin(theta_y);
    rot_y(1,2) = 0.0;
    rot_y(2,2) = cos(theta_y);
    rot_y(3,2) = 0.0;
    rot_y(0,3) = 0.0;
    rot_y(1,3) = 0.0;
    rot_y(2,3) = 0.0;
    rot_y(3,3) = 1.0;

    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud_out.points[i].x = rot_y(0,0) * cloud->points[i].x + rot_y(0,1) * cloud->points[i].y + rot_y(0,2) * cloud->points[i].z + rot_y(0,3) * 1;
        cloud_out.points[i].y = rot_y(1,0) * cloud->points[i].x + rot_y(1,1) * cloud->points[i].y + rot_y(1,2) * cloud->points[i].z + rot_y(1,3) * 1;
        cloud_out.points[i].z = rot_y(2,0) * cloud->points[i].x + rot_y(2,1) * cloud->points[i].y + rot_y(2,2) * cloud->points[i].z + rot_y(2,3) * 1;
    }

    return cloud_out;
}

int main(int argc, char **argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());

  cloud_in->width = 5;
  cloud_in->height = 1;
  cloud_in->is_dense = false;
  cloud_in->resize(cloud_in->width * cloud_in->height);

  cloud_out->width = 5;
  cloud_out->height = 1;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->width * cloud_out->height);

  cloud_in->points[0].x = 0;
  cloud_in->points[0].y = 0;
  cloud_in->points[0].z = 0;

  cloud_in->points[1].x = 2;
  cloud_in->points[1].y = 0;
  cloud_in->points[1].z = 0;

  cloud_in->points[2].x = 0;
  cloud_in->points[2].y = 2;
  cloud_in->points[2].z = 0;

  cloud_in->points[3].x = 3;
  cloud_in->points[3].y = 0;
  cloud_in->points[3].z = 4;

  cloud_in->points[4].x = 0;
  cloud_in->points[4].y = 3;
  cloud_in->points[4].z = 4;

  cloud_in->points[5].x = 0;
  cloud_in->points[5].y = 0;
  cloud_in->points[5].z = 5;

  // make a translation
  Eigen::Vector3f trans;
  trans << 0.5,1.0,0.75;
  //std::cout << "here" <<std::endl;
  std::vector<int> indices(cloud_in->points.size());
  for (int i=0;i<cloud_in->points.size();i++)
  {
    indices[i] = i;
    cloud_out->points[i].x = cloud_in->points[i].x + trans(0);
    cloud_out->points[i].y = cloud_in->points[i].y + trans(1);
    cloud_out->points[i].z = cloud_in->points[i].z + trans(2);
    std::cout << cloud_out->points[i].x << " — " << cloud_out->points[i].y << " — " << cloud_out->points[i].z << std::endl;
  }

  // rotate
  *cloud_out = rotation_x(cloud_out, 90);

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
  TESVD.estimateRigidTransformation (*cloud_in,*cloud_out,transformation2);
  std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));

  return 0;
}
