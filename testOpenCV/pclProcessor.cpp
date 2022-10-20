#include "pclProcessor.h"

#include <pcl/PCLHeader.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace pcl;

void SORFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::cout<<"Start filter method"<<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    std::cout<<"setInputCloud"<<endl;
    sor.setInputCloud(cloud);
    std::cout<<"setMeanK"<<endl;
    sor.setMeanK(50);
    std::cout<<"setStddevMulThresh"<<endl;
    sor.setStddevMulThresh(1.0);
    std::cout<<"cloud_filtered"<<endl;
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after SOR filtering: " << cloud_filtered->width * cloud_filtered->height << " data points " << std::endl;
    copyPointCloud(*cloud_filtered, *cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr img_to_cloud(std::shared_ptr<cv::Mat> image)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (image->type() == CV_8UC1) {
        std::cout<<"first if"<<std::endl;
      for (int r = 0; r < image->rows; ++r) {
        for (int c = 0; c < image->cols; ++c) {
          if (image->at<unsigned char>(r, c) > 5) {
            cloud->push_back(pcl::PointXYZ(r, c, image->at<unsigned char>(r, c)));
          }
        }
      }
      std::cout<<"OK"<<std::endl;
    } else {
      std::cout << "std::vector<pcl::PointXYZRGB> convertMatToPointsCloud(const "
                   "cv::Mat& img) need CV_8UC1 mat as input. Current type is:"
                << image->type() << std::endl;
    }

   /* std::cout<<"Start point cloud"<<std::endl;
    cv::Mat coords(3, image->cols * image->rows, CV_64FC1);
    for (int col = 0; col < coords.cols; ++col)
    {
        coords.at<double>(0, col) = col % image->cols;
        coords.at<double>(1, col) = col / image->cols;
        coords.at<double>(2, col) = 10;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::cout<<"Start point cloud 2"<<std::endl;
    for (int y = 0; y < image->rows; y++)
    {
        for (int x = 0; x < image->cols; x++)
        {
          //  std::cout<<"PointXYZRGB"<<std::endl;
            pcl::PointXYZRGB point;
            point.x = coords.at<double>(0, y * image->cols + x);
            point.y = coords.at<double>(1, y * image->cols + x);
            point.z = coords.at<double>(2, y * image->cols + x);

         //   std::cout<<"color"<<std::endl;
            cv::Vec3b color = image->at<cv::Vec3b>(cv::Point(x, y));
            uint8_t r = (color[2]);
            uint8_t g = (color[1]);
            uint8_t b = (color[0]);

         //   std::cout<<"rgb"<<std::endl;
            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);
           // std::cout<<"cloud"<<std::endl;
            cloud->points.push_back(point);
        }
    }
    std::cout<<"Start filter"<<std::endl;
    SORFilter(cloud);*/
    //cout<<cloud<<endl;
    //std::cout<<"Start filter"<<std::endl;
    //SORFilter(cloud);
    return cloud;
}

