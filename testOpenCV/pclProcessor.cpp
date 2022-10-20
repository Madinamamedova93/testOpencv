#include "pclProcessor.h"

#include <pcl/PCLHeader.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace pcl;


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

    return cloud;
}

