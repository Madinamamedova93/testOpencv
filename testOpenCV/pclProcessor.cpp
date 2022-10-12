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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    std::cerr << "Cloud after SOR filtering: " << cloud_filtered->width * cloud_filtered->height << " data points " << std::endl;
    copyPointCloud(*cloud_filtered, *cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(std::shared_ptr<cv::Mat> image)
{
    cv::Mat coords(3, image->cols * image->rows, CV_64FC1);
    for (int col = 0; col < coords.cols; ++col)
    {
        coords.at<double>(0, col) = col % image->cols;
        coords.at<double>(1, col) = col / image->cols;
        coords.at<double>(2, col) = 10;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int y = 0; y < image->rows; y++)
    {
        for (int x = 0; x < image->cols; x++)
        {
            pcl::PointXYZRGB point;
            point.x = coords.at<double>(0, y * image->cols + x);
            point.y = coords.at<double>(1, y * image->cols + x);
            point.z = coords.at<double>(2, y * image->cols + x);

            cv::Vec3b color = image->at<cv::Vec3b>(cv::Point(x, y));
            uint8_t r = (color[2]);
            uint8_t g = (color[1]);
            uint8_t b = (color[0]);

            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);

            cloud->points.push_back(point);
        }
    }
    SORFilter(cloud);
    return cloud;
}


void cloud_to_img(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cv::Mat& coords, cv::Mat& image)
{
    coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
    image = cv::Mat(480, 640, CV_8UC3);
    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            coords.at<double>(0, y * image.cols + x) = cloud->points.at(y * image.cols + x).x;
            coords.at<double>(1, y * image.cols + x) = cloud->points.at(y * image.cols + x).y;
            coords.at<double>(2, y * image.cols + x) = cloud->points.at(y * image.cols + x).z;

            cv::Vec3b color = cv::Vec3b(
            cloud->points.at(y * image.cols + x).b,
            cloud->points.at(y * image.cols + x).g,
            cloud->points.at(y * image.cols + x).r);

            image.at<cv::Vec3b>(cv::Point(x, y)) = color;
        }
    }
}

