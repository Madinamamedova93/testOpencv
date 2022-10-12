#pragma once

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/core.hpp>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(std::shared_ptr<cv::Mat> image);
void SORFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
void cloud_to_img(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cv::Mat& coords, cv::Mat& image);

#endif // VISUALIZATION_H
