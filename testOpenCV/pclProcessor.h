#pragma once

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/core.hpp>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr img_to_cloud(std::shared_ptr<cv::Mat> image);

#endif // VISUALIZATION_H
