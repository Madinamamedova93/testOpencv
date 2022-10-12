#pragma once

#ifndef CONSUMER_H
#define CONSUMER_H

#include "ConcurrentQueue.h"

#include <iostream>

#include <opencv4/opencv2/core.hpp>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>

class Consumer
{
public:
    void AddImage(std::shared_ptr<cv::Mat> image);
    void Run();
    std::string getKey();
    void resetKey();
    void stop();
    void draw_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* );

private:
    ConcurrentQueue<std::shared_ptr<cv::Mat>> m_Queue;
    bool m_IsStopped = false;
    std::string key_arrow;
    int m_Key = -1;
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_Viewer;

};

#endif // CONSUMER_H
