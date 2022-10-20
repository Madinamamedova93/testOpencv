#include "consumer.h"
#include "pclProcessor.h"

#include <iostream>
#include <thread>

#include <opencv2/core.hpp>

#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>


void Consumer::AddImage(std::shared_ptr<cv::Mat> image)
{
    m_Queue.Queue(image);
}

void Consumer::Run()
{
    std::vector<cv::Point3d> points;
    stereogramSolver solver;
        while (!m_IsStopped)
        {            
            std::shared_ptr<cv::Mat> img;
            if (m_Queue.GetNext(img))
            {
                std::shared_ptr<cv::Mat> depthMap = solver.reconstructDepthMap(img, 1);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = img_to_cloud(depthMap);
                draw_cloud(cloud);
            }
            if (m_Viewer != nullptr && !m_Viewer->wasStopped())
            {
                m_Viewer->spinOnce(100);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (m_Viewer && !m_Viewer->wasStopped())
        {
            m_Viewer->close();
            m_Viewer = nullptr;
        }

}

void Consumer::draw_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::cout << "draw_cloud" << std::endl;
    if (m_Viewer == nullptr)
    {
        cout<<"NULL "<<endl;
        m_Viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
        m_Viewer->registerKeyboardCallback(&Consumer::keyboard_callback, *this);
    }
    else
    {
        m_Viewer->removeAllPointClouds();
        m_Viewer->spinOnce(100);
    }

    m_Viewer->addPointCloud(cloud);
}

void Consumer::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)// viewer_void)
{
    if (event.keyDown())
      {
        if (event.getKeySym() == "Left")
        {
            key_arrow = event.getKeySym();
        }
        else if(event.getKeySym() == "Right")
        {
            key_arrow = event.getKeySym();
        }
        else if(event.getKeySym() == "Escape")
        {
            key_arrow = event.getKeySym();
        }
    }
}

void Consumer::stop()
{
    m_IsStopped = true;
}

std::string Consumer::getKey()
{
    return key_arrow;
}

void Consumer::resetKey()
{
    key_arrow = "";
}


