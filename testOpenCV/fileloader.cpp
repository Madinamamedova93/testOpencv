#include "fileloader.h"

#include <iostream>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;

void FileLoader::LoadFile(const std::string& filename)
{
    m_Queue.Queue(filename);
}

void FileLoader::Run()
{
    while (!m_IsStopped)
        {
            std::string filePath;
            if (m_Queue.GetNext(filePath)) {
                std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(cv::imread(filePath));
                consumer->AddImage(imgPtr);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

}

void FileLoader::Stop()
{
    m_IsStopped = true;

}

