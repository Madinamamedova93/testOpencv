#ifndef STEREOGRAMSOLVER_H
#define STEREOGRAMSOLVER_H
#include <mutex>
#include <thread>

#include <iostream>
#include <opencv2/opencv.hpp>

#define MIN_REPEAT_SEGMENTS_COUNT 4
#define MAX_REPEAT_SEGMENTS_COUNT 20
#define MATCH_BLOCKS_WIDTH 10


std::vector<cv::Point3f> convertMatToPointsCloud(std::shared_ptr<cv::Mat> img);

class stereogramSolver {
 public:
  stereogramSolver();
  ~stereogramSolver();

  std::shared_ptr<cv::Mat> reconstructDepthMap(std::shared_ptr<cv::Mat>  stereogram, int countThreads = 1);

 private:
  int findRepeatOffset(std::shared_ptr<cv::Mat> stereogram) const;
  void reconstructDepthMapPartImg(const cv::Mat partOfStereogram,
                                  cv::Mat* outDepthMap,
                                  int offset,
                                  int threadIndex);

  int m_countThreads;
  std::vector<std::thread*> m_threads;

  mutable std::mutex m_PrintLocker;
};
#endif
