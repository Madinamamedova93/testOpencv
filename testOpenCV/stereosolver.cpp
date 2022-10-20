#include "stereosolver.h"

stereogramSolver::stereogramSolver() : m_countThreads(1) {}

stereogramSolver::~stereogramSolver() {

  for (const auto& i : m_threads) {
    if (i) {
      i->join();
      delete i;
    }
  }
  m_threads.clear();
}

std::shared_ptr<cv::Mat> stereogramSolver::reconstructDepthMap(std::shared_ptr<cv::Mat> stereogram,
                                              int countThreads) {
  if (stereogram->empty()) {
    std::cout << "ERROR. cv::Mat stereogramSolver::reconstructDepthMap(const "
                 "cv::Mat& stereogram) get empty Mat..."
              << std::endl;
    return std::make_shared<cv::Mat>(stereogram->size(), CV_8UC1, cv::Scalar::all(0));
  }
  m_countThreads = countThreads <= stereogram->rows ? countThreads : stereogram->rows;

  int offset = findRepeatOffset(stereogram);
  if (offset <= 0) {
    return std::make_shared<cv::Mat>();
  }

  int step = stereogram->rows / countThreads;

  std::vector<cv::Mat> chunksStereogram;
  for (int i = 0; i < m_countThreads; i++) {
    chunksStereogram.push_back((*stereogram)(
        cv::Rect(0, i * step, stereogram->cols,
                 i == m_countThreads - 1 ? stereogram->rows - i * step
                                         : i * step + step < stereogram->rows
                                               ? step
                                               : stereogram->rows - i * step)));
  }

  std::vector<cv::Mat*> chunksDepthMap;
  for (size_t i = 0; i < chunksStereogram.size(); ++i) {
    chunksDepthMap.push_back(new cv::Mat());
  }

  for (int i = 0; i < chunksStereogram.size(); i++) {
    m_threads.push_back(
        new std::thread(&stereogramSolver::reconstructDepthMapPartImg, this,
                        chunksStereogram[i], chunksDepthMap[i], offset, i));
    std::lock_guard<std::mutex> locker(m_PrintLocker);
    std::cout << "Thread " << i << " was started.." << std::endl;
  }

  for (auto& i : m_threads) {
    if (i)
      i->join();
  }

  std::cout << "All threads finished!!" << std::endl;

  cv::Mat depthMap(stereogram->rows, stereogram->cols - offset, CV_8UC1,
                   cv::Scalar::all(255));
  int yOffset = 0;

  for (auto& i : chunksDepthMap) {
    i->copyTo(depthMap(cv::Rect(0, yOffset, i->cols, i->rows)));
    yOffset += i->rows;

    if (i)
      delete i;
    i = nullptr;
  }

  for (auto& i : m_threads) {
    if (i)
      delete i;
    i = nullptr;
  }
  m_threads.clear();
  auto depthMapP = std::make_shared<cv::Mat>(depthMap);
  return depthMapP;
}

int stereogramSolver::findRepeatOffset(std::shared_ptr<cv::Mat> stereogram) const {
  if (stereogram->empty()) {
    std::lock_guard<std::mutex> locker(m_PrintLocker);
    std::cout << "ERROR. \tint stereogramSolver::findRepeatOffset(const "
                 "cv::Mat& stereogram)\t get empty Mat..."
              << std::endl;
    return -1;
  }
  int minDifference = -1;
  int bestStep = -1;
  std::vector<std::pair<int, int>> storage;  // all overlaps result storage
  for (int i = stereogram->cols / MAX_REPEAT_SEGMENTS_COUNT;
       i < stereogram->cols / MIN_REPEAT_SEGMENTS_COUNT; ++i) {
    int currentDiff = 0;
    for (int r = 0; r < stereogram->rows; ++r) {
      for (int c = 0; c < stereogram->cols - i; ++c) {
        for (int nChannel = 0; nChannel < stereogram->channels(); ++nChannel) {
          currentDiff +=
              std::abs(stereogram->at<unsigned char>(
                           r, c * stereogram->channels() + nChannel) -
                       stereogram->at<unsigned char>(
                           r, (c + i) * stereogram->channels() + nChannel));
        }
      }
    }
    storage.push_back(std::pair<int, int>(i, currentDiff));
    if (minDifference < 0 || currentDiff < minDifference) {
      minDifference = currentDiff;
      bestStep = i;
    }
  }
  auto halfShiftIt = std::find_if(storage.begin(), storage.end(),
                                  [=](const std::pair<int, int>& elem) -> bool {
                                    return elem.first == bestStep / 2;
                                  });
  if (halfShiftIt != storage.end()) {
    int halfShift_i, halfShift_dif;
    std::tie(halfShift_i, halfShift_dif) = *halfShiftIt;
    if (static_cast<double>(std::abs(halfShift_dif - minDifference)) /
            static_cast<double>(minDifference) <
        0.5) {
      std::cout << "difference precentage"
                << static_cast<double>(
                       std::abs(halfShift_dif - minDifference)) /
                       static_cast<double>(minDifference)
                << std::endl;
      return halfShift_i;
    }
  }
  return bestStep;
}

void stereogramSolver::reconstructDepthMapPartImg(
    const cv::Mat partOfStereogram,
    cv::Mat* outDepthMap,
    int offset,
    int threadIndex) {
  if (partOfStereogram.empty() || offset < 1) {
    std::lock_guard<std::mutex> locker(m_PrintLocker);
    std::cout
        << "ERROR. \tvoid stereogramSolver::reconstructDepthMapPartImg(const "
           "cv::Mat& partOfStereogram, cv::Mat& outDepthMap, int offset)\t "
           "invalid input param..."
        << std::endl;
    return;
  }

  *outDepthMap =
      cv::Mat(cv::Size(partOfStereogram.cols - offset, partOfStereogram.rows),
              CV_8UC1, cv::Scalar::all(0));

  int completePrecent = 0;
  for (int r = 0; r < partOfStereogram.rows; r++) {
    for (int c = 0; c < partOfStereogram.cols - offset - MATCH_BLOCKS_WIDTH;
         c++) {
      cv::Rect zoneAfterOffset(c + offset, r, MATCH_BLOCKS_WIDTH, 1);

      int bestDepth = 0;
      double bestDiff = -1;
      for (int depth = 0; depth < offset / 2; ++depth) {
        double currentDiff = 0;
        for (int nChannel = 0; nChannel < partOfStereogram.channels();
             ++nChannel) {
          cv::Mat diffMat;
          cv::absdiff(
              partOfStereogram(cv::Rect(c + depth, r, MATCH_BLOCKS_WIDTH, 1)),
              partOfStereogram(zoneAfterOffset), diffMat);
          cv::Scalar diff = cv::sum(diffMat);
          for (int i = 0; i < diff.channels; ++i) {
            currentDiff += diff[i];
          }
        }
        if (bestDiff < 0 || bestDiff > currentDiff) {
          bestDiff = currentDiff;
          bestDepth = depth;
        }
      }
      outDepthMap->at<unsigned char>(r, c) = bestDepth;
    }
    int currentCompletePrecent =
        static_cast<int>(static_cast<double>(r) /
                         static_cast<double>(partOfStereogram.rows) * 100);
    if (completePrecent != currentCompletePrecent) {
      completePrecent = currentCompletePrecent;
      std::lock_guard<std::mutex> locker(m_PrintLocker);
      std::cout << "Thread " << threadIndex
                << ". Complete precent: " << completePrecent << std::endl;
    }
  }
  *outDepthMap *= 255 / (offset / 4);
}
