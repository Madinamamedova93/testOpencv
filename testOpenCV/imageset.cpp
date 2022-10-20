#include "imageset.h"

#include <iostream>
#include <queue>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

namespace fs = boost::filesystem;

ImageSet::ImageSet(const std::string &path)
{
    Imgpath = path;
    fs::directory_entry directory{ path };

    for (const auto& entry : fs::directory_iterator{ directory }) {
        std::string filename = entry.path().string();
        filenames.insert(entry.path().string());
    }

    currentFile = *(filenames.begin());

}

std::string ImageSet::getCurrentFile()
{
    return currentFile;
}

std::string ImageSet::getNextFile()
{
    auto it = filenames.find(currentFile);
    it++;
    if (it == filenames.end())
        it = filenames.begin();
    currentFile = *it;

    return currentFile;
}

std::string ImageSet::getPrevFile()
{
    auto it = filenames.find(currentFile);
    if (it == filenames.begin())
        it = filenames.end();
    it--;
    currentFile = *it;

    return currentFile;
}
