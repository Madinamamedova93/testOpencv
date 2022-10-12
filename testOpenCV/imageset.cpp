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

    for (const auto& entry : fs::directory_iterator{ directory })
    {
        std::string filename = entry.path().string();
        if (is_right_type(filename))
        {
            filenames.insert(entry.path().string());
        }
    }

    currentFile = *(filenames.begin());

}

bool ImageSet::is_right_type(const std::string& filename)
{
    FILE* file = fopen(filename.c_str(), "r");

    if (file == NULL)
        return false;

    unsigned char bytes[3];
    fread(bytes, 3, 1, file);

    if (bytes[0] == 0xff && bytes[1] == 0xd8 && bytes[2] == 0xff)
        return true;
    else
        return false;

    return 0;
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
