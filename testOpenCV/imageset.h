#pragma once

#ifndef IMAGESET_H
#define IMAGESET_H

#include <iostream>
#include <set>

class ImageSet
{
public:

    ImageSet(const std::string& path);
    ~ImageSet() {}

    std::string getCurrentFile();
    std::string getNextFile();
    std::string getPrevFile();

private:
    std::string Imgpath;
    std::set<std::string> filenames;
    std::string currentFile;
};

#endif // IMAGESET_H
