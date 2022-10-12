#pragma once

#ifndef IMAGESET_H
#define IMAGESET_H

#include <iostream>
#include <set>

class ImageSet
{
public:

    ImageSet(const std::string& path);

    std::string getCurrentFile();
    std::string getNextFile();
    std::string getPrevFile();

    bool is_right_type(const std::string& filename);

    std::string Imgpath;
    std::set<std::string> filenames;
    std::string currentFile;

    ~ImageSet() {}
};

#endif // IMAGESET_H
