#pragma once

#ifndef FILELOADER_H
#define FILELOADER_H


#include <iostream>

#include "consumer.h"
#include "ConcurrentQueue.h"

class FileLoader
{

public:
    FileLoader(std::shared_ptr<Consumer> _consumer) : consumer(_consumer) {}

    void LoadFile(const std::string& filename);
    void Run();
    void Stop();

private:
    ConcurrentQueue<std::string> m_Queue;
    bool m_IsStopped = false;
    std::shared_ptr<Consumer> consumer;

};

#endif // FILELOADER_H
