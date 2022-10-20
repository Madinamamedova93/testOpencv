#pragma once
#ifndef CONCURRENTQUEUE_H
#define CONCURRENTQUEUE_H

#include <deque>
#include <mutex>
#include "stereosolver.h"

template <class T>
class ConcurrentQueue
{
public:

    void Queue(const T& value)
    {
        std::unique_lock<std::mutex> lock(m_Mutex);
        m_Queue.push_back(value);
    }

    bool GetNext(T& value)
    {
        std::unique_lock<std::mutex> lock(m_Mutex);
        if (m_Queue.empty())
            return false;

        value = m_Queue.front();
        m_Queue.pop_front();
        return true;
    }

private:

    std::mutex m_Mutex;
    std::deque<T> m_Queue;
    stereogramSolver stereogram;

};

#endif // CONCURRENTQUEUE_H
