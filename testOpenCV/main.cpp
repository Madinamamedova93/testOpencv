
#include <iostream>
#include <thread>

#include "imageset.h"

#include "fileloader.h"
#include "consumer.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

using namespace std;
namespace fs = boost::filesystem;

bool is_right_path(const std::string& path)
{
    try {
        if (!fs::exists(path)) {
            std::cout << "Path Not exists" << std::endl;
            return 0;
        }

        if (fs::is_directory(path)) {
            if (fs::is_empty(path)) {
                std::cout << "Directory is empty" << std::endl;
                return 0;
            }
            else return true;
        }
        else {
            std::cout << "It isn't directory" << std::endl;
            return 0;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
    }
    return EXIT_SUCCESS;
}


int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        std::cout << "start" << std::endl;
        if (is_right_path(argv[1])) {
            ImageSet imgSt(argv[1]);

            auto consumer = std::make_shared<Consumer>();
            auto fileLoader = std::make_shared<FileLoader>(consumer);

            std::thread loaderThread([&fileLoader]() -> void {fileLoader->Run(); });
            std::thread consumerThread([&consumer]() -> void {consumer->Run(); });

            fileLoader->LoadFile(imgSt.getCurrentFile());

            bool m_IsStopped = false;
            while (!m_IsStopped) {
                std::string key = consumer->getKey();

                if (key == "Right") {
                    fileLoader->LoadFile(imgSt.getNextFile());
                    consumer->resetKey();
                }
                else if (key == "Left") {
                    fileLoader->LoadFile(imgSt.getPrevFile());
                    consumer->resetKey();
                }
                else if (key == "Escape") {
                    fileLoader->Stop();
                    consumer->stop();
                    m_IsStopped = true;
                }

            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            loaderThread.join();

            consumerThread.join();

            cv::destroyAllWindows();
        }
        else
            return 0;
    }

    else {
        std::cout << "Not arguments" << std::endl;
        return 0;
    }

    return 0;

}
