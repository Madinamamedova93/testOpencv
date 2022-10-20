cmake_minimum_required(VERSION 3.5)

project(testOpenCV LANGUAGES CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(VTK REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
include_directories( ${BOOST_INCLUDE_DIRS} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(testOpenCV
    main.cpp
    imageset.cpp imageset.h
    fileloader.cpp fileloader.h
    consumer.cpp consumer.h
    pclProcessor.cpp pclProcessor.h
    stereosolver.cpp stereosolver.h
    ConcurrentQueue.h
    )

target_link_libraries( testOpenCV ${OpenCV_LIBS} ${BOOST_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

