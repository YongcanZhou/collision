cmake_minimum_required(VERSION 3.12)

project(Robot_OCC)

# use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_compile_options(-bigobj)
if (WIN32)
set(Qt5Core_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Core)
set(Qt5Widgets_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Widgets)
set(Qt5Gui_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Gui)
set(Qt5Gamepad_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Gamepad)
set(Qt5PrintSupport_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5PrintSupport)
# set(Eigen3_DIR D:/app/eigen-3.4.0)
endif(WIN32)

# if (UNIX)
# set(Qt5Core_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Core)
# set(Qt5Widgets_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Widgets)
# set(Qt5Qt5Gui_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Gui)
# set(Qt5Gamepad_DIR /opt/Qt5.11.2/5.11.HEADER2/gcc_64/lib/cmake/Qt5Gamepad)
# set(Qt5PrintSupport_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5PrintSupport)
# endif(UNIX)

find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Gamepad REQUIRED)
find_package(Eigen3 REQUIRED)

# find OCC
IF(WIN32)
        find_package(OpenCASCADE REQUIRED PATHS D:/app/OpenCASCADE-7.4.0-vc14-64/opencascade-7.4.0/cmake)
endif(WIN32)

# find aris
if(UNIX)
        add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        # find_package(aris REQUIRED PATHS C:/aris/aris-2.2.1.220620/share/aris/cmake)
        find_package(aris REQUIRED PATHS C:/aris/aris-2.0.0.220225/share/aris/cmake)
endif(WIN32)

# find ccd
find_package(ccd REQUIRED PATHS “C:/Program Files (x86)/libccd/lib/ccd”)
# find fcl
#find_package(fcl REQUIRED PATHS C:/Users/ZHOUYC/Desktop/contact/6.refer_project/fcl/build)
find_package(fcl REQUIRED PATHS “C:/Program Files (x86)/fcl/CMake”)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCASCADE_INCLUDE_DIR})
include_directories("D:/app/OpenCASCADE-7.4.0-vc14-64/opencascade-7.4.0/inc")
include_directories("D:/app/eigen-3.4.0")
include_directories(${aris_INCLUDE_DIRS})
include_directories(${fcl_INCLUDE_DIRS})
#include_directories(${ccd_INCLUDE_DIRS})
include_directories("C:/Program Files (x86)/fcl/include")
include_directories("C:/Program Files (x86)/libccd/include")

file(GLOB HEADERS
    ./src/visualization/*.h
    ./src/simulator/*.h
)
file(GLOB SOURCES
    main.cpp
    ./src/visualization/*.cpp
    ./src/simulator/*.cpp
    ./src/visualization/*.ui
)

set(RESOURCES images/image.qrc )
#message(${OpenCASCADE_DIR} )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad)
target_link_libraries(${PROJECT_NAME} ${OpenCASCADE_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} ${ccd_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${fcl_LIBRARIES})


if(UNIX)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES})
endif(WIN32)











