cmake_minimum_required(VERSION 3.12)

project(Robot_OCC)
#for assimp import stl
add_compile_options(-bigobj)
# use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#find QT
if (WIN32)
set(Qt5Core_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Core)
set(Qt5Widgets_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Widgets)
set(Qt5Gui_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Gui)
set(Qt5Gamepad_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5Gamepad)
set(Qt5PrintSupport_DIR D:/app/Qt/5.12.12/msvc2017_64/lib/cmake/Qt5PrintSupport)
endif(WIN32)
 if (UNIX)
 set(Qt5Core_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Core)
 set(Qt5Widgets_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Widgets)
 set(Qt5Qt5Gui_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5Gui)
 set(Qt5Gamepad_DIR /opt/Qt5.11.2/5.11.HEADER2/gcc_64/lib/cmake/Qt5Gamepad)
 set(Qt5PrintSupport_DIR /opt/Qt5.11.2/5.11.2/gcc_64/lib/cmake/Qt5PrintSupport)
 endif(UNIX)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Gamepad REQUIRED)
find_package(Eigen3 REQUIRED)

# find OCC
set(TARGET_OPENCASCADE_PATH "D:/app/OpenCASCADE-7.4.0-vc14-64" CACHE PATH "OpenCASCADE install path")
if(EXISTS ${TARGET_OPENCASCADE_PATH})
        message(STATUS "Directory to search OCC at ${TARGET_OPENCASCADE_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_OPENCASCADE_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_OPENCASCADE_PATH not exists!")
endif()
find_package(OpenCASCADE REQUIRED)

# find VTK
set(TARGET_VTK_PATH "C:/Program Files (x86)/VTK" CACHE PATH "VTK install path")
if(EXISTS ${TARGET_VTK_PATH})
       message(STATUS "Directory to search VTK at ${TARGET_VTK_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_VTK_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_VTK_PATH not exists!")
endif()
find_package(VTK REQUIRED)

# find aris
if(UNIX)
        add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        # find_package(aris REQUIRED PATHS C:/aris/aris-2.2.1.220620/share/aris/cmake)
        find_package(aris REQUIRED PATHS C:/aris/aris-2.3.2.220803/share/aris/cmake)
endif(WIN32)

# find assimp
set(TARGET_ASSIMP_PATH "C:/Program Files (x86)/Assimp" CACHE PATH "Assimp install path")
if(EXISTS ${TARGET_ASSIMP_PATH})
    message(STATUS "Directory to search fcl at ${TARGET_ASSIMP_PATH}")
    list(APPEND CMAKE_PREFIX_PATH ${TARGET_ASSIMP_PATH})
else()
    message(WARNING "File/Directory at variable TARGET_ASSIMP_PATH not exists!")
endif()
find_package(assimp REQUIRED)

# find ccd
set(TARGET_CCD_PATH "C:/Program Files (x86)/libccd" CACHE PATH "CCD install path")
if(EXISTS ${TARGET_CCD_PATH})
    message(STATUS "Directory to search fcl at ${TARGET_CCD_PATH}")
    list(APPEND CMAKE_PREFIX_PATH ${TARGET_CCD_PATH})
else()
    message(WARNING "File/Directory at variable TARGET_CCD_PATH not exists!")
endif()
find_package(ccd REQUIRED)

# find fcl
set(TARGET_FCL_PATH "C:/Program Files (x86)/fcl" CACHE PATH "Fcl install path")
if(EXISTS ${TARGET_FCL_PATH})
        message(STATUS "Directory to search fcl at ${TARGET_FCL_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_FCL_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_FCL_PATH not exists!")
endif()
find_package(fcl REQUIRED)

# find aris_sim
set(TARGET_ARIS_SIM_PATH "C:/aris_sim/aris_sim-1.0" CACHE PATH "Aris_sim install path")
if(EXISTS ${TARGET_ARIS_SIM_PATH})
        message(STATUS "Directory to search aris_sim at ${TARGET_ARIS_SIM_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_ARIS_SIM_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_ARIS_SIM_PATH not exists!")
endif()
find_package(aris_sim REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCASCADE_INCLUDE_DIR})
include_directories("D:/app/OpenCASCADE-7.4.0-vc14-64/opencascade-7.4.0/inc")
include_directories("D:/app/eigen-3.4.0")
include_directories(${aris_INCLUDE_DIRS})
include_directories(${fcl_INCLUDE_DIRS})
include_directories(${ccd_INCLUDE_DIRS})
include_directories(${air_sim_INCLUDE_DIRS})

file(GLOB HEADERS
    ./src/visualization/*.h
    ./src/simulator/*.h)
file(GLOB SOURCES
    main.cpp
    ./src/visualization/*.cpp
    ./src/simulator/*.cpp
    ./src/visualization/*.ui)

set(RESOURCES images/image.qrc )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad)
target_link_libraries(${PROJECT_NAME} ${OpenCASCADE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${ccd_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${fcl_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${aris_sim_LIBRARIES})

if(UNIX)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES})
endif(WIN32)











