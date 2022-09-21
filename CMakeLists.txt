cmake_minimum_required(VERSION 3.12)
project(Demo)

# use c++ 17 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(ARIS_INSTALL_PATH C:/aris/aris-2.2.0.220524)

# find QT
set(TARGET_QT_PATH "" CACHE PATH "QT install path")
# find Assimp
if(EXISTS ${TARGET_QT_PATH})
	message(STATUS "Directory to search PCL at ${TARGET_QT_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_QT_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_QT_PATH not exists!")
endif()
find_package(Qt5 COMPONENTS Core Widgets Gui Gamepad PrintSupport REQUIRED)

# find PCL
set(TARGET_PCL_PATH "" CACHE PATH "PCL install path")
if(EXISTS ${TARGET_PCL_PATH})
	message(STATUS "Directory to search PCL at ${TARGET_PCL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_PCL_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_PCL_PATH not exists!")
endif()
find_package(PCL REQUIRED)

# find VTK
# set(TARGET_VTK_PATH "" CACHE PATH "VTK install path")
# if(EXISTS ${TARGET_VTK_PATH})
# 	message(STATUS "Directory to search VTK at ${TARGET_VTK_PATH}")
# 	list(APPEND CMAKE_PREFIX_PATH ${TARGET_VTK_PATH})
# else()
# 	message(WARNING "File/Directory at variable TARGET_VTK_PATH not exists!")
# endif()
# find_package(VTK REQUIRED COMPONENTS vtkCommonCore vtkRenderingOpenGL2)
# 
# IF(VTK_FOUND)
#   message("found VTK. Version:" ${VTK_VERSION}. VTK_DIR: ${VTK_DIR}. VTK_INCLUDE: ${VTK_INCLUDE_DIR})
# ELSE(VTK_FOUND)
#   MESSAGE(FATAL_ERROR
#     "Cannot build the executable without VTK. Please set the                                                                                                   
# VTK_DIR")
# ENDIF(VTK_FOUND)
# message(STATUS "FUCK ${VTK_LIBRARIES}")

# find OpenCascade
set(TARGET_OCC_PATH "" CACHE PATH "OCC install path")
if(EXISTS ${TARGET_OCC_PATH})
	message(STATUS "Directory to search OCC at ${TARGET_OCC_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_OCC_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_OCC_PATH not exists!")
endif()

find_package(OpenCASCADE CONFIG REQUIRED)

# find aris
if(UNIX)
        add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS ARIS_INSTALL_PATH)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS ARIS_INSTALL_PATH)
endif(WIN32)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCASCADE_INCLUDE_DIR})
include_directories(${aris_INCLUDE_DIRS})

file(GLOB HEADERS
    ./src/visualization/*.h
)
file(GLOB SOURCES
    main.cpp
    ./src/visualization/*.cpp
    ./src/visualization/*.ui
)

set(RESOURCES images/image.qrc)
SET(CMAKE_VERBOSE_MAKEFILE ON) 

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad)
target_link_libraries(${PROJECT_NAME} ${OpenCASCADE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
message(${PCL_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} vtkCommonCore vtkRenderingOpenGL2)


if(UNIX)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES})
endif(WIN32)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${ARIS_INSTALL_PATH}/lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

