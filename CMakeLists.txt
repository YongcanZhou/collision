cmake_minimum_required(VERSION 3.12)
project(Demo)

# use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#set(Boost_LIB_PREFIX "lib")
#set(Boost_ARCHITECTURE "-x64")
SET(CMAKE_EXE_LINKER_FLAGS /NODEFAULTLIB:"msvcrt.lib")

# find QT
set(TARGET_QT_PATH "" CACHE PATH "QT install path")
if(EXISTS ${TARGET_QT_PATH})
	message(STATUS "Directory to search QT at ${TARGET_QT_PATH}")
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
set(TARGET_VTK_PATH "" CACHE PATH "VTK install path")
if(EXISTS ${TARGET_VTK_PATH})
	message(STATUS "Directory to search VTK at ${TARGET_VTK_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_VTK_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_VTK_PATH not exists!")
endif()
find_package(VTK REQUIRED)

# find OpenCascade
set(TARGET_OCC_PATH "" CACHE PATH "OCC install path")
if(EXISTS ${TARGET_OCC_PATH})
	message(STATUS "Directory to search OCC at ${TARGET_OCC_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_OCC_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_OCC_PATH not exists!")
endif()
find_package(OpenCASCADE CONFIG REQUIRED)

# find hpp-fcl
set(TARGET_HPP_FCL_PATH "" CACHE PATH "Hpp-fcl install path")
if(EXISTS ${TARGET_HPP_FCL_PATH})
	message(STATUS "Directory to search hpp-fcl at ${TARGET_HPP_FCL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_HPP_FCL_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_HPP_FCL_PATH not exists!")
endif()
find_package(hpp-fcl REQUIRED)

# find aris
set(TARGET_ARIS_PATH "" CACHE PATH "Aris install path")
if(EXISTS ${TARGET_ARIS_PATH})
        message(STATUS "Directory to search aris at ${TARGET_ARIS_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_ARIS_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_ARIS_PATH not exists!")
endif()
find_package(aris REQUIRED)

# find sire
set(TARGET_SIRE_PATH "" CACHE PATH "sire install path")
if(EXISTS ${TARGET_SIRE_PATH})
        message(STATUS "Directory to search sire at ${TARGET_SIRE_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_SIRE_PATH})
else()
        message(WARNING "File/Directory at variable TARGET_SIRE_PATH not exists!")
endif()
find_package(sire REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCASCADE_INCLUDE_DIR})
# include_directories(${aris_INCLUDE_DIRS})
include_directories(${hpp-fcl_INCLUDE_DIRS})
include_directories(${sire_INCLUDE_DIRS})

file(GLOB HEADERS
    ./src/visualization/*.h
)
file(GLOB SOURCES
    main.cpp
    ./src/visualization/*.cpp
    ./src/visualization/*.ui
)

set(RESOURCES images/image.qrc)
set(CMAKE_VERBOSE_MAKEFILE ON) 

# message("aris_sim_path:"${TARGET_ARIS_SIM_PATH})
# message("cmake: ${CMAKE_BUILD_TYPE}")
# message(""${aris_sim_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_sim_lib.dll"")
# message($<TARGET_FILE_DIR:${PROJECT_NAME}>)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad)
target_link_libraries(${PROJECT_NAME} ${OpenCASCADE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} ${sire_LIBRARIES})

# STRING(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_LOW)
# message(${CMAKE_BUILD_TYPE_LOW})

 add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
     COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
         "${sire_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/sire_lib.dll"      # <--this is in-file
         $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
        "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path