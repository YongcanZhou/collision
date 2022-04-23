# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )
 
# 声明一个 cmake 工程
project( HelloSLAM )
 
# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
 
# 添加一个可执行程序
add_executable( main main.cpp )

# 添加一个静态库
add_library( add add.cpp )

target_link_libraries(main add)