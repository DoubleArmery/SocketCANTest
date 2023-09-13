# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.1)



# demo 代表项目名称
project (demo)

# 添加一个可执行程序，demo是可执行程序名称，main.cpp是源文件
add_executable(main main.c can_driver.c can_driver.h)
target_link_libraries(main pthread)