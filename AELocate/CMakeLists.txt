# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.0)
# 项目信息
project (AELocate)
# Eigen 库
# find_package (Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)
# 指定生成目标
add_executable(AELocate ${DIR_SRCS})
# target_link_libraries (AELocate Eigen3::Eigen)