cmake_minimum_required(VERSION 3.0.0)
project(m210-2020-match)

add_subdirectory(tkDnn)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -g -O0")



################ 设置库配置文件查找路径 ################
# 设置查找xxxConfig.cmake路径
set(CMAKE_PREFIX_PATH 
    "${CMAKE_CURRENT_SOURCE_DIR}/third-party/osdk-3.9/lib/cmake/djiosdk"
    "${CMAKE_CURRENT_SOURCE_DIR}/third-party/spdlog/lib/cmake/spdlog"
    "/usr/local/lib/cmake/opencv4"
)
# 设置查找Findxxx.cmake的路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")
# 设置查找pkgconfig路径
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third-party/confuse/lib/pkgconfig")


################ 加载第三方库 ################
# 通过查找xxxConfig.cmake加载第三方库
find_package(DJIOSDK CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(PkgConfig)
# 通过查找Findxxx.cmake加载第三方库
# find_package(Darknet MODULE REQUIRED)
# 通过查找xxx.pc加载第三方库
pkg_search_module(CONFUSE REQUIRED libconfuse)

################ 设置源文件和头文件路径 ################
# 设置头文件路径
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/cmy"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc/dji"
    ${DJIOSDK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CONFUSE_INCLUDE_DIRS}
    # ${DARKNET_INCLUDE_DIRS}
)
# 设置源文件路径
aux_source_directory("${CMAKE_CURRENT_SOURCE_DIR}/src/cmy" CMYSRC)
aux_source_directory("${CMAKE_CURRENT_SOURCE_DIR}/src/dji" DJISRC)
set(SRC ${CMYSRC} ${DJISRC})

################ 设置并链接target ################
# 设置生成可执行文件
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin") 
add_executable(${PROJECT_NAME} ${SRC})
# 链接目标与第三方库
target_link_libraries(${PROJECT_NAME} 
    ${DJIOSDK_LIBRARIES} 
    ${OpenCV_LIBS}
    spdlog::spdlog
    ${CONFUSE_LIBRARIES}
    # ${DARKNET_LIBRARY}
    tkDNN
)

get_target_property(tkDnn_include tkDNN INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} PUBLIC ${tkDnn_include})
