#pragma once
#include <string>
#include <dji_vehicle.hpp>
#include <opencv2/opencv.hpp>


std::string getVideoFileName();

/**
 * @brief initLogger 初始化日志接收器
 * 创建的logger对应标准输出sink和文件sink,这里的sink理解为日志接收器。两个sink可以分别设置不同的
 * 日志格式和日志级别。
 * @param path 日志文件的路径
 * @param name logger的名字
 * @note 使用时只需要引入#include <spdlog/spdlog.hpp>头文件，然后调用spdlog::get("logger's name")
 * 即可获取日志接收器对象。
 */
void initLogger(const std::string path, const std::string name);

/**
 * @brief The M210Config struct
 */
struct M210Config {
    std::string log_path;
    // Yolov4 tiny arguments
    // std::string configPath;
    // std::string weightsPath;
    // std::string metaDataPath;
    std::string net;
    int n_classes;
    int n_batch;
    float conf_thresh;
};

/**
 * @brief getConfig 获取配置文件内容填充的结构体
 * @param path 配置文件的路径
 * @return 配置文件的结构体表示
 * @note 配置文件的读取只会在该函数第一次被调用时进行。
 */
M210Config getConfig(const std::string path);

/**
 * @brief getVehicle 获取飞行器实例
 * @param argc 控制台参数个数
 * @param argv 控制台参数值
 * @return 飞行器实例
 * @note 官方示例代码中原有的LinuxSetup类的实例在销毁时会对飞行器实例进行delete操作
 * 为了获得飞行器实例的所有权，我将LinuxSetup析构函数中的delete vehicle;语句注释掉，转而由
 * 我们在合适的时机进行delete
 */
std::unique_ptr<Vehicle> getVehicle(int argc, char **argv);

/**
 * @brief vsr_init 初始化视频流接收器
 * @param vehicle 飞行器实例
 */
void vsr_init(Vehicle* vehicle);
/**
 * @brief vsr_receive 接收视频流
 * 直接从摄像头拿到的帧的颜色格式为rgb,为了匹配opencv的颜色格式，对拿到的帧进行了rgb2bgr的转换。
 * 为了记录飞行了整个过程同时为了对帧的处理过程阻塞帧的获取，视频流获取过程运行在额外的线程中。
 * 采用了互斥量保证了对最新帧的读写不冲突。同时使用条件变量通知其他线程有新的帧到来。
 * @param vehicle 飞行器实例
 */
void vsr_receive(Vehicle* vehicle);
/**
 * @brief vsr_stop 停止接收视频流
 * 通过改变原子变量的值通知视频流接收线程停止执行
 */
void vsr_stop();
/**
 * @brief vsr_get 获取最新帧
 * @param mat 存储最新帧的复制文本
 */
void vsr_get(cv::Mat& mat);
