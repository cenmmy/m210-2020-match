#include <utils.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <confuse.h>
#include <iostream>
#include <dji_linux_helpers.hpp>


std::string getTimePrefix() {
    time_t tt = time(NULL);
    tm *t = localtime(&tt);
    std::string timePrefix
            = "[" + std::to_string(t->tm_hour) + ":" +
              std::to_string(t->tm_min) + ":" +
              std::to_string(t->tm_sec) + "]";
    t = nullptr;
    return timePrefix;
}

std::string getVideoFileName() {
    return getTimePrefix() + "-m210.avi";
}

/**
 * @brief getLogFileName 获取日志文件名
 * 日志文件名的命名规则为当前时间的时分秒与特定后缀的组合
 * 例： [09:33:52]-m210.log
 * @return 生成的日志文件名
 */
std::string getLogFileName()
{
    return getTimePrefix() + "-m210.log";
}

/**
 * @brief initLogger 初始化日志接收器
 * 创建的logger对应标准输出sink和文件sink,这里的sink理解为日志接收器。两个sink可以分别设置不同的
 * 日志格式和日志级别。
 * @param path 日志文件的路径
 * @param name logger的名字
 * @note 使用时只需要引入#include <spdlog/spdlog.hpp>头文件，然后调用spdlog::get("logger's name")
 * 即可获取日志接收器对象。
 */
void initLogger(const std::string path, const std::string name) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[%H:%M:%S] [%^%L%$] %v");
    // truncate： 是否清空日志文件中的内容
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path + getLogFileName(), true);
    file_sink->set_level(spdlog::level::info);

    std::vector<spdlog::sink_ptr> sinks {file_sink, console_sink};
    auto logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    // 创建的logger需要进行注册，注册成功之后，该logger通过spdlog::get在项目的任意位置访问。
    spdlog::register_logger(logger);
}

/**
 * @brief getConfig 获取配置文件内容填充的结构体
 * @param path 配置文件的路径
 * @return 配置文件的结构体表示
 * @note 配置文件的读取只会在该函数第一次被调用时进行。
 */
M210Config getConfig(const std::string path) {
    // 静态局部变量在函数第一次执行时初始化，在进程结束时析构
    static M210Config config {
        "/home/dji/WorkSpace/m210-2020-match/log/"
    };
    static bool isConfigFileRead = false;
    if (isConfigFileRead)
        return config;
    isConfigFileRead = true;
    cfg_opt_t opts[] = {
        CFG_STR("log_path", "/home/dji/WorkSpace/m210-2020-match/log/", CFGF_NONE),
        // Yolov4tiny 参数
        // CFG_STR("configPath", "/home/dji/WorkSpace/m210-2020-match/yolo/yolov4-tiny.cfg", CFGF_NONE),
        // CFG_STR("weightsPath", "/home/dji/WorkSpace/m210-2020-match/yolo/yolov4-tiny.weights", CFGF_NONE),
        // CFG_STR("metaDataPath", "/home/dji/WorkSpace/m210-2020-match/yolo/coco.data", CFGF_NONE),
        CFG_STR("net", "/home/dji/WorkSpace/m210-2020-match/tensor_rt/car.rt", CFGF_NONE),
        CFG_INT("n_classes", 2, CFGF_NONE),
        CFG_INT("n_batch", 1, CFGF_NONE),
        CFG_FLOAT("conf_thresh", 0.3, CFGF_NONE),
        CFG_END()
    };
    cfg_t* cfg = cfg_init(opts, CFGF_NONE);
    if (cfg_parse(cfg, path.c_str()) == CFG_PARSE_ERROR){
        std::cout << "配置文件解析失败，请检查文件格式！当前使用默认配置。" << std::endl;
        return config;
    }
    config.log_path = cfg_getstr(cfg, "log_path");
    // Yolov4tiny 参数配置
    // config.configPath = cfg_getstr(cfg, "configPath");
    // config.weightsPath = cfg_getstr(cfg, "weightsPath");
    // config.metaDataPath = cfg_getstr(cfg, "metaDataPath");
    config.net = cfg_getstr(cfg, "net");
    config.n_classes = cfg_getint(cfg, "n_classes");
    config.n_batch = cfg_getint(cfg, "n_batch");
    config.conf_thresh = cfg_getfloat(cfg, "conf_thresh");

    return config;
}

std::unique_ptr<Vehicle> getVehicle(int argc, char **argv) {
    spdlog::get("logger")->info("初始化linux环境！");
    LinuxSetup linuxEnvironment(argc, argv, true);
    Vehicle *vehicle = linuxEnvironment.getVehicle();
    const char *acm_dev =
    linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
    vehicle->advancedSensing->setAcmDevicePath(acm_dev);
    spdlog::get("logger")->info("linux环境初始化完成！");

    if (vehicle == nullptr) {
        spdlog::get("logger")->error("飞行器初始化失败，退出程序!");
        throw std::runtime_error("飞行器初始化失败！");
    }
    return std::unique_ptr<Vehicle>(vehicle);
}

#include <condition_variable>
#include <mutex>

static std::condition_variable cond;
static std::mutex m_frame;
static cv::Mat frame;
static std::atomic<bool> _stop {false};

void vsr_init(Vehicle* vehicle) {
    spdlog::get("logger")->info("初始化视频流接收器 ... ");
    if (!vehicle->advancedSensing->startMainCameraStream()) {
        spdlog::get("logger")->error("启动主摄像头失败，退出程序！");
        throw std::runtime_error("摄像头开启失败！");
    }
    while(!vehicle->advancedSensing->newMainCameraImageReady());
    spdlog::get("logger")->info("视频流接收器初始化完成！");
}

/**
 * @brief vsr_receive 接收视频流
 * 直接从摄像头拿到的帧的颜色格式为rgb,为了匹配opencv的颜色格式，对拿到的帧进行了rgb2bgr的转换。
 * 为了记录飞行了整个过程同时为了对帧的处理过程阻塞帧的获取，视频流获取过程运行在额外的线程中。
 * 采用了互斥量保证了对最新帧的读写不冲突。同时使用条件变量通知其他线程有新的帧到来。
 * @param vehicle 飞行器实例
 */
void vsr_receive(Vehicle* vehicle) {
    while (!_stop.load()) {
        if (vehicle->advancedSensing->newMainCameraImageReady()) {
            CameraRGBImage img {};
            if (vehicle->advancedSensing->getMainCameraImage(img)) {
                cv::Mat temp_frame = cv::Mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
                cv::cvtColor(temp_frame, temp_frame, cv::COLOR_RGB2BGR);
                std::unique_lock<std::mutex> lock {m_frame};
                frame = temp_frame.clone();
                cond.notify_all();
                usleep(1000 * 5);
            } else {
                usleep(1000 * 1);
            }
        } else {
            usleep(1000 * 1);
        }
    }
}

void vsr_stop() {
    _stop.store(true);
}

void vsr_get(cv::Mat& mat) {
    std::unique_lock<std::mutex> lock {m_frame};
    cond.wait(lock);
    mat = frame.clone();
}
