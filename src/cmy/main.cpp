#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <iostream>
#include <utils.hpp>
#include <spdlog/spdlog.h>
#include <signal.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <cmy_detector.hpp>
#include <dji_flight_control.hpp>

using namespace DJI::OSDK;

static const std::string config_path = "/home/dji/WorkSpace/m210-2020-match/config/m210.conf";

std::atomic<bool> gRun{true};
std::atomic<bool> dRun{true};
std::atomic<bool> tRun{true};
cv::Mat frame;
std::mutex m_frame;

CmyDetector detector;

void sig_handler(int signal)
{
    std::cout << "request gateway stop\n";
    gRun = false;
}

void detectFrame(Vehicle *vehicle)
{
    std::cout << "开始进行检测" << std::endl;
    while (dRun.load())
    {
        m_frame.lock();
        cv::Mat _frame = frame.clone();
        m_frame.unlock();
        // std::cout << "开始识别" << std::endl;
        detector.update(_frame.clone());
        // std::cout << "识别完成" << std::endl;
        // 对识别的结果按置信度从大到小进行排序
        std::sort(std::begin(detector.boxes), std::end(detector.boxes), [](tk::dnn::box box1, tk::dnn::box box2) {
            return box1.prob > box1.prob;
        });
        int n_res = detector.boxes.size() > 12 ? 12 : detector.boxes.size();
        int _size = 1 + 1 + 2 * 4 * n_res;
        uint8_t *data = new uint8_t[_size];
        data[0] = 0;
        data[1] = n_res;
        int index = 2;
        for (int i = 0; i < n_res; ++i)
        {
            uint16_t x = static_cast<int>(detector.boxes[i].x);
            uint16_t y = static_cast<int>(detector.boxes[i].y);
            uint16_t w = static_cast<int>(detector.boxes[i].w);
            uint16_t h = static_cast<int>(detector.boxes[i].h);
            // std::cout << "x： " << static_cast<int>(detector.boxes[i].x) << ", "
            //           << "y： " << static_cast<int>(detector.boxes[i].y)<< ", "
            //           << "w： " << static_cast<int>(detector.boxes[i].w) << ", "
            //           << "h： " << static_cast<int>(detector.boxes[i].h) << std::endl;

            data[index++] = x >> 8;
            data[index++] = x;
            data[index++] = y >> 8;
            data[index++] = y;
            data[index++] = w >> 8;
            data[index++] = w;
            data[index++] = h >> 8;
            data[index++] = h;
        }
        // std::cout << "识别结果的个数" << n_res << std::endl;
        // std::cout << "开始发送识别结果" << std::endl;
        vehicle->moc->sendDataToMSDK(data, _size);
        delete[] data;
        usleep(20 * 1000);
    }
    std::cout << "识别任务结束" << std::endl;
}

void tracking(Vehicle *vehicle, cv::Rect roi)
{
    std::cout << "开始追踪" << std::endl;
    auto tracker = cv::TrackerKCF::create();
    m_frame.lock();
    cv::Mat _frame = frame.clone();
    m_frame.unlock();
    cv::Rect2d _roi = {static_cast<double>(roi.x), static_cast<double>(roi.y), static_cast<double>(roi.width), static_cast<double>(roi.height)};
    tracker->init(_frame, _roi);
    const int frameCenterX = 640;
    const int frameCenterY = 360;

    uint8_t flag = (Control::StableMode::STABLE_ENABLE |
                    Control::HorizontalCoordinate::HORIZONTAL_BODY |
                    Control::YawLogic::YAW_ANGLE |
                    Control::VerticalLogic::VERTICAL_VELOCITY |
                    Control::HorizontalLogic::HORIZONTAL_VELOCITY);

    while (tRun.load())
    {
        m_frame.lock();
        cv::Mat _frame = frame.clone();
        m_frame.unlock();
        cv::Rect2d newRoi;
        auto update_bf = std::chrono::system_clock::now();
        tracker->update(_frame, newRoi);
        auto update_af = std::chrono::system_clock::now();
        // std::ostringstream oss;
        // oss << "kcf detect cost time is " << std::fixed << std::setprecision(2) << std::chrono::duration<double, std::milli>(update_af - update_bf).count() << "ms";
        // std::cout << oss.str() << std::endl;
        int x = static_cast<int>(newRoi.x);
        int y = static_cast<int>(newRoi.y);
        int w = static_cast<int>(newRoi.width);
        int h = static_cast<int>(newRoi.height);

        // 根据距离帧中心的像素距离设置flightCtrl参数
        int objectCenterX = x + w / 2;
        int objectCenterY = y + h / 2;

        int deltaX = objectCenterX - frameCenterX;
        int deltaY = -(objectCenterY - frameCenterY);

        float left_right_cmd = std::abs(deltaX / 50.0) > 2 ? deltaX / std::abs(deltaX) * 2 : deltaX / 50.0;
        float forword_back_cmd = std::abs(deltaY / 50.0) > 2 ? deltaY / std::abs(deltaY) * 2 : deltaY / 50.0;

        std::cout << "forward_back_cmd: " << forword_back_cmd << std::endl;
        std::cout << "left_right_cmd: " << left_right_cmd << std::endl;

        // 控制飞行器飞向汽车
        Control::CtrlData ctrlData(flag, forword_back_cmd, left_right_cmd, 30, 0);
        vehicle->control->flightCtrl(ctrlData);
        uint8_t *data = new uint8_t[9];
        data[0] = 1;
        data[1] = x >> 8;
        data[2] = x;
        data[3] = y >> 8;
        data[4] = y;
        data[5] = w >> 8;
        data[6] = w;
        data[7] = h >> 8;
        data[8] = h;

        vehicle->moc->sendDataToMSDK(data, 9);
        delete[] data;
    }
}

void recvFrame(std::unique_ptr<Vehicle> &vehicle)
{
    while (gRun.load())
    {
        if (vehicle->advancedSensing->newMainCameraImageReady())
        {
            CameraRGBImage img{};
            if (vehicle->advancedSensing->getMainCameraImage(img))
            {
                m_frame.lock();
                frame = cv::Mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
                m_frame.unlock();
                usleep(20 * 1000);
            }
        }
    }
}

void parseFromMobileCallback(Vehicle *vehicle, RecvContainer recvFrame,
                             UserData userData)
{
    // 收到的前两个字节作为data_id表示要执行的操作
    uint8_t mobile_data_id = recvFrame.recvData.raw_ack_array[0];
    switch (mobile_data_id)
    {
    // 测试联通性
    case 'a':
    {
        uint8_t data[] = {'a'};
        vehicle->moc->sendDataToMSDK(data, 1);
    }
    break;
    // 起飞
    case 'b':
    {
        monitoredTakeoff(vehicle);
        uint8_t data[] = {'b'};
        vehicle->moc->sendDataToMSDK(data, 1);
    }
    break;
    // 上升指定的高度
    case 'c':
    {
        uint8_t data1[] = {'c'};
        vehicle->moc->sendDataToMSDK(data1, 1);
        moveByPositionOffset(vehicle, 0, 0, 30, 0);
        uint8_t data2[] = {'C'};
        vehicle->moc->sendDataToMSDK(data2, 1);
    }
    break;
    // 开始识别
    case 'd':
    {
        std::cout << "开始识别" << std::endl;
        dRun.store(true);
        uint8_t data[] = {'d'};
        vehicle->moc->sendDataToMSDK(data, 1);
        std::thread detect_thread{detectFrame, vehicle};
        detect_thread.detach();
    }
    break;
    // 开始跟踪
    case 'e':
    {
        std::cout << "接收开始追踪的指令" << std::endl;
        // 停止识别
        dRun.store(false);
        tRun.store(true);
        uint8_t data[] = {'e'};
        vehicle->moc->sendDataToMSDK(data, 1);
        std::cout << "receive data is ";
        for (int i = 1; i < 9; ++i)
        {
            std::cout << recvFrame.recvData.raw_ack_array[i] << " ";
        }
        std::cout << std::endl;
        int x = (0 | (recvFrame.recvData.raw_ack_array[1] << 8) | recvFrame.recvData.raw_ack_array[2]);
        int y = (0 | (recvFrame.recvData.raw_ack_array[3] << 8) | recvFrame.recvData.raw_ack_array[4]);
        int w = (0 | (recvFrame.recvData.raw_ack_array[5] << 8) | recvFrame.recvData.raw_ack_array[6]);
        int h = (0 | (recvFrame.recvData.raw_ack_array[7] << 8) | recvFrame.recvData.raw_ack_array[8]);
        std::cout << x << " " << y << " " << w << " " << h << std::endl;
        cv::Rect2d roi = {static_cast<double>(x), static_cast<double>(y), static_cast<double>(w), static_cast<double>(h)};
        std::thread tracking_thread{tracking, vehicle, roi};
        tracking_thread.detach();
    }
    break;
    // 结束跟踪
    case 'f':
    {
        tRun.store(false);
        uint8_t data[] = {'f'};
        vehicle->moc->sendDataToMSDK(data, 1);
        gRun.store(false);
        tRun.store(false);
        dRun.store(false);
    }
    break;
    // 返航
    case 'g':
    {
        uint8_t data[] = {'g'};
        vehicle->moc->sendDataToMSDK(data, 1);
        vehicle->control->goHome();
    }
    break;
    // 降落
    case 'h':
    {
        uint8_t data[] = {'h'};
        vehicle->moc->sendDataToMSDK(data, 1);
        monitoredLanding(vehicle);
    }
    break;
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, sig_handler);

    // 初始化日志接收器
    auto config = getConfig(config_path);
    initLogger(config.log_path, "logger");

    detector.init(config.net, config.n_classes);

    std::unique_ptr<Vehicle> vehicle = getVehicle(argc, argv);
    spdlog::get("logger")->info("初始化视频流接收器 ... ");
    if (!vehicle->advancedSensing->startMainCameraStream())
    {
        spdlog::get("logger")->error("启动主摄像头失败，退出程序！");
        throw std::runtime_error("摄像头开启失败！");
    }
    while (!vehicle->advancedSensing->newMainCameraImageReady())
        ;
    spdlog::get("logger")->info("视频流接收器初始化完成！");

    std::thread recvFrameThread(recvFrame, std::ref(vehicle));
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback);
    // 告知mobile端， 程序启动成功！
    uint8_t data[] = {3};
    vehicle->moc->sendDataToMSDK(data, 1);
    recvFrameThread.join();

    vehicle->advancedSensing->stopMainCameraStream();
    return 0;
}