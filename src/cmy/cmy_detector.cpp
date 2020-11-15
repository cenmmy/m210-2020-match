#include "cmy_detector.hpp"

void CmyDetector::init(const std::string &net, int n_classes, int n_batch, float conf_thresh)
{
    bool isInit = detector.init(net, n_classes, n_batch, conf_thresh);
    if (!isInit)
    {
        std::cout << "分类器初始化失败!" << std::endl;
    }
}

void CmyDetector::update(const cv::Mat &frame)
{
    std::vector<cv::Mat> _frames;
    _frames.push_back(frame.clone());
    auto detect_bf = std::chrono::high_resolution_clock::now();
    detector.update(_frames, 1);
    auto detect_af = std::chrono::high_resolution_clock::now();
    std::ostringstream oss{};
    oss << "cost: " << std::fixed << std::setprecision(2) << (detect_af - detect_bf).count() / 1000000.0 << " ms.";
    cost_time = oss.str();
    boxes = detector.batchDetected[0];
}

void CmyDetector::draw(cv::Mat &frame)
{
    std::vector<cv::Mat> _frames = {frame};
    detector.draw(_frames);
    cv::putText(frame, cost_time, cv::Point(10, 30), 0, 1.0, cv::Scalar(0, 0, 255), 2, 16, false);
}