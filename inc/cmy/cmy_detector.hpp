#pragma

#include <string>
#include <Yolo3Detection.h>

class CmyDetector {
    tk::dnn::Yolo3Detection detector;
    std::string cost_time;
public:
    void init(const std::string& net, int n_classes,  int n_batch = 1, float conf_thresh = 0.3);
    void update(const cv::Mat& frame);
    void draw(cv::Mat& frame);

public:
    std::vector<tk::dnn::box> boxes;
};