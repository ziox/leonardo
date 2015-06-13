#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>


struct Marker
{
    std::string id;
    cv::Mat position;
    cv::Mat orientation;
};


struct Detector
{
    virtual ~Detector() = default;
    virtual std::vector<Marker> detect(const cv::Mat &) = 0;
};


#endif
