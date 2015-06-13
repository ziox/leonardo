#ifndef SIMPLE_DETECTOR_H
#define SIMPLE_DETECTOR_H

#include <aruco/aruco.h>
#include <vector>
#include <string>

struct Marker
{
    std::string id;
    cv::Mat position;
    cv::Mat orientation;
};

class SimpleDetector
{
public:
    SimpleDetector(aruco::CameraParameters camera_parameters, double marker_size);

    std::vector<Marker> detect(const cv::Mat &);

private:
    aruco::CameraParameters camera_parameters_;
    aruco::MarkerDetector marker_detector_;
    double marker_size_;
    std::vector<aruco::Marker> markers_;
};

#endif
