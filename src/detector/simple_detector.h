#ifndef SIMPLE_DETECTOR_H
#define SIMPLE_DETECTOR_H

#include "detector.h"
#include <aruco/aruco.h>

class SimpleDetector : public Detector
{
public:
    SimpleDetector(aruco::CameraParameters camera_parameters, double marker_size);

    std::vector<Marker> detect(const cv::Mat &) override;

private:
    aruco::CameraParameters camera_parameters_;
    aruco::MarkerDetector marker_detector_;
    double marker_size_;
    std::vector<aruco::Marker> markers_;
};

#endif
