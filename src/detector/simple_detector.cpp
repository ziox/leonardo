#include "simple_detector.h"

SimpleDetector::SimpleDetector(aruco::CameraParameters camera_parameters, double marker_size)
    : camera_parameters_(camera_parameters)
    , marker_size_(marker_size)
{
}

std::vector<Marker> SimpleDetector::detect(const cv::Mat & image)
{
    markers_.clear(); // Premature optimization? I know, I'm sorry!
    marker_detector_.detect(image, markers_, camera_parameters_, marker_size_);

    std::vector<Marker> detected;
    for (auto & marker : markers_) {
        Marker m;
        m.id = std::string("marker_") + std::to_string(marker.id);
        m.position = marker.Tvec;
        m.orientation = marker.Rvec;
        detected.push_back(m);
    }

    return detected;
}
