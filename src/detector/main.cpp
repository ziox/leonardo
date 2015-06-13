#include "simple_detector.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <cv.h>

#include <geometry_msgs/TransformStamped.h>

#include <fstream>
#include <string>
#include <vector>
#include <memory>

void processImage(const sensor_msgs::ImageConstPtr & msg);
aruco::CameraParameters loadCameraCalibration(const char * camera_calibration_url);


namespace
{

std::unique_ptr<Detector> the_detector;
ros::Publisher the_marker_publisher;

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "marker_detector");
    ROS_INFO("MarkerDetector initialized");

    ros::NodeHandle handle("~");

    std::string camera_calibration_url;
    std::string ros_home(getenv("HOME"));
    handle.param<std::string>("calibration_url", camera_calibration_url, ros_home + "/.ros/camera_info/camera.yaml");
    aruco::CameraParameters camera_parameters = loadCameraCalibration(camera_calibration_url.c_str());

    double marker_size;
    handle.param<double>("marker_size", marker_size, 0.05);

    ROS_INFO("marker size: %f", marker_size);
    the_detector = std::unique_ptr<Detector>(new SimpleDetector(camera_parameters, marker_size));

    image_transport::ImageTransport it(handle);
    image_transport::Subscriber image_topic = it.subscribe("camera", 1, processImage);

    the_marker_publisher = handle.advertise<geometry_msgs::TransformStamped>("markers", 100);

    ros::spin();

    return 0;
}

void processImage(const sensor_msgs::ImageConstPtr & msg)
{
    static tf::TransformBroadcaster tf_broadcaster;
    cv_bridge::CvImagePtr image_bridge = cv_bridge::toCvCopy(msg);
    for (auto & marker : the_detector->detect(image_bridge->image)) {
        cv::Mat rot(3, 3, CV_32FC1);
        cv::Rodrigues(marker.orientation, rot);

        tf::Matrix3x3 tf_rotation(
            rot.at<float>(0, 0), rot.at<float>(0, 1), rot.at<float>(0, 2),
            rot.at<float>(1, 0), rot.at<float>(1, 1), rot.at<float>(1, 2),
            rot.at<float>(2, 0), rot.at<float>(2, 1), rot.at<float>(2, 2));

        tf::Vector3 tf_translation(
            marker.position.at<float>(0),
            marker.position.at<float>(1),
            marker.position.at<float>(2));

        tf::StampedTransform tf_transform(
            tf::Transform(tf_rotation, tf_translation),
            ros::Time::now(),
            "camera",
            marker.id);

        geometry_msgs::TransformStamped transform;
        tf::transformStampedTFToMsg(tf_transform, transform);

        the_marker_publisher.publish(transform);
        tf_broadcaster.sendTransform(tf_transform);
    }
}

bool exists(const char * file_name)
{
    return std::ifstream(file_name).good();
}

aruco::CameraParameters loadCameraCalibration(const char * camera_calibration_url)
{
    aruco::CameraParameters camera_parameters;
    if (exists(camera_calibration_url)) {
        try {
            ROS_INFO("Using camera calibration file: %s", camera_calibration_url);
            camera_parameters.readFromXMLFile(camera_calibration_url);
        } catch (std::exception & e) {
            ROS_WARN("Unable to load calibration file: %s", e.what());
        }
    } else {
        ROS_WARN("Camera calibration file %s NOT found!", camera_calibration_url);
    }

    return camera_parameters;
}
