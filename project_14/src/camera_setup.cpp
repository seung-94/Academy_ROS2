#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraSetupNode : public rclcpp::Node
{
public:
    CameraSetupNode() : Node("camera_setup")
    {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_compressed", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CameraSetupNode::capture_and_publish, this));
    }

private:
    void capture_and_publish()
    {
        cv::Mat frame;
        // Capture image from camera (replace with actual camera capture code)
        // Example: frame = cv::imread("path/to/image.jpg");

        // For demonstration, let's assume we have an image from the camera
        frame = cv::imread("path/to/image.jpg"); // Replace with actual camera capture

        // Compress image to JPEG with compression level 8 or 9
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        std::vector<uchar> compressed_data;
        cv::imencode(".jpg", frame, compressed_data, compression_params);

        // Convert compressed data to sensor_msgs::Image
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->data = compressed_data;

        // Publish compressed image
        image_pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSetupNode>());
    rclcpp::shutdown();
    return 0;
}
