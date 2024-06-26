/*******************************************************************
*  File Name       : akaze_detector.cpp                            *
*  Version         : 1.0.0                                         *
*  Description     : A ROS2 node that subscribes to an image topic,*
*                    applies the AKAZE feature detector, and       *
*                    publishes the processed image with keypoints. *
*  Author          : Dean Rowlett                                  *
*  Target          : ROS2                                          *
*  Compiler        : gcc                                           *
*  IDE             : VSCode Version: 1.84.2                        *
*  Last Updated    : 26th June 2024                                *
*  Notes           : None                                          *
*******************************************************************/

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @class AKAZEDetectorNode
 * @brief A ROS2 node that subscribes to an image topic, applies the AKAZE feature detector, and publishes the processed image with keypoints.
 */
class AKAZEDetectorNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for AKAZEDetectorNode.
     *
     * Initialises the node, declares and retrieves parameters, sets up the subscriber and publisher, and starts the timer.
     */
    AKAZEDetectorNode();

private:
    /**
     * @brief Callback function for image subscription.
     * @param msg The received image message.
     *
     * This function is called whenever a new image message is received on the subscribed topic.
     * It stores the received image in the last_image_ member variable.
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Timer callback function for processing and publishing images.
     *
     * This function is called at a fixed rate defined by the timer.
     * It processes the last received image using the AKAZE feature detector and publishes the result.
     */
    void timerCallback();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  ///< Subscription to the image topic.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;        ///< Publisher for the processed image with keypoints.
    rclcpp::TimerBase::SharedPtr timer_;                                     ///< Timer for controlling processing frequency.
    sensor_msgs::msg::Image::SharedPtr last_image_;                          ///< Pointer to the last received image.
    double frequency_;                                                       ///< Processing frequency in Hz.
    std::string input_topic_;                                                ///< Name of the input image topic.
};

AKAZEDetectorNode::AKAZEDetectorNode() : Node("akaze_detector_node"), frequency_(60.0) {
    // Declare and initialise the frequency parameter with a default value
    this->declare_parameter<double>("frequency", frequency_);

    // Declare and initialise the input topic parameter with a default value
    this->declare_parameter<std::string>("input_topic", "image");

    // Retrieve the frequency parameter value from the parameter server
    this->get_parameter("frequency", frequency_);

    // Retrieve the input topic parameter value from the parameter server
    this->get_parameter("input_topic", input_topic_);

    // Set the QoS profile for the subscription to the sensor data quality of service
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());

    // Create a subscription to the input topic with the specified QoS profile
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, qos_profile, std::bind(&AKAZEDetectorNode::imageCallback, this, std::placeholders::_1)
    );

    // Create a publisher for the processed image with keypoints
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("akaze_image", qos_profile);

    // Create a timer to call the timerCallback function at the specified frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&AKAZEDetectorNode::timerCallback, this)
    );

    // Log a message indicating that the node has been started
    RCLCPP_INFO(this->get_logger(), "AKAZE Detector Node has been started.");
}

void AKAZEDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Store the received image message in the last_image_ member variable
    last_image_ = msg;
}

void AKAZEDetectorNode::timerCallback() {
    // Check if the last_image_ member variable is not null
    if (!last_image_) {
        return;
    }

    // Convert the received image message to an OpenCV image
    cv::Mat image = cv_bridge::toCvShare(last_image_, "bgr8")->image;
    if (image.empty()) {
        // Log an error message if the image is empty
        RCLCPP_ERROR(this->get_logger(), "Received an empty image!");
        return;
    }

    // Declare variables to store the keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Create an AKAZE feature detector
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    // Detect and compute the keypoints and descriptors
    akaze->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

    // Log the number of detected keypoints
    RCLCPP_INFO(this->get_logger(), "Detected %zu keypoints.", keypoints.size());

    // Draw the keypoints on the image
    cv::Mat output_image;
    cv::drawKeypoints(image, keypoints, output_image);

    // Convert the processed OpenCV image to a ROS image message
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
        last_image_->header, "bgr8", output_image).toImageMsg();

    // Publish the processed image message
    publisher_->publish(*output_msg);
}

/**
 * @brief Main function that initialises and runs the AKAZEDetectorNode.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
    // Initialise the ROS 2 system
    rclcpp::init(argc, argv);

    // Create and spin the AKAZEDetectorNode
    rclcpp::spin(std::make_shared<AKAZEDetectorNode>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();

    // Return success status
    return 0;
}
