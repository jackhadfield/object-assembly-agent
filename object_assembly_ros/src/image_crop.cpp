#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <vector> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>

using namespace cv;

class ImageCropNode{

public:
    ImageCropNode(int crop_height,
                  int crop_width,
                  int crop_top,
                  int crop_left,
                  double scaling_factor,
                  std::string new_type);
    void CameraInfoCallback(const sensor_msgs::CameraInfo &cinfo);
    void callback(const sensor_msgs::ImageConstPtr& image);

private:
    int height_;
    int width_;
    bool image_received_ = false;
    bool cinfo_set_ = false;
    int crop_height_;
    int crop_width_;
    int crop_top_;
    int crop_left_;
    double scaling_factor_;
    std::string new_type_;
    ros::NodeHandle node_handle_;
    ros::Publisher cropped_image_publisher_;
    ros::Publisher newcamerainfo_publisher_;

    sensor_msgs::CameraInfo cinfo_;
};

ImageCropNode::ImageCropNode(int crop_height,
                             int crop_width,
                             int crop_top,
                             int crop_left,
                             double scaling_factor,
                             std::string new_type) : node_handle_("~"),
                                                     crop_height_(crop_height),
                                                     crop_width_(crop_width),
                                                     crop_top_(crop_top),
                                                     crop_left_(crop_left),
                                                     scaling_factor_(scaling_factor),
                                                     new_type_(new_type)
{
    cropped_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("cropped_image", 0);
    newcamerainfo_publisher_ = node_handle_.advertise<sensor_msgs::CameraInfo>("newCameraInfo", 0);
}

void ImageCropNode::CameraInfoCallback(const sensor_msgs::CameraInfo &cinfo) {
    if (image_received_) {
        if (!cinfo_set_) {
            cinfo_ = cinfo;
            cinfo_.P[2] = width_/2 - crop_left_ - 0.5;
            cinfo_.P[6] = height_/2 - crop_top_ - 0.5;
            cinfo_.K[2] = cinfo_.P[2];
            cinfo_.K[5] = cinfo_.P[6];
            cinfo_.height = crop_height_;
            cinfo_.width = crop_width_;
            cinfo_set_ = true;
        }
        newcamerainfo_publisher_.publish(cinfo_);
    }
}

void ImageCropNode::callback(
        const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.",
                  image->encoding.c_str());
    }
    image_received_ = true;
    height_ = cv_ptr->image.rows;
    width_ = cv_ptr->image.cols;
    if (crop_left_+crop_width_ > width_ || crop_top_+crop_height_ > height_)
        ROS_ERROR("Invalid crop parameters");
    Rect r1(crop_left_, crop_top_, crop_width_, crop_height_);
    Mat img = cv_ptr->image(r1);
    cv_ptr->image = img;
    //img.copyTo(cv_ptr->image);
    cv_ptr->image.convertTo(cv_ptr->image, CV_32FC1, scaling_factor_);
    auto img_msg = cv_ptr->toImageMsg();
    img_msg->encoding = "32FC1";

    cropped_image_publisher_.publish(img_msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_crop");
    ros::NodeHandle nh("~");

    std::string image_topic;
    nh.getParam("image_topic", image_topic);
    std::string camera_info_topic;
    nh.getParam("camera_info_topic", camera_info_topic);

    std::string new_type;;
    nh.getParam("convert_to", new_type);

    int crop_height, crop_width, crop_top, crop_left;
    double scaling_factor;
    nh.getParam("crop_height", crop_height);
    nh.getParam("crop_width", crop_width);
    nh.getParam("crop_top", crop_top);
    nh.getParam("crop_left", crop_left);
    nh.getParam("scaling_factor", scaling_factor);

    if (crop_left < 0 || crop_width < 0 || crop_top < 0 || crop_height < 0 || scaling_factor < 0)
        ROS_ERROR("Negative crop parameters");

    ImageCropNode image_crop_node(crop_height, crop_width, crop_top, crop_left, scaling_factor, new_type);

    ros::Subscriber image_subscriber = 
        nh.subscribe(image_topic,
                     1,
                     &ImageCropNode::callback,
                     &image_crop_node);

    ros::Subscriber cinfosubscriber =
        nh.subscribe(camera_info_topic,
                     1,
                     &ImageCropNode::CameraInfoCallback,
                     &image_crop_node);

    ros::spin();
}
