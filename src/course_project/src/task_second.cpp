#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <image_handler.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "second_task_node");
    ros::NodeHandle n;
    Image_Handler ic(n);
    ros::spin();
    return 0;
}