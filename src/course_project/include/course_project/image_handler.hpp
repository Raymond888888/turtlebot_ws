
#ifndef _IMAGE_HANDLER_HPP
#define _IMAGE_HANDLER_HPP
#pragma once
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <mypid.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuple>

static const std::string OPENCV_WINDOW = "Image window";

enum robot_state {
    DETECT_ROAD,
    DETECT_IMAGE,
    STOP_ACTION
};

enum rotate_direction {
    NULL_ROTATION,
    LEFT_ROTATION,
    RIGHT_ROTATION
};

int area_calc(cv::Mat img) {
    int a = 0;
    for (int i = 1; i < img.rows; i++) {
        for (int j = 1; j < img.cols; j++) {
            if (img.at<uchar>(i, j)) a++;
        }
    }
    return a;
}

std::tuple<bool, float, float, float, float> get_red_centers(cv::Mat img) {
    std::tuple<bool, float, float, float, float> ret;

    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float a = 0;
    for (int i = 1; i < img.rows; i++) {
        for (int j = 1; j < img.cols / 2; j++) {
            if (img.at<uchar>(i, j)) {
                x1 += i;
                y1 += j;
                a++;
            }
        }
    }
    x1 /= a;
    y1 /= a;

    a = 0;
    for (int i = 1; i < img.rows; i++) {
        for (int j = img.cols / 2; j < img.cols; j++) {
            if (img.at<uchar>(i, j)) {
                x2 += i;
                y2 += j;
                a++;
            }
        }
    }
    x2 /= a;
    y2 /= a;

    if (0 < y1 && y1 < 320 && 320 < y2 && y2 < 640) {
        // ROS_INFO("TRUE!!!!!!!");
        ret = std::make_tuple(true, (int)y1, (int)x1, (int)y2, (int)x2);
    } else
        ret = std::make_tuple(false, (int)y1, (int)x1, (int)y2, (int)x2);
    return ret;
}
class Image_Handler {
   private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int area_red, area_blue;
    std::tuple<bool, float, float, float, float> result;
    bool status;
    int x1, y1, x2, y2;
    std::pair<int, int> red_p1, red_p2;
    pid_config config;
    pid cone_pid;

   public:
    cv::Mat image_cv;
    ros::Publisher vel_pub;

    float redcone_delta;
    int bluecone_delta;

    Image_Handler(ros::NodeHandle& nh_)
        : it_(nh_) {
        vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/d435/color/image_raw", 1,
                                   &Image_Handler::imageCb, this);
        image_pub_ = it_.advertise("/second_task_node/output_video", 1);

        config.KP = 0.01;
        config.KI = 0.000001;
        config.KD = 0.000005;
        config.error_max = 10;
        config.outputMax = 1;
        config.PID_Mode = PID_POSITION;

        PID_Init(&cone_pid, &config);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~Image_Handler() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        // Update GUI Window
        image_cv = cv_ptr->image;
        // cv::imshow(OPENCV_WINDOW, image_cv);
        // cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

        image_process();
    }

    void image_process() {
        std::vector<cv::Mat> channels;
        //    通道分离
        cv::split(image_cv, channels);

        //    分别得到不同的颜色分量
        cv::Mat bluecone, greencone, redcone;
        bluecone = channels.at(0);
        greencone = channels.at(1);
        redcone = channels.at(2);
        // cv::imshow("red", redcone);
        // cv::imshow("blue", bluecone);

        // TODO RED
        // mask
        cv::Mat mask = redcone.clone();
        for (int i = 1; i < redcone.rows / 3; i++) {
            for (int j = 1; j < redcone.cols; j++) {
                mask.at<uchar>(i, j) = 0;
            }
        }
        cv::Mat dst;
        // 直方图均衡化
        cv::equalizeHist(mask, dst);
        // 二值化
        // cv::adaptiveThreshold(dst,dst,128,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,3,0);
        cv::threshold(dst, dst, 240, 255, cv::THRESH_BINARY);

        // 开操作
        cv::Mat element;
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(dst, dst, cv::MORPH_OPEN, element);

        // 计算面积
        area_red = area_calc(dst);
        // ROS_INFO("area: %d", area_red);

        // 更新锥桶中心点坐标和状态
        result = get_red_centers(dst);
        std::tie(status, x1, y1, x2, y2) = result;

        red_p1 = std::make_pair(x1, y1);
        red_p2 = std::make_pair(x2, y2);

        cv::circle(image_cv, cv::Point(red_p1.first, red_p1.second), 10, CV_RGB(0, 255, 0));
        cv::circle(image_cv, cv::Point(red_p2.first, red_p2.second), 10, CV_RGB(0, 255, 0));
        // ROS_INFO("%d %d %d %d", red_p1.first, red_p1.second, red_p2.first, red_p2.second);
        // ROS_INFO("%d %d %d %d", x1, y1, x2, y2);
        redcone_delta = (x1 + x2) / 2.0 - 320;
        // ROS_INFO("redcone_delta %f", redcone_delta);
        // ROS_INFO(" status %d", (int)status);

        cv::imshow("reddst", dst);

        robot_rontrol();

        // TODO BLUE
        mask = bluecone.clone();
        for (int i = 1; i < bluecone.rows / 3; i++) {
            for (int j = 1; j < bluecone.cols; j++) {
                mask.at<uchar>(i, j) = 0;
            }
        }
        cv::equalizeHist(mask, dst);
        // cv::adaptiveThreshold(dst,dst,128,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,3,0);
        cv::threshold(dst, dst, 240, 255, cv::THRESH_BINARY);

        //    开操作
        cv::morphologyEx(dst, dst, cv::MORPH_OPEN, element);

        area_blue = area_calc(dst);
        cv::imshow("bluedst", dst);

        cv::imshow(OPENCV_WINDOW, image_cv);

        cv::waitKey(10);
    }

    void robot_rontrol() {
        // cmd speed publish
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;

        if (area_red > 100) {  // mode=red
            cone_pid.fdb = redcone_delta;
            cone_pid.ref = 0.0;
            PID_Calc(&cone_pid);

            // 调PID用 调完可注释
            // ROS_INFO("fdb %f ref %f  err %f output %f", cone_pid.fdb, cone_pid.ref, cone_pid.error[0], cone_pid.output);

            if (status) {
                cmd.linear.x = 0.2;

                cmd.angular.z = cone_pid.output;
            } else
                cmd.angular.z = 0;
        }

        vel_pub.publish(cmd);
    }
};

#endif