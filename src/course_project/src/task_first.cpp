// 2022.11.17-18
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>

using namespace std;
using namespace cv;

enum robot_state
{
    DETECT_ROAD,
    DETECT_IMAGE,
    STOP_ACTION
};

enum rotate_direction
{
    NULL_ROTATION,
    LEFT_ROTATION,
    RIGHT_ROTATION
};

// class Planner
// {
// public:
//     Planner(ros::NodeHandle &nh);
//     ~Planner();

// private:
//     ros::Publisher vel_pub;
//     // ros::Subscriber pose_sub;
//     VideoCapture video_device;
//     robot_state current_state;
//     rotate_direction rotate_cmd; // useless but the program looks more solid with it

//     void poseCallback(const nav_msgs::Odometry &pose_msg);
//     void DetectRoad(Mat &origin_frame, geometry_msgs::Twist &cmd);
//     void PlanSpeed(Mat &direction_frame, int direction_line[], geometry_msgs::Twist &cmd);
//     void IdentifyImage(Mat &origin_frame, geometry_msgs::Twist &cmd);
//     void MoveControl(Mat &origin_frame);
// };

// Planner::Planner(ros::NodeHandle &nh)
// {
//     ROS_INFO("Planner has started");
//     video_device.open(1);
//     vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
//     // pose_sub = nh.subscribe("/odom", 10, poseCallback);
    
//     // init the statement of robot
//     current_state = DETECT_ROAD;
//     rotate_cmd = NULL_ROTATION;

//     Mat road_image;
//     ////////////////////////////
//     while (true)
//     {
//         video_device.read(road_image);
//         MoveControl(road_image);
//         waitKey(10);
//     }
// }

// Planner::~Planner()
// {
//     ROS_INFO("Planner has terminated");
// }

// void Planner::poseCallback(const nav_msgs::Odometry &pose_msg)
// {
//     static double px, py;
//     px = pose_msg.pose.pose.position.x;
//     py = pose_msg.pose.pose.position.y;
//     return;
// }

// void Planner::MoveControl(Mat &origin_frame)
// {
//     geometry_msgs::Twist cmd;
//     cmd.linear.x = 0;
//     cmd.linear.y = 0;
//     cmd.linear.z = 0;
//     cmd.angular.x = 0;
//     cmd.angular.y = 0;
//     cmd.angular.z = 0;

//     imshow("origin_frame", origin_frame);

//     if (current_state == DETECT_ROAD)
//     {
//         DetectRoad(origin_frame, cmd);
//     }
//     if (current_state == DETECT_IMAGE)
//     {
//         IdentifyImage(origin_frame, cmd);
//     }

//     vel_pub.publish(cmd);
//     return;
// }

// void Planner::DetectRoad(Mat &origin_frame, geometry_msgs::Twist &cmd)
// {
//     GaussianBlur(origin_frame, origin_frame, Size(3, 3), 1);
//     Mat hsv_frame;
//     cvtColor(origin_frame, hsv_frame, COLOR_BGR2HSV);

//     // use the threshold to segment the range of blue roads
//     Scalar hsv_blue_min(100, 43, 46);
//     Scalar hsv_blue_max(124, 255, 255);
//     inRange(hsv_frame, hsv_blue_min, hsv_blue_max, hsv_frame);
//     morphologyEx(hsv_frame, hsv_frame, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(5, 5)));

//     // split the image from binary-view camera
//     Rect left_box(0, 0,
//                   static_cast<int>(origin_frame.cols / 2), static_cast<int>(origin_frame.rows));
//     Rect right_box(static_cast<int>(origin_frame.cols / 2), 0,
//                    static_cast<int>(origin_frame.cols / 2), static_cast<int>(origin_frame.rows));

//     Mat left_frame, right_frame;
//     left_frame = hsv_frame(left_box);
//     right_frame = hsv_frame(right_box);
//     // fill the image to get the better edge
//     floodFill(left_frame, Point(left_frame.cols / 2, left_frame.rows * 4 / 5), Scalar(255));
//     floodFill(right_frame, Point(right_frame.cols / 2, right_frame.rows * 4 / 5), Scalar(255));

//     Mat merge_frame;
//     bitwise_or(left_frame, right_frame, merge_frame);
//     imshow("test", merge_frame);

//     // compute the mid line of the two sides of road and display it through drawing the mat
//     Mat direction_frame = Mat::zeros(left_frame.size(), CV_8UC1); // display the direction for the car
//     Mat road_frame = Mat::zeros(left_frame.size(), CV_8UC1);
//     int mid_line[left_frame.rows][2] = {0}; // record the edge points' coordinate from the road
//     int direction_line[left_frame.rows] = {0};

//     // get the coordinate of points on edge
//     for (int i = left_frame.rows - 1; i > 0; i--)
//     {
//         for (int j = 0; j < left_frame.cols; j++)
//         {
//             if (left_frame.at<uint8_t>(i, j) == 255)
//             {
//                 mid_line[i][0] = j;
//                 circle(road_frame, Point(j, i), 1, Scalar(255), -1);
//                 break;
//             }
//         }

//         for (int j = right_frame.cols - 1; j > 0; j--)
//         {
//             if (right_frame.at<uint8_t>(i, j) == 255)
//             {
//                 mid_line[i][1] = j;
//                 circle(road_frame, Point(j, i), 1, Scalar(255), -1);
//                 break;
//             }
//         }
//     }

//     // calculate the average value of edge points
//     for (int i = 0; i < hsv_frame.rows; i++)
//     {
//         direction_line[i] = (mid_line[i][0] + mid_line[i][1]) / 2;
//         circle(direction_frame, Point(direction_line[i], i), 1, Scalar(255), -1);
//     }

//     imshow("road", road_frame);
//     imshow("direction", direction_frame);
//     PlanSpeed(merge_frame, direction_line, cmd);
//     return;
// }

// void Planner::PlanSpeed(Mat &merge_frame, int direction_line[], geometry_msgs::Twist &cmd)
// {
//     static float stop_conv = 0;
//     int row = merge_frame.rows;
//     int col = merge_frame.cols;

//     // determine the speed through collecting the average bias from the mid_line
//     int center_col = static_cast<int>(col / 2);
//     float direction_deviation = 0;

//     // accumulate the deviation value, used to specify the angular speed
//     for (int i = row / 2; i > 0; i--)
//     {
//         direction_deviation += (direction_line[i] - center_col) / 80;
//     }

//     int cross_count = 0; // judge whether come to the crossroad
//     for (int i = row; i > 0; i--)
//     {
//         if (direction_line[i] == center_col - 1)
//             cross_count++;
//     }

//     // if (cross_count >= row * 3 / 4 && cross_count < row)
//     if (cross_count >= row * 3 / 4)
//     {
//         stop_conv += 0.1;
//     }
//     // avoid the init empty image
//     if (stop_conv > 1)
//     {
//         stop_conv = 0;
//         if (cross_count < row)
//         {
//             current_state = DETECT_IMAGE;
//             ROS_INFO("find the crossroad");
//         }
//         // not working but don't want to change
//         if (cross_count == row)
//         {
//             current_state = STOP_ACTION;
//             ROS_INFO("come to the end");
//         }
//         return;
//     }

//     // specify the speed's value based on the prior information
//     cmd.linear.x = 0.2;
//     cmd.angular.z = direction_deviation * (-0.06);
//     cout << "deviation: " << direction_deviation << endl;
//     cout << "cmd.z: " << cmd.angular.z << endl;

//     return;
// }

// void Planner::IdentifyImage(Mat &origin_frame, geometry_msgs::Twist &cmd)
// {
//     Mat hsv_frame;
//     cvtColor(origin_frame, hsv_frame, COLOR_BGR2HSV);

//     Mat read_split;
//     Mat orange_split;

//     // hsv thresholds
//     Scalar hsv_orange_min(11, 43, 46);
//     Scalar hsv_orange_max(25, 255, 255);

//     Scalar hsv_red_min(156, 43, 46);
//     Scalar hsv_red_max(180, 255, 255);

//     inRange(hsv_frame, hsv_red_min, hsv_red_max, read_split);
//     inRange(hsv_frame, hsv_orange_min, hsv_orange_max, orange_split);

//     // detect the different images of pills through finding respective contours
//     Mat canvas = Mat::zeros(hsv_frame.size(), CV_8UC3);
//     vector<vector<Point>> contours[2];
//     vector<Vec4i> hierachy[2];
//     findContours(orange_split, contours[0], hierachy[0], RETR_TREE, CHAIN_APPROX_NONE);
//     findContours(read_split, contours[1], hierachy[1], RETR_TREE, CHAIN_APPROX_NONE);

//     int pill_count[2] = {0}; // record the numbers of the corresponding pill, 0 for elliptical pill, 1 for round pill
//     for (const auto &pill : contours[0])
//     {
//         // reduce the impact of noisy points by setting the area's threshold
//         if (contourArea(pill) > 300)
//         {
//             vector<vector<Point>> pill_contour{pill};
//             drawContours(canvas, pill_contour, -1, Scalar(255, 255, 0), -1);
//             pill_count[0]++;
//         }
//     }
//     for (const auto &pill : contours[1])
//     {
//         if (contourArea(pill) > 300)
//         {
//             vector<vector<Point>> pill_contour{pill};
//             drawContours(canvas, pill_contour, -1, Scalar(0, 0, 255), -1);
//             pill_count[1]++;
//         }
//     }

//     imshow("detect pill", canvas);
//     if (pill_count[0] >= 16)
//     {
//         ROS_INFO("Detect result: elliptical pills (===)");
//         rotate_cmd = LEFT_ROTATION; // useless
//         // empirical value
//         cmd.linear.x = 0.3;
//         cmd.angular.z = 0.4;
//     }
//     else if (pill_count[1] >= 16)
//     {
//         ROS_INFO("Detect result: round pills (=)");
//         rotate_cmd = RIGHT_ROTATION;
//         cmd.linear.x = 0.3;
//         cmd.angular.z = -0.4;
//     }
//     else
//     {
//         rotate_cmd = NULL_ROTATION;
//         return;
//     }

//     // <<<<<<< open-loop control >>>>>>>>>
//     // set the frequency of publishing geometry message
//     ros::Rate fix_frequency(10);
//     // last for four seconds
//     for (int i = 0; i < 40; i++)
//     {
//         vel_pub.publish(cmd);
//         // let the frequency of loop coorespond to 10
//         fix_frequency.sleep();
//     }

//     cmd.angular.z = 0;
//     // go straight for two seconds
//     // make the robot looks like find the road at end section and drive down the road
//     for (int i = 0; i < 20; i++)
//     {
//         vel_pub.publish(cmd);
//         fix_frequency.sleep();
//     }

//     return;
// }

// // to make the ros node be able to quit by pressing ctrl + c
// void MySigintHandler(int sig)
// {
//     ROS_INFO("shut down ros node");
//     ros::shutdown();
//     exit(0);
// }

int main(int argc, char **argv)
{
    // the origin size of image from zed camera is 4416 pixels for width, 1242 pixels for height
    // ros::init(argc, argv, "task_first_node1");
    // ros::NodeHandle n;
    // signal(SIGINT, MySigintHandler);

    // Planner p(n);

    return 0;
}