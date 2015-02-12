#include <iostream>
#include <getopt.h>
#include <linux/input.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace sensor_msgs;

Mat disp, left, right;

Mat colorize_disp(Mat disp_mono)
{
    Mat disp_color(disp_mono.rows, disp_mono.cols, CV_8UC3);
    Mat hsv(disp_mono.rows, disp_mono.cols, CV_32FC3);

    disp_color = Scalar::all(0);
    hsv = Scalar::all(0);

    for (int y=0; y<disp_mono.rows; y++)
        for (int x=0; x<disp_mono.cols; x++)
        {
            uchar d = disp_mono.at<uchar>(y,x);
           if (d==0)
                continue;

           hsv.at<Vec3f>(y,x) = Vec3f(255-d, 1.0,1.0);
        }

    cvtColor(hsv, disp_color, CV_HSV2BGR);

    return disp_color;
}

void disp_cb(sensor_msgs::Image disp_img)
{
    cv_bridge::CvImagePtr disp_ptr;
    try
    {
        disp_ptr = cv_bridge::toCvCopy(disp_img, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    disp = disp_ptr->image.clone();

    Mat disp_color = colorize_disp(disp);

    imshow("disp", disp);
    imshow("disp colored", disp_color);

    waitKey(5);
}
void disp_filtered_cb(sensor_msgs::Image disp_filtered_img)
{
    cv_bridge::CvImagePtr disp_ptr;
    try
    {
        disp_ptr = cv_bridge::toCvCopy(disp_filtered_img, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    disp = disp_ptr->image.clone();

    Mat disp_color = colorize_disp(disp);

    imshow("disp_filtered", disp);
    imshow("disp_filtered colored", disp_color);

    waitKey(5);
}

void left_cb(sensor_msgs::Image left_img)
{
    cv_bridge::CvImagePtr left_ptr;
    try
    {
        left_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    left = left_ptr->image.clone();

    imshow("left", left);

    waitKey(5);
}

int main(int argc, char *argv[])
{
    // Initialize ROS
    ros::init (argc, argv, "orview");
    ros::NodeHandle nh;

    ros::Subscriber sub_disp = nh.subscribe("/orsens/disparity", 1, disp_cb);
    ros::Subscriber sub_disp_filtered = nh.subscribe("/orsens/disparity_filtered", 1, disp_filtered_cb);
    ros::Subscriber sub_left = nh.subscribe("/orsens/left", 1, left_cb);

    namedWindow("disp", 1);
    namedWindow("left", 1);

    // Spin
    ros::spin ();

    return 0;
}

