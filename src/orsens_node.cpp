#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>


#include <pcl_conversions/pcl_conversions.h> //!

#include <signal.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../include/orsens.h"
#include <../../devel/include/orsens/NearestObstacle.h>

using namespace cv;
using namespace sensor_msgs;

ros::Publisher pub_left;
ros::Publisher pub_right;
ros::Publisher pub_disp;
ros::Publisher pub_depth;
ros::Publisher pub_disp_filtered;
ros::Publisher pub_info;
ros::Publisher pub_cloud;
ros::Publisher pub_nearest_point;
ros::Publisher pub_segmentation_mask;

Orsens orsens_device;

bool working = false;

void sigint_handler(int sig)
{
    working = false;
    orsens_device.stop();
    printf("stopped\n");
    ros::shutdown();
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "orsens_node");
    ros::NodeHandle nh;

    string capture_mode_string;
    string data_path;
    int color_width, depth_width;
    int color_rate, depth_rate;
    bool compress_color, compress_depth;
    bool publish_color, publish_disp, publish_depth, publish_cloud, publish_nearest_point, publish_segmentation_mask;

    std::string node_name = ros::this_node::getName();

    nh.param<string>(node_name+"/capture_mode", capture_mode_string, "depth_only");
    nh.param<string>(node_name+"/data_path", data_path, "../data");
    nh.param<int>(node_name+"/color_width", color_width, 640);
    nh.param<int>(node_name+"/depth_width", depth_width, 640);
    nh.param<int>(node_name+"/color_rate", color_rate, 15);
    nh.param<int>(node_name+"/depth_rate", depth_rate, 15);
    nh.param<bool>(node_name+"/compress_color", compress_color, false);
    nh.param<bool>(node_name+"/compress_depth", compress_depth, false);
    nh.param<bool>(node_name+"/publish_depth", publish_depth, true);
    nh.param<bool>(node_name+"/publish_cloud", publish_cloud, false);
    nh.param<bool>(node_name+"/publish_nearest_point", publish_nearest_point, false);
    nh.param<bool>(node_name+"/publish_segmentation_mask", publish_segmentation_mask, false);

    Orsens::CaptureMode capture_mode = Orsens::captureModeFromString(capture_mode_string);

    if (!orsens_device.start(capture_mode, data_path, color_width, depth_width, color_rate, depth_rate, compress_color, compress_depth))
    {
        ROS_ERROR("unable to start OrSens device, check connection\n");
        return -1;
    }

    // Create a ROS publishers for the output messages
    pub_left = nh.advertise<sensor_msgs::Image> ("/orsens/left", 1);
    pub_right = nh.advertise<sensor_msgs::Image> ("/orsens/right", 1);
    pub_disp = nh.advertise<sensor_msgs::Image> ("/orsens/disparity", 1); // 0-255
    pub_depth = nh.advertise<sensor_msgs::Image> ("/orsens/depth", 1); // uint16 in mm
//    pub_info = nh.advertise<sensor_msgs::CameraInfo>("/orsens/camera_info", 1);
    pub_cloud = nh.advertise<pcl::PCLPointCloud2>("/orsens/cloud", 1);
    pub_nearest_point = nh.advertise<orsens::NearestObstacle>("/orsens/nearest_point", 1);
    pub_segmentation_mask = nh.advertise<sensor_msgs::Image>("/orsens/segmentation_mask", 1);

    ros::Rate loop_rate(15);
    string frame_id = "orsens_camera";

    sensor_msgs::Image ros_left,  ros_right, ros_disp, ros_depth, ros_mask;
    orsens::NearestObstacle obs;

    working = true;

    while (nh.ok() && working)
    {
        orsens_device.grabSensorData();

        Mat left, right, disp, depth, cloud;

        switch (capture_mode)
        {
        case Orsens::CAPTURE_DEPTH_ONLY:
            disp = orsens_device.getDisp();
            break;

        case Orsens::CAPTURE_LEFT_ONLY:
            left = orsens_device.getLeft();
            break;

        case Orsens::CAPTURE_DEPTH_LEFT:
            disp = orsens_device.getDisp();
            left = orsens_device.getLeft();
            break;

        case Orsens::CAPTURE_LEFT_RIGHT:
            left = orsens_device.getLeft();
            right = orsens_device.getRight();
            break;
        }

        ros::Time time = ros::Time::now();

        if (!left.empty())
        {
            fillImage(ros_left, "bgr8", left.rows, left.cols, left.step.buf[0], left.data);

            ros_left.header.stamp = time;
            ros_left.header.frame_id = frame_id;

            pub_left.publish(ros_left);
        }

        if (!right.empty())
        {
            fillImage(ros_right, "bgr8", right.rows, right.cols, right.step.buf[0], right.data);

            ros_right.header.stamp = time;
            ros_right.header.frame_id = frame_id;

            pub_right.publish(ros_right);
        }

        if  (!disp.empty())
        {
            fillImage(ros_disp, "mono8", disp.rows, disp.cols, disp.cols, disp.data);

            ros_disp.header.stamp = time;

            pub_disp.publish(ros_disp);

            if (publish_depth)
            {
                Mat depth = orsens_device.getDepth();
                if (!depth.empty())
                {
                    fillImage(ros_depth, "mono16", depth.rows, depth.cols, 2*depth.cols, depth.data);

                    ros_depth.header.stamp = time;

                    pub_depth.publish(ros_depth);
                }
            }

            if (publish_cloud)
            {
                Mat cloud = orsens_device.getPointCloud();
                if (!cloud.empty())
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB>);

                    // reset the point cloud
                    cloudOut->clear();
                    cloudOut->width = cloud.cols;
                    cloudOut->height = cloud.rows;
                    cloudOut->points.resize(cloudOut->width * cloudOut->height);

                    float px, py, pz;
                    uchar pb=0, pr=0, pg=0;
                    int index = 0;

                    for (int y=0; y<cloud.rows; y++)
                        for (int x=0; x<cloud.cols; x++)
                        {

                            px = cloud.at<cv::Vec3f>(y,x)[0];
                            py = cloud.at<cv::Vec3f>(y,x)[1];
                            pz = cloud.at<cv::Vec3f>(y,x)[2];

                            if (!left.empty())
                            {
                                pb = left.at<cv::Vec3b>(y,x)[0];
                                pg = left.at<cv::Vec3b>(y,x)[1];
                                pr = left.at<cv::Vec3b>(y,x)[2];
                            }

                            cloudOut->points[index].x = px;
                            cloudOut->points[index].y = py;
                            cloudOut->points[index].z = pz;
                            cloudOut->points[index].r = pr;
                            cloudOut->points[index].g = pg;
                            cloudOut->points[index].b = pb;

                            index ++;
                        }

                    pcl::PCLPointCloud2::Ptr cloudOutROS(new pcl::PCLPointCloud2); //output ROS message
                    pcl::toPCLPointCloud2 (*cloudOut, *cloudOutROS);
                    cloudOutROS->header.frame_id = frame_id;
                    pub_cloud.publish(cloudOutROS);
                }
            }

            if(publish_nearest_point)
            {
                ScenePoint scene_point = orsens_device.getNearestPoint();

                obs.header.stamp = time;
                obs.header.frame_id = frame_id;
                obs.u = scene_point.pt_image.x;
                obs.v = scene_point.pt_image.y;
                obs.pt.x = scene_point.pt_world.x;
                obs.pt.y = scene_point.pt_world.y;
                obs.pt.z = scene_point.pt_world.z;

                pub_nearest_point.publish(obs);
            }

            if(publish_segmentation_mask)
            {
                Mat mask = orsens_device.getSegmentationMask();

                if (!mask.empty())
                {
                    fillImage(ros_mask, "mono8", mask.rows, mask.cols, mask.cols, mask.data);

                    ros_mask.header.stamp = time;
                    ros_mask.header.frame_id = frame_id;

                    pub_segmentation_mask.publish(ros_mask);
                }

            }

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    sigint_handler(0);

}
