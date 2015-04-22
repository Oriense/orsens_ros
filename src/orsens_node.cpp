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

#include <pcl_conversions/pcl_conversions.h>

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
#include <../../devel/include/orsens/Obstacles.h>
#include <../../devel/include/orsens/Way.h>

using namespace cv;
using namespace sensor_msgs;
using namespace camera_info_manager;

///TODO sharedptr
CameraInfoManager *cinfo_left;
CameraInfoManager *cinfo_right;

static Orsens orsens_device;

static bool working = false;

void sigint_handler(int sig)
{
    working = false;
    delete cinfo_left;
    delete cinfo_right;
    orsens_device.stop();
    printf("stopped\n");
    ros::shutdown();
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "orsens_node");
    ros::NodeHandle nh;
    string node_name = ros::this_node::getName();

    string frame_id;
    string camera_namespace;
    string left_camera_info_url, right_camera_info_url;
    string capture_mode_string;
    string data_path;
    int color_width, depth_width;
    int color_rate, depth_rate;
    bool compress_color, compress_depth;
    bool publish_color, publish_disp, publish_depth, publish_cloud, publish_obstacles, publish_segmentation_mask, publish_left_cam_info, publish_right_cam_info;

    nh.param<string>(node_name+"/capture_mode", capture_mode_string, "depth_only");
    nh.param<string>(node_name+"/data_path", data_path, "data"); ///TODO find orsens here?
    nh.param<string>(node_name+"/left_camera_info_url", left_camera_info_url, "file://"+data_path+"/calibration/caminfo_left.yml");
    nh.param<string>(node_name+"/right_camera_info_url", right_camera_info_url, "file://"+data_path+"/calibration/caminfo_right.yml");
    nh.param<string>(node_name+"/frame_id", frame_id, "orsens_camera");
    nh.param<string>(node_name+"/camera_namespace", camera_namespace, "/orsens/");
    nh.param<int>(node_name+"/color_width", color_width, 640);
    nh.param<int>(node_name+"/depth_width", depth_width, 640);
    nh.param<int>(node_name+"/color_rate", color_rate, 15);
    nh.param<int>(node_name+"/depth_rate", depth_rate, 15);
    nh.param<bool>(node_name+"/compress_color", compress_color, false);
    nh.param<bool>(node_name+"/compress_depth", compress_depth, false);
    nh.param<bool>(node_name+"/publish_depth", publish_depth, true);
    nh.param<bool>(node_name+"/publish_cloud", publish_cloud, false);
    nh.param<bool>(node_name+"/publish_left_camera_info", publish_left_cam_info, false);
    nh.param<bool>(node_name+"/publish_right_camera_info", publish_right_cam_info, false);
    nh.param<bool>(node_name+"/publish_obstacles", publish_obstacles, false);
    nh.param<bool>(node_name+"/publish_segmentation_mask", publish_segmentation_mask, false);
    bool pub_obstacle = true;

    Orsens::CaptureMode capture_mode = Orsens::captureModeFromString(capture_mode_string);

    if (!orsens_device.start(capture_mode, data_path, color_width, depth_width, color_rate, depth_rate, compress_color, compress_depth))
    {
        ROS_ERROR("unable to start OrSens device, check connection\n");
        return -1;
    }

    // Create a ROS publishers for the output messages
    ros::Publisher pub_left = nh.advertise<sensor_msgs::Image> (camera_namespace+"left/image_raw", 1);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::Image> (camera_namespace+"right/image_raw", 1);
    ros::Publisher pub_disp = nh.advertise<sensor_msgs::Image> (camera_namespace+"disp", 1); // 0-255
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image> (camera_namespace+"depth", 1); // uint16 in mm
    ros::Publisher pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(camera_namespace+"left/camera_info", 1);
    ros::Publisher pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(camera_namespace+"right/camera_info", 1);
    ros::Publisher pub_cloud = nh.advertise<pcl::PCLPointCloud2>(camera_namespace+"cloud", 1);
    ros::Publisher pub_obs = nh.advertise<orsens::Obstacles>(camera_namespace+"obstacles", 1);
    ros::Publisher pub_segmentation_mask = nh.advertise<sensor_msgs::Image>(camera_namespace+"segmentation_mask", 1);

    ros::Rate loop_rate(15);

    sensor_msgs::Image ros_left,  ros_right, ros_disp, ros_depth, ros_mask;
    sensor_msgs::CameraInfo l_info_msg, r_info_msg;
    orsens::Obstacles obs;

    bool caminfo_loaded = false;

    if (publish_left_cam_info || publish_right_cam_info)
    {
        cinfo_left = new CameraInfoManager(nh, "/orsens/left", left_camera_info_url);
        cinfo_right = new CameraInfoManager(nh, "/orsens/right", right_camera_info_url);

        caminfo_loaded = cinfo_left->isCalibrated() && cinfo_right->isCalibrated();
        if (!caminfo_loaded)
            ROS_WARN("Failed to load calibration data, check path");
        else
        {
            ROS_INFO("Calibration data loaded\n");

            l_info_msg.header.frame_id = frame_id;

            l_info_msg.height = color_width*3/4;
            l_info_msg.width = color_width;

            r_info_msg.header.frame_id = frame_id;

            r_info_msg.height = color_width*3/4;
            r_info_msg.width = color_width;
        }
    }

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

        if (caminfo_loaded)
        {
            if (publish_left_cam_info)
            {
                l_info_msg = cinfo_left->getCameraInfo();
                l_info_msg.header.frame_id = frame_id;
                l_info_msg.header.stamp = time;
                pub_left_info.publish(l_info_msg);

            }

            if (publish_right_cam_info)
            {
                r_info_msg = cinfo_right->getCameraInfo();
                r_info_msg.header.frame_id = frame_id;
                r_info_msg.header.stamp = time;
                pub_right_info.publish(r_info_msg);

            }
        }

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
            ros_disp.header.frame_id = frame_id;

            pub_disp.publish(ros_disp);

            if (publish_depth)
            {
                Mat depth = orsens_device.getDepth();
                if (!depth.empty())
                {
                    fillImage(ros_depth, "16UC1", depth.rows, depth.cols, 2*depth.cols, depth.data);

                    ros_depth.header.stamp = time;
                    ros_depth.header.frame_id = frame_id;

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

            if(publish_obstacles)
            {
                if(pub_obstacle)
                {
                    Obstacle obstacle = orsens_device.getNearestObstacle();
                    obs.u = obstacle.centre.x;
                    obs.v = obstacle.centre.y;
                    obs.centre_pt.x = obstacle.centre_world.x;
                    obs.centre_pt.y = obstacle.centre_world.y;
                    obs.centre_pt.z = obstacle.centre_world.z;
                    obs.dist = obstacle.dist;
                    obs.angle = obstacle.angle;
                    obs.min_pt.x = obstacle.min_pt_world.x;
                    obs.min_pt.y = obstacle.min_pt_world.y;
                    obs.min_pt.z = obstacle.min_pt_world.z;
                    obs.max_pt.x = obstacle.max_pt_world.x;
                    obs.max_pt.y = obstacle.max_pt_world.y;
                    obs.max_pt.z = obstacle.max_pt_world.z;
                }
                else
                {
                    ScenePoint scene_point = orsens_device.getNearestPoint();

                    obs.u = scene_point.pt_image.x;
                    obs.v = scene_point.pt_image.y;
                    obs.centre_pt.x = scene_point.pt_world.x;
                    obs.centre_pt.y = scene_point.pt_world.y;
                    obs.centre_pt.z = scene_point.pt_world.z;
                }

                obs.header.stamp = time;
                obs.header.frame_id = "orsens_camera";
                pub_obs.publish(obs);
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
