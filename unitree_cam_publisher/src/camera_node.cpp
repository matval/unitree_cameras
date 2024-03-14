#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>

// Unitree SDK
#include <UnitreeCameraSDK.hpp>

// ROS stuff
#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Header.h"

std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main(int argc, char **argv)
{
    // Start ROS stuff
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    //< default 0 -> /dev/video0
    int deviceNode;
    n.getParam("device", deviceNode);
    cv::Size frameSize(1856, 800); ///< default frame size 1856x800
    int fps = 30; ///< default camera fps: 30

    std::string topic_name;
    if(deviceNode == 0)
    {
        topic_name = "right_stereo";
    }
    else if(deviceNode == 1)
    {
        topic_name = "left_stereo";
    }
    
    // Load camera stuf
    UnitreeCamera cam(deviceNode); //"stereo_camera_config.yaml"); ///< init camera by device node number

    if(!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    // cam.startStereoCompute(); ///< start disparity computing
    
    usleep(500000);

    image_transport::ImageTransport it(nh);
    // image_transport::Publisher color_pub = it.advertise("camera/image", 1);
    image_transport::CameraPublisher left_cam_pub = it.advertiseCamera(topic_name + "/left_camera/image_rect_color", 1);
    image_transport::CameraPublisher right_cam_pub = it.advertiseCamera(topic_name + "/right_camera/image_rect_color", 1);
    // image_transport::Publisher depth_pub = it.advertise("camera/depth", 1);

    std_msgs::Header image_header;
    sensor_msgs::ImagePtr left_msg, right_msg; //, depth_msg;
    sensor_msgs::CameraInfoPtr left_cam_info(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info(new sensor_msgs::CameraInfo());

    // Get calibration parameters for the right camera
    std::vector<cv::Mat> right_params;
    if(cam.getCalibParams(right_params, false))
    {
        // intrinsic, distortion, xi, rotation, translation, kfe
        for(std::size_t i=0; i<right_params.size(); i++)
        {
            std::cout << "\ndata:\n" << right_params[i] << std::endl;
        }

        // Populate CameraInfo message
        right_cam_info->distortion_model = "rational_polynomial";
        for(int i=0; i<right_params[0].rows*right_params[0].cols; i++)
        {
            // right_cam_info->K[i] = right_params[0].reshape(1).at<double>(i);
            right_cam_info->K[i] = right_params[5](
                cv::Rect( 0, 0, right_params[0].rows, right_params[0].cols)).reshape(1).at<double>(i);
        }
        right_cam_info->D.resize(4, 0.0);
        // for(int i=0; i<right_params[1].rows*right_params[1].cols; i++)
        // {
        //     right_cam_info->D[i] = right_params[1].reshape(1).at<double>(i);
        // }
        for(int i=0; i<right_params[3].rows*right_params[3].cols; i++)
        {
            right_cam_info->R[i] = right_params[3].reshape(1).at<double>(i);
        }
        for(int i=0; i<right_params[5].rows*right_params[5].cols; i++)
        {
            right_cam_info->P[i] = right_params[5].reshape(1).at<double>(i);
        }
    }

    // Get calibration parameters for the left camera
    std::vector<cv::Mat> left_params;
    if(cam.getCalibParams(left_params, true))
    {
        // intrinsic, distortion, xi, rotation, translation, kfe
        for(std::size_t i=0; i<left_params.size(); i++)
        {
            std::cout << "\ndata:\n" << left_params[i] << std::endl;
        }

        // Populate CameraInfo message
        left_cam_info->distortion_model = "rational_polynomial";
        for(int i=0; i<left_params[0].rows*left_params[0].cols; i++)
        {
            // right_cam_info->K[i] = right_params[0].reshape(1).at<double>(i);
            left_cam_info->K[i] = left_params[5](
                cv::Rect( 0, 0, left_params[0].rows, left_params[0].cols)).reshape(1).at<double>(i);
        }
        left_cam_info->D.resize(4, 0.0);
        // for(int i=0; i<left_params[1].rows*left_params[1].cols; i++)
        // {
        //     right_cam_info->D[i] = left_params[1].reshape(1).at<double>(i);
        // }
        for(int i=0; i<left_params[3].rows*left_params[3].cols; i++)
        {
            left_cam_info->R[i] = left_params[3].reshape(1).at<double>(i);
        }
        for(int i=0; i<left_params[5].rows*left_params[5].cols; i++)
        {
            left_cam_info->P[i] = left_params[5].reshape(1).at<double>(i);
        }
    }
    
    ros::Rate loop_rate(fps);
    while(ros::ok() && cam.isOpened())
    {
        cv::Mat left, right; //, depth; //, dispf;
        std::chrono::microseconds t;

        // Get rectified left and right frames  
        if(!cam.getRectStereoFrame(right, left))
        {
            usleep(50);
            continue;
        }

        // Get stereo camera depth image
        // cam.getDepthFrame(dispf, depth);

        // if(!cam.getDepthFrame(depth, false, t))
        // {
        //     usleep(50);
        //     continue;
        // }

        image_header.stamp = ros::Time::now();

        if (!left.empty())
        {
            std::string frame_id = "left_camera_frame";
            if(deviceNode == 0)
            {
                frame_id = "right_" + frame_id;
            }
            else if (deviceNode == 1)
            {
                frame_id = "left_" + frame_id;
            }
            image_header.frame_id = frame_id;
            // Debug color image type
            // std::string ty =  type2str( left.type() );
            // printf("Color: %s %dx%d \n", ty.c_str(), left.cols, left.rows);

            cv::flip(left, left, -1);
            left_msg = cv_bridge::CvImage(image_header, "bgr8", left).toImageMsg();

            left_cam_info->height = left.rows;
            left_cam_info->width = left.cols;
            left_cam_info->header = image_header;

            // Publish via image_transport
            left_cam_pub.publish(left_msg, left_cam_info);
            // cv::waitKey(1);
        }

        if (!right.empty())
        {
            std::string frame_id = "right_camera_frame";
            if(deviceNode == 0)
            {
                frame_id = "right_" + frame_id;
            }
            else if (deviceNode == 1)
            {
                frame_id = "left_" + frame_id;
            }
            image_header.frame_id = frame_id;
            // Debug color image type
            // std::string ty =  type2str( left.type() );
            // printf("Color: %s %dx%d \n", ty.c_str(), left.cols, left.rows);
            cv::flip(right, right, -1);
            right_msg = cv_bridge::CvImage(image_header, "bgr8", right).toImageMsg();

            right_cam_info->height = right.rows;
            right_cam_info->width = right.cols;
            right_cam_info->header = image_header;

            // Publish via image_transport
            right_cam_pub.publish(right_msg, right_cam_info);
            // cv::waitKey(1);
        }

        // if (!depth.empty())
        // {
        //     // Debug depth image type
        //     // std::string ty =  type2str( depth.type() );
        //     // printf("Depth: %s %dx%d \n", ty.c_str(), depth.cols, depth.rows);

        //     // std::string ty2 =  type2str( dispf.type() );
        //     // printf("Disparity: %s %dx%d \n", ty2.c_str(), dispf.cols, dispf.rows);

        //     depth_msg = cv_bridge::CvImage(image_header, "bgr8", depth).toImageMsg();
        //     depth_pub.publish(depth_msg);
        //     // cv::waitKey(1);
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // cam.stopStereoCompute();
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}
