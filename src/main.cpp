// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

void save_frame_depth_data(int category,
                           int idx,
                           rs2::frame depth,
                           Mat color,
                           float depth_units)
{
    if (auto image = depth.as<rs2::video_frame>())
    {
        std::ofstream myfile;
        std::stringstream depth_name;
        depth_name << "./data/" << category << "_" << idx  << "_depth_"<< ".txt";
        myfile.open(depth_name.str());
        myfile << std::setprecision(6);
        auto pixels = (uint16_t*)image.get_data();

        for (auto y = 0; y < image.get_height(); y++)
        {
            for (auto x = 0; x < image.get_width(); x++)
            {
                myfile << pixels[image.get_width() * y + x] * depth_units << ", ";
            }
            myfile << "\n";
        }

        myfile.close();

        std::stringstream color_name;
        color_name << "./data/" << category << "_" << idx  << "_color_"<< ".png";
        imwrite(color_name.str(), color);
    }
}

int main(int argc, char * argv[]) 
try
{
    // Initialnization
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/RGB_image", 1);
    image_transport::Publisher pub2 = it.advertise("/Depth_image", 1);
    sensor_msgs::ImagePtr msgRGB,msgD;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::align align_to_color(RS2_STREAM_COLOR);

    auto depth_units = rs2::context().query_devices().front()
            .query_sensors().front().get_option(RS2_OPTION_DEPTH_UNITS);

    std::cout << "depth_units" << depth_units <<std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    const auto color_name = "Display Color";
    const auto depth_name = "Display Depth";

    namedWindow(color_name, WINDOW_AUTOSIZE);
    namedWindow(depth_name, WINDOW_AUTOSIZE);

    char keyinput;
    int catagory=0;
    int idx=0;

    std::cout<<"Save current frame: S"<<std::endl;
    std::cout<<"Change a new item: N"<<std::endl;
    std::cout<<"Next frame: any other keys"<<std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;

   // std::cout<<"What is the initial catagory?"<<std::endl;
  //  std::cin>>catagory;
  //  std::cout<<"Initial Catagory: "<< catagory <<std::endl;

    int iteration=0;


    ros::Rate loop_rate(20);
    while (ros::ok)
    {

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        data = align_to_color.process(data);
     	rs2::depth_frame depth= data.get_depth_frame();
        rs2::frame color = data.get_color_frame();
        rs2::frame depth_show = depth.apply_filter(color_map);
        
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
       // float dist_to_center = depth.get_distance(w/2,h/2);

	//cout << "distance from center to carema" << dist_to_center;
        // Create OpenCV matrix of size (w,h) from the colorized depth data
      //  Mat image_depth_show(Size(w, h), CV_8UC3, (void*)depth_show.get_data(), Mat::AUTO_STEP);
        Mat image_depth(Size(w, h), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP); //16U comes from the official sdk
        Mat image_color(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image_color, image_color, COLOR_BGR2RGB);

        msgRGB = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_color).toImageMsg();
        pub.publish(msgRGB);

        msgD = cv_bridge::CvImage(std_msgs::Header(), "mono16", image_depth).toImageMsg();
        pub2.publish(msgD);

        // Update the window with new data
        imshow(color_name, image_color);
        imshow(depth_name, image_depth);
        waitKey(10);  // essential, otherwise the show window will not appear
        ros::spinOnce();

        loop_rate.sleep();

    }


    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



