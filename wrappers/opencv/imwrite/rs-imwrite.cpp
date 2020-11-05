// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <fstream>

using namespace rs2;

bool save_frame_raw_data(const std::string& filename, rs2::frame frame)
{
    bool ret = false;
    auto image = frame.as<video_frame>();
    if (image)
    {
        std::ofstream outfile(filename.data(), std::ofstream::binary);
        outfile.write(static_cast<const char*>(image.get_data()), image.get_height()*image.get_stride_in_bytes());

        outfile.close();
        ret = true;
    }

    return ret;
}

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);

        rs2::frame raw_depth = data.get_depth_frame();
        // Save 16bit depth raw data to a binary file
        save_frame_raw_data("depth.bin", raw_depth); //into binary file

        // Save 16bit depth raw data to a png file
        cv::Mat cv_depth = cv::Mat(cv::Size(w, h), CV_16SC1, (void*)raw_depth.get_data());
        std::vector<int> cmprs_params;
        cmprs_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        cmprs_params.push_back(0);
        cv::imwrite("depth.png", cv_depth, cmprs_params);
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



