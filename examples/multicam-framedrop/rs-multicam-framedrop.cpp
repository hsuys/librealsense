#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>


int main(int argc, char * argv[]) try
{
    std::string d455_serial = argv[1];
    std::string d435_serial = argv[2];

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices()) {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "Detected device with serial " << serial << std::endl;
        serials.push_back(serial);
    }

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        if (serial == d455_serial) // D455 sensor - depth stream
        {
            cfg.enable_stream(
                RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        }
        if (serial == d435_serial) // D435 sensor - depth stream
        {
            cfg.enable_stream(
                RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        }
        if (serial == d455_serial || serial == d435_serial)
        {
            cfg.enable_device(serial);
            pipe.start(cfg);
            pipelines.emplace_back(pipe);
        }
    }

    std::mutex my_lock;

    rs2::frame_queue d455_frames_queue(5);
    rs2::frame_queue d435_frames_queue(5);

    long long prev_d455_frame_timestamp = 0;
    long long prev_d435_frame_timestamp = 0;

    std::thread proc_d455([&]() {
        while (true)
        {
            rs2::frame f;
            if (d455_frames_queue.poll_for_frame(&f))
            {
                my_lock.lock();

                auto d455_frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                if (prev_d455_frame_timestamp == 0) prev_d455_frame_timestamp = d455_frame_timestamp;
                auto d455_frame_timestamp_diff = d455_frame_timestamp - prev_d455_frame_timestamp;
                prev_d455_frame_timestamp = d455_frame_timestamp;
                
              
                if (d455_frame_timestamp_diff > 67000)
                {
                    std::cout << "\033[33m" << "D455 frame dropped: " << "[" <<  d455_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }

                my_lock.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
        }
    });
    proc_d455.detach();

    std::thread proc_d435([&]() {
        while (true)
        {
            rs2::frame f;
            if (d435_frames_queue.poll_for_frame(&f))
            {
                my_lock.lock();

                auto d435_frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                if (prev_d435_frame_timestamp == 0) prev_d435_frame_timestamp = d435_frame_timestamp;
                auto d435_frame_timestamp_diff = d435_frame_timestamp - prev_d435_frame_timestamp;
                prev_d435_frame_timestamp = d435_frame_timestamp;
                
                if (d435_frame_timestamp_diff > 67000)
                {
                    std::cout << "\033[33m" << "D435 frame dropped: " << "[" <<  d435_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }

                my_lock.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
        }
    });
    proc_d435.detach();

    // Main app loop
    while (true)
    {
        for (auto &&pipe : pipelines)
        {
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs))
            {
                for (const rs2::frame& f : fs) {
                    std::string serial = rs2::sensor_from_frame(f)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    if (serial == d455_serial) {
                        d455_frames_queue.enqueue(f);
                    }
                    if (serial == d435_serial) {
                        d435_frames_queue.enqueue(f);
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
