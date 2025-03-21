#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <common/cli.h>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include <atomic>
#include <signal.h>
#include  <algorithm>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define WIDTH           1280              // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          720               // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SLEEEP_MS       1 

std::atomic_bool stopped(false);

void crtlc(int s)
{
    stopped = true;
}

int main(int argc, char * argv[]) try
{
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::vector<rs2::pipeline>            pipelines;
    std::string d455_serial = "";
    std::string d435_serial = "";

    if (argc < 3) {

        std::cout << "Please input two camera serial numbers to start" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        d455_serial = argv[1];
        d435_serial = argv[2];
    }

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices()) {
        std::string desc = dev.get_description();
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        std::cout << "Found: " << desc << " f/w " << fw << std::endl;
        serials.push_back(serial);
    }

    if (serials.size() < 2) {
        std::cout << " Please connect 2 RealSense Cameras to the host" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        for (const std::string sn : {d455_serial, d435_serial})
            if (std::find(serials.begin(), serials.end(), sn) == serials.end()) {
                std::cout << sn << " cannot be paired" << std::endl;
                return EXIT_FAILURE;
            }
            else
                std::cout << sn << " paired successfully" << std::endl;
    }

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials) {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        if (serial == d455_serial) { // D455 sensor - depth stream
                cfg.enable_stream(
                RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
                //cfg.enable_stream(
                //    RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, FPS);
        }
        if (serial == d435_serial) { // D435 sensor - depth stream
                cfg.enable_stream(
                RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
                //cfg.enable_stream(
                //    RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, FPS);
        }
        if (serial == d455_serial || serial == d435_serial){
            cfg.enable_device(serial);
            pipe.start(cfg);
            pipelines.emplace_back(pipe);
        }
    }

    signal(SIGINT, crtlc);
    std::mutex my_lock;

    rs2::frame_queue d455_frames_queue(5);
    rs2::frame_queue d435_frames_queue(5);
    
    auto d455_frame_counts = 0;
    auto d435_frame_counts = 0;

    int prev_d455_frame_count = 0;
    int prev_d435_frame_count = 0;
        
    std::thread proc_d455([&]() {
        long long prev_d455_frame_timestamp = 0;
        while (!stopped) {
            rs2::frame f;
            if (d455_frames_queue.poll_for_frame(&f)) {
                d455_frame_counts++;
                std::cout<< "\033[35m" << "*" << "\033[0m";
                my_lock.lock();

                auto d455_frame_count = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                auto d455_frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                if (prev_d455_frame_timestamp == 0) prev_d455_frame_timestamp = d455_frame_timestamp;
                auto d455_frame_timestamp_diff = d455_frame_timestamp - prev_d455_frame_timestamp;
                prev_d455_frame_timestamp = d455_frame_timestamp;
                
                if (d455_frame_count - prev_d455_frame_count == 2)
                {
                    std::cout << "\033[33m" << "D455 frame dropped 1: " << d455_frame_count << " ! (" << d455_frame_count - prev_d455_frame_count << ")" << "[" << d455_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }
                else if (d455_frame_count - prev_d455_frame_count > 2)
                {
                    std::cout << "\033[31m" << "D455 frame dropped > 1: " << d455_frame_count << " ! (" << d455_frame_count - prev_d455_frame_count << ")" << "[" << d455_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }
                prev_d455_frame_count = d455_frame_count;

                if (d455_frame_timestamp_diff > 34000) {
                    std::cout << "\033[33m" << "D455 frame dropped: " << "[" <<  d455_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }

                my_lock.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEEP_MS)); // sleep for 1 ms
        }
        std::cout << "proc_d455 stopped" << std::endl;
    });
    proc_d455.detach();

    std::thread proc_d435([&]() {
        long long prev_d435_frame_timestamp = 0;
        while (!stopped) {
            rs2::frame f;
            if (d435_frames_queue.poll_for_frame(&f)) {
                d435_frame_counts++;
                std::cout<< "\033[36m" << "-" << "\033[0m";
                my_lock.lock();

                auto d435_frame_count = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                auto d435_frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                if (prev_d435_frame_timestamp == 0) prev_d435_frame_timestamp = d435_frame_timestamp;
                auto d435_frame_timestamp_diff = d435_frame_timestamp - prev_d435_frame_timestamp;
                prev_d435_frame_timestamp = d435_frame_timestamp;

                if (d435_frame_count - prev_d435_frame_count == 2)
                {
                    std::cout << "\033[33m" << "D435 frame dropped 1: " << d435_frame_count << " ! (" << d435_frame_count - prev_d435_frame_count << ")" << "[" << d435_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }
                else if (d435_frame_count - prev_d435_frame_count > 2)
                {
                    std::cout << "\033[31m" << "D435 frame dropped > 1: " << d435_frame_count << " ! (" << d435_frame_count - prev_d435_frame_count << ")" << "[" << d435_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }
                prev_d435_frame_count = d435_frame_count;

                if (d435_frame_timestamp_diff > 34000) {
                    std::cout << "\033[33m" << "D435 frame dropped: " << "[" <<  d435_frame_timestamp_diff << "]" << "\033[0m" << std::endl;
                }

                my_lock.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEEP_MS)); // sleep for 1 ms
        }
        std::cout << "proc_d435 stopped" << std::endl;
    });
    proc_d435.detach();  
    
    std::thread proc_grab([&]() {
        while (!stopped) {
            for (auto &&pipe : pipelines) {
                rs2::frameset fs;
                if (pipe.poll_for_frames(&fs)) {
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
        }        
        std::cout << "proc_grab stopped" << std::endl;
    });
    proc_grab.detach();

    // Main app loop
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0;
    
    std::cout << "\033[32m" <<  "Duration (second), frame counts (d455), frame counts (d435)" << "\033[0m" << std::endl;
    while (!stopped) {
        auto t2 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count();
        auto t4 = std::chrono::duration_cast<std::chrono::seconds>(t2-t0).count();
        if (t3 >= 5) {
            t1 = t2;
            std::cout << "\033[32m" << t4 << "," << d455_frame_counts << "," << d435_frame_counts << "\033[0m" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep for 1 ms
    }
    std::cout << "proc_main stopped" << std::endl;
    stopped = true;

    return EXIT_SUCCESS;
}
catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}