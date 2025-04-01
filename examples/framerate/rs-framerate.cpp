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

std::mutex my_lock;

struct _stream_profile
{
    rs2::frame_queue frame_queue;
    std::string serial_number;
    std::string stream_name;
    int fps;
    int unique_id;
    int total_frame_count;
    //rs2::pipeline pipe;
};

void proc_FrameCheck(_stream_profile& stream_profile)
{
    int prev_frame_count = 0;
    long long prev_frame_timestamp = 0;
    int tolerance = 1.2*1000000/stream_profile.fps; // current tolerance is 1.2 times of the 1/fps
    while (!stopped) {
        rs2::frame f;
        if (stream_profile.frame_queue.poll_for_frame(&f)) {
            stream_profile.total_frame_count++;
            //std::cout << "--- " << stream_profile.total_frame_count << " " << stream_profile.stream_name << " " << stream_profile.serial_number << " " << std::endl;

            my_lock.lock();

            auto frame_count = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);

            auto frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
            if (prev_frame_timestamp == 0) prev_frame_timestamp = frame_timestamp;
            auto frame_timestamp_diff = frame_timestamp - prev_frame_timestamp;
            prev_frame_timestamp = frame_timestamp;

            if (frame_count - prev_frame_count == 2) {
                std::cout << "\033[33m" << "frame dropped 1: " << frame_count << " ! (" << frame_count - prev_frame_count << ")" << "[" << frame_timestamp_diff << "]" << "\033[0m" << std::endl;
            }
            else if (frame_count - prev_frame_count > 2) {
                std::cout << "\033[31m" << "frame dropped > 1: " << frame_count << " ! (" << frame_count - prev_frame_count << ")" << "[" << frame_timestamp_diff << "]" << "\033[0m" << std::endl;
            }
            
            prev_frame_count = frame_count;
            
            if (frame_timestamp_diff > tolerance) {
                std::cout << "\033[33m" << stream_profile.serial_number << " " << stream_profile.stream_name << " dropped: " << "[" << frame_timestamp_diff << "]" << "\033[0m" << std::endl;
            }

            my_lock.unlock();

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEEP_MS)); 
    }
    std::cout << stream_profile.serial_number << " " << stream_profile.stream_name << " proc " << " stopped" << std::endl;
}

void proc_FrameGrab(std::vector<_stream_profile>& stream_profiles, std::vector<rs2::pipeline>& pipelines)
{
    while (!stopped) {
        for (auto&& pipe : pipelines) {
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs)) {
                for (const rs2::frame& f : fs) {
                    std::string serial = rs2::sensor_from_frame(f)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    int unique_id = f.get_profile().unique_id();

                    for (auto&& stream_profile : stream_profiles) {
                        if (stream_profile.unique_id == unique_id && stream_profile.serial_number == serial) {
                            //std::cout << "+++ " << stream_profile.total_frame_count << " " << stream_profile.stream_name << " " << stream_profile.serial_number << " " << std::endl;
                            stream_profile.frame_queue.enqueue(f);
                        }
                    }
                }
            }
        }
    }
    std::cout << "proc_grab stopped" << std::endl;
}

int main(int argc, char* argv[]) try
{
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string> test_serials;
    std::vector<std::string> connect_serials;
    std::vector<std::thread> threads;

    if (argc < 2) {
        std::cout << "Please input at least one camera serial numbers to start" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        for (int i=1; i<argc; i++)
            test_serials.push_back(argv[i]);

        for (int i = 0; i < test_serials.size(); i++)
            std::cout << "Testing: " << test_serials.at(i) << std::endl;
    }

    // Capture serial numbers before opening streaming
    for (auto&& dev : ctx.query_devices()) {
        std::string desc = dev.get_description();
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        std::cout << "Found: " << desc << " f/w " << fw << std::endl;
        connect_serials.push_back(serial);
    }

    if (connect_serials.size() < 1) {
        std::cout << " Please connect at least one RealSense Cameras to the host" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        for (const std::string sn : test_serials)
            if (std::find(connect_serials.begin(), connect_serials.end(), sn) == connect_serials.end()) {
                std::cout << sn << " cannot be paired" << std::endl;
                return EXIT_FAILURE;
            }
            else
                std::cout << sn << " paired successfully" << std::endl;
    }

    // Start a streaming pipe per each connected device
    for (auto&& serial : test_serials) {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;

        cfg.enable_stream(
            RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
        //cfg.enable_stream(
        //    RS2_STREAM_INFRARED, WIDTH, HEIGHT, RS2_FORMAT_Y8, FPS);
        cfg.enable_stream(
            RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, FPS);
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }
    stopped = false;
    
    int total_streams = 0;
    int profile_id = 0;
    for (auto&& pipe : pipelines) {
        auto streams = pipe.get_active_profile().get_streams();
        for (auto&& stream : streams) {
            total_streams++;
        }
    }

    std::vector<_stream_profile> stream_profs(total_streams);
    
    for (auto&& pipe : pipelines) {
        std::string sn = pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        auto streams = pipe.get_active_profile().get_streams();
        for (auto&& stream : streams) {
            rs2::frame_queue frame_q(5);
            std::string str_name = stream.stream_name();
            int unique_id = stream.unique_id();
            int frame_rate = stream.fps();
            stream_profs[profile_id].serial_number = sn;
            stream_profs[profile_id].stream_name = str_name;
            stream_profs[profile_id].total_frame_count = 0;
            stream_profs[profile_id].frame_queue = frame_q;
            stream_profs[profile_id].unique_id = unique_id;
            stream_profs[profile_id].fps = frame_rate;
            threads.push_back(std::thread(proc_FrameCheck, std::ref(stream_profs[profile_id])));
            profile_id++;

        }
    }

    threads.push_back(std::thread(proc_FrameGrab, std::ref(stream_profs), std::ref(pipelines)));

    signal(SIGINT, crtlc);

    // Main app loop
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0;

    std::cout << "Duration(second), ";
    for (auto&& stream_profile : stream_profs) {
        std::cout << stream_profile.serial_number << "-" << stream_profile.stream_name <<", ";
    }
    std::cout << std::endl;

    while (!stopped) {
        auto t2 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
        auto t4 = std::chrono::duration_cast<std::chrono::seconds>(t2 - t0).count();
        if (t3 >= 5) {
            t1 = t2;
            std::cout << t4;
            for (auto&& stream_profile : stream_profs) {
                //std::cout << "*** " << stream_profile.total_frame_count << " " << stream_profile.stream_name << " " << stream_profile.serial_number << " " << std::endl;
                std::cout << ", " << stream_profile.total_frame_count ;
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep for 1 ms
    }
    std::cout << "proc_main stopped" << std::endl;
    stopped = true;

    return EXIT_SUCCESS;
}
catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

