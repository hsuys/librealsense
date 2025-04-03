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
std::atomic_bool logging(false);
std::mutex my_lock;

void crtlc(int s)
{
    stopped = true;
}

struct _stream
{
    rs2::stream_profile sp;
    int total_frame_count = 0;
    int prev_total_frame_count = 0 ;
    int prev_frame_count = 0 ;
    long long prev_frame_timestamp = 0;
};
struct _sensor
{
    std::string sn;
    float tolerance; // current tolerance is 1.2 times of the 1/fps
    int num_active_stream;
    std::vector<_stream> streams;
};

void proc_stat(rs2::frame& f, std::vector<_sensor>& sensors)
{
    auto sensor_serial = rs2::sensor_from_frame(f)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    auto frame_unique_id = f.get_profile().unique_id();
    //std::string stream_format = rs2_format_to_string(f.get_profile().format());
    std::string stream_name = f.get_profile().stream_name();
    //int stream_index = f.get_profile().stream_index();
    long long frame_timestamp = 0;

    for (auto&& sen : sensors) {
        
        if (sen.sn == sensor_serial) {
            for (auto&& str : sen.streams) {
                if (str.sp.unique_id() == frame_unique_id) {
                    //std::cout << sen.sn << ", " << s.sp.unique_id() << ", " << std::endl;

                    if (f.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)) {
                        frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                    }
                    else if (f.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)) {
                        frame_timestamp = f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
                        sen.tolerance = sen.tolerance / 1000.0;
                    }
                    else {
                        frame_timestamp = f.get_timestamp();
                        sen.tolerance = sen.tolerance / 1000.0;
                    }
                    if (str.prev_frame_timestamp == 0) str.prev_frame_timestamp = frame_timestamp;
                    auto frame_timestamp_diff = frame_timestamp - str.prev_frame_timestamp;

                    if (logging) {
                        if (frame_timestamp_diff == 0) { //duplicate or no frame md?
                            std::cout << "Duplicated Frame, " << sen.sn << ", " << frame_unique_id << ", " << stream_name << std::endl;
                        }
                        else if (frame_timestamp_diff > sen.tolerance) { //frame drop
                            std::cout << sen.sn << ", " << frame_unique_id << ", " << stream_name << ", " << frame_timestamp_diff << std::endl;
                            str.total_frame_count += 1;
                        }
                        else {
                            str.total_frame_count += 1;
                        }
                    }
                    str.prev_frame_timestamp = frame_timestamp;
                }
            }
        }
    }
}

int main(int argc, char* argv[]) try
{
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::vector<std::string> detected_sns;
    std::vector<std::string> test_sns;
    
    // Capture serial numbers before opening streaming
    for (auto&& dev : ctx.query_devices()) {
        std::string desc = dev.get_description();
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        std::cout << "Found: " << desc << " f/w " << fw << std::endl;
        detected_sns.push_back(serial);
    }

    if (detected_sns.size() < 1) {
        std::cout << " Please connect at least one RealSense Cameras to the host" << std::endl;
        return EXIT_FAILURE;
    }

    if (argc < 2) {
        std::cout << "Please input at least one camera serial numbers to start" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        for (int i = 1; i < argc; i++) {
            std::string sn = argv[i]; 
            std::cout << "Searching " << sn;
            if (std::find(detected_sns.begin(), detected_sns.end(), sn) == detected_sns.end()) {
                std::cout << " ... Not Found" << std::endl;
            }
            else {
                test_sns.push_back(sn);
                std::cout << " ... Found" << std::endl;
            }
        }
    }
    if (!test_sns.size()) {
        std::cout << "Cannot find any camera to start" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        for (int i = 0; i < test_sns.size(); i++) {
            //std::cout << "Testing: " << test_sns.at(i) << std::endl;
        }
    }

    signal(SIGINT, crtlc);

    std::vector<rs2::sensor> active_sensors;

    for (auto&& dev : ctx.query_devices()) {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        if (std::find(test_sns.begin(), test_sns.end(), serial) != test_sns.end()) {

            auto sensors = dev.query_sensors();
            std::cout << "SN: " << serial << " has " << sensors.size() << " modules:" << std::endl;
            for (auto& s : sensors) {
                auto info = std::string(s.get_info(RS2_CAMERA_INFO_NAME));
                auto stream_profiles = s.get_stream_profiles();
                std::vector<rs2::stream_profile > rgb_stream_profile;

                //for (auto& sp : stream_profiles) {
                //    auto vsp = sp.as<rs2::video_stream_profile>();
                //    std::cout << vsp.stream_name() << "," << vsp.stream_index() << "," << vsp.width() << "," << vsp.height() << "," << vsp.fps() << "," << vsp.unique_id() << std::endl;
                //}
                
                std::cout << "... Configuring " << info << std::endl;

                if (info == "RGB Camera") {
                    auto stream_profiles = s.get_stream_profiles();
                    std::vector<rs2::stream_profile > rgb_stream_profile;

                    for (auto& sp : stream_profiles)
                    {
                        auto vsp = sp.as<rs2::video_stream_profile>();
                        if (!(vsp.width() == WIDTH && vsp.height() == HEIGHT && vsp.fps() == FPS)) continue;
                        //Configure RGB Stream Profiles
                        if (sp.stream_type() == RS2_STREAM_COLOR && sp.format() == RS2_FORMAT_RGB8)
                            rgb_stream_profile.push_back(sp);
                    }
                    s.open(rgb_stream_profile);
                    active_sensors.emplace_back(s);
                }
                if (info == "Stereo Module") {
                    auto stream_profiles = s.get_stream_profiles();
                    std::vector<rs2::stream_profile > stereo_stream_profile;
                    for (auto& sp : stream_profiles)
                    {
                        auto vsp = sp.as<rs2::video_stream_profile>();
                        if (!(vsp.width() == WIDTH && vsp.height() == HEIGHT && vsp.fps() == FPS)) continue;
                        //Configure Stereo Stream Profiles
                        if (sp.stream_type() == RS2_STREAM_DEPTH && sp.format() == RS2_FORMAT_Z16)
                            stereo_stream_profile.push_back(sp);
                        if (sp.stream_type() == RS2_STREAM_INFRARED && sp.format() == RS2_FORMAT_Y8 && sp.stream_index() == 1)
                            stereo_stream_profile.push_back(sp);
                        //if (sp.stream_type() == RS2_STREAM_INFRARED && sp.format() == RS2_FORMAT_Y8 && sp.stream_index() == 2)
                        //    stereo_stream_profile.push_back(sp);
                    }
                    s.open(stereo_stream_profile);
                    active_sensors.emplace_back(s);
                }
            }
        }
    }

    std::vector<_sensor> sens;
    for (auto& s : active_sensors) {
        std::string serial = s.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        auto info = std::string(s.get_info(RS2_CAMERA_INFO_NAME));
        auto all_stream_profiles = s.get_stream_profiles();
        auto active_stream_profiles = s.get_active_streams();

        _sensor sen;
        sen.sn = serial;
        sen.num_active_stream = static_cast<int>(active_stream_profiles.size());
        sen.tolerance = static_cast <int>(1.2 * 1000000 / FPS); // current tolerance is 1.2 times of the 1/fps

        for (auto&& stream_profile : active_stream_profiles) {
            _stream s;
            s.sp = stream_profile;
            sen.streams.push_back(s);
        }
        sens.push_back(sen);

        s.start([&sens](rs2::frame f)
            {
                //std::string format = rs2_format_to_string(f.get_profile().format());
                //std::cout << modules[id].sn << ", " << modules[id].count << ", " << modules[id].prev_frame_count << ", " << modules[id].tolerance << std::endl;
                proc_stat(f, sens);
            });
    }

    for (auto&& sen : sens) {
        for (auto&& s : sen.streams) {
            std::cout <<"* " << sen.sn << "-" << s.sp.unique_id() << ", " << s.sp.stream_name();
            if (auto vsp = s.sp.as< rs2::video_stream_profile >())
                std::cout << ", " << vsp.width() << ", " << vsp.height();
            std::cout << ", " << s.sp.format() << ", " << s.sp.fps() << std::endl;
        }
    }

    // Main app loop
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0;

    //std::cout << "Duration(second), " << std::endl;
    while (!stopped) {
        auto t2 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::duration<double>(t2 - t1).count();
        auto t4 = std::chrono::duration<double>(t2 - t0).count();
        
        if (t3 >= 5.0) {
            t1 = t2;
            std::cout << "Time(sec)," << t4;
            for (auto&& sen : sens) {
                for (auto&& str : sen.streams) {
                    std::cout << ",SN," << sen.sn << "-" << str.sp.unique_id() << ",Total," << str.total_frame_count << ",Diff," << str.total_frame_count - str.prev_total_frame_count;
                    str.prev_total_frame_count = str.total_frame_count;
                }
            }
            std::cout << std::endl;
            logging = true;
        }
        //std::this_thread::sleep_for(std::chrono::seconds(1)); // putting thread to sleep may cause drop
    }
    std::cout << "Stopping main proc..." << std::endl;
    stopped = true;

    if (stopped) {
        for (auto& s : active_sensors) {
            auto info = std::string(s.get_info(RS2_CAMERA_INFO_NAME));
            auto stream_profiles = s.get_stream_profiles();
            auto active_streams = s.get_active_streams();
            std::string serial = s.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER);

            std::cout << "Stopping SN: " << serial << ", " << info << ", Number of Active Streams: " << active_streams.size() << std::endl;
            s.stop();
        }
    }
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
