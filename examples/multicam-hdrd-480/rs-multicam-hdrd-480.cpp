#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <iomanip>

// #include "hdr_fusion.hpp"

int main(int argc, char * argv[]) try
{
    std::string rs_serial = argv[1];

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::vector<rs2::pipeline>            pipelines;

    for (auto&& dev : ctx.query_devices()) {
        std::cout << "Resetting device: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        dev.hardware_reset();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices()) {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "Detected device with serial " << serial << std::endl;
        if (serial == rs_serial)
        {
            auto sensor = dev.query_sensors().front();
#if 1
            /* enable hdrd mode*/
            std::cout << "Enabling HDRD mode for " << serial << std::endl;

            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
            if (sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                //sensor.set_option(RS2_OPTION_EXPOSURE, 16000);
            }
                
            // setting the HDR sequence size to 2 frames
            if (sensor.supports(RS2_OPTION_SEQUENCE_SIZE))
                sensor.set_option(RS2_OPTION_SEQUENCE_SIZE, 2);
            // configuring id for this hdr config (value must be in range [0,3])
            sensor.set_option(RS2_OPTION_SEQUENCE_NAME, 0);
            // configuration for the first HDR sequence ID
            sensor.set_option(RS2_OPTION_SEQUENCE_ID, 1);
            sensor.set_option(RS2_OPTION_EXPOSURE, 15500); // setting exposure to 16000, so sequence 1 will be set to high exposure
            sensor.set_option(RS2_OPTION_GAIN, 25); // setting gain to 25, so sequence 1 will be set to high gain
            // configuration for the second HDR sequence ID
            sensor.set_option(RS2_OPTION_SEQUENCE_ID, 2);
            sensor.set_option(RS2_OPTION_EXPOSURE, 2000);  // setting exposure to 2000, so sequence 2 will be set to low exposure
            sensor.set_option(RS2_OPTION_GAIN, 16); // setting gain to 16, so sequence 2 will be set to low gain
            // turning ON the HDR with the above configuration
            sensor.set_option(RS2_OPTION_HDR_ENABLED, 1);
#endif
            //Enable Global Time Domain
            if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);

            serials.push_back(serial);
        }

    }

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        if (serial == rs_serial) // RS sensor - depth stream
        {
            rs2::pipeline pipe(ctx);
            rs2::config cfg;

            cfg.enable_stream(
                RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60);
            cfg.enable_stream(
                RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 60);

            cfg.enable_device(serial);

            pipe.start(cfg);
            pipelines.emplace_back(pipe);
        }
    }

    std::mutex my_lock;

    rs2::frame_queue rs_frames_queue(5);

    int prev_rs_frame_count = 0;

    std::thread proc_rs([&]() {
        while (true)
        {
            rs2::frameset fs;
            if (rs_frames_queue.poll_for_frame(&fs))
            {
                my_lock.lock();

                auto f_depth = fs.get_depth_frame();
                auto rs_frame_count = f_depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
                if (prev_rs_frame_count == 0) prev_rs_frame_count = rs_frame_count;
                if (rs_frame_count % 150 == 0)
                    std::cout << "\033[32m" << "RS frame: " << rs_frame_count  << "\033[0m" << std::endl;
                if (rs_frame_count - prev_rs_frame_count == 2)
                {
                    std::cout << "\033[33m" << "RS frame dropped: " << rs_frame_count  << " ! (" << rs_frame_count - prev_rs_frame_count << ")" << "\033[0m" << std::endl;
                }
                else if (rs_frame_count - prev_rs_frame_count > 2)
                {
                    std::cout << "\033[31m" << "RS frame dropped: " << rs_frame_count  << " ! (" << rs_frame_count - prev_rs_frame_count << ")" << "\033[0m" << std::endl;
                }

                prev_rs_frame_count = rs_frame_count;

                my_lock.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
        }
    });
    proc_rs.detach();

    int pre_frame_md_ts = 0;
    double pre_frame_ts = 0;
    // Main app loop
    while (true)
    {
        for (auto &&pipe : pipelines)
        {
            //rs2::frameset fs = pipe.wait_for_frames();
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs)) {
              auto f_depth = fs.get_depth_frame();
              
              // yhsu debug
              auto frame_md_ts = f_depth.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
              auto frame_ts = f_depth.get_timestamp();
              std::cout << "** " << std::setprecision(17) << frame_ts-pre_frame_ts << ", " << frame_md_ts-pre_frame_md_ts << std::endl;
              pre_frame_md_ts = frame_md_ts;
              pre_frame_ts = frame_ts;
              // yhsu debug end

              std::string serial = rs2::sensor_from_frame(f_depth)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
              if (serial == rs_serial) {
                rs_frames_queue.enqueue(fs);
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
