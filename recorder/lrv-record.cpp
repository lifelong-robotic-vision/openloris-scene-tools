// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <memory>
#include <functional>
#include <thread>
#include <string.h>
#include <chrono>
#include <signal.h>

bool running = true;

void intHandler(int sig)
{
    signal(SIGINT, SIG_DFL);
    running = false;
}

int main(int argc, char * argv[])
{
    std::string outfile = (argc > 1) ? std::string(argv[1]) : std::string("record");

    rs2::context ctx; // Create librealsense context for managing devices

    std::vector<rs2::config> cfgs;
    std::vector<std::string> filenames;
    for (auto&& dev : ctx.query_devices())
    {
        std::string camera_name(dev.get_info(RS2_CAMERA_INFO_NAME));
        std::cout << "Found " << camera_name << std::endl;
        if (camera_name.find(std::string("D435I")) != std::string::npos) {
            filenames.push_back(outfile + "-d400.bag");
            cfgs.emplace_back();
            rs2::config &cfg = cfgs.back();
            cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            cfg.enable_record_to_file(filenames.back());
            cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_ACCEL, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 250);
            cfg.enable_stream(RS2_STREAM_GYRO, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 400);
            auto sensors = dev.query_sensors();
            for (auto s : sensors) {
                if (s.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
                    s.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0);
                }
                if (s.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) {
                    float max = s.get_option_range(RS2_OPTION_FRAMES_QUEUE_SIZE).max;
                    s.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, max);
                }
                if (s.supports(RS2_OPTION_VISUAL_PRESET)) {
                    s.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
                }
                if (s.supports(RS2_OPTION_ENABLE_MOTION_CORRECTION)) {
                    s.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
                }
            }
        } else if (camera_name.find(std::string("T265")) != std::string::npos) {
            filenames.push_back(outfile + "-t265.bag");
            cfgs.emplace_back();
            rs2::config &cfg = cfgs.back();
            // Comment out the following line to avoid a bug
            //cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            cfg.enable_record_to_file(filenames.back());
            cfg.enable_stream(RS2_STREAM_FISHEYE, 1, 848, 800, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_FISHEYE, 2, 848, 800, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_ACCEL, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 62);
            cfg.enable_stream(RS2_STREAM_GYRO, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 200);
            cfg.enable_stream(RS2_STREAM_POSE, 0, 0, RS2_FORMAT_6DOF, 200);
        }
    }

    std::mutex m;
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(m);
        auto t = std::chrono::system_clock::now();
        static auto tk = t;
        static auto t0 = t;
        if (t - tk >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(3) << std::fixed
                      << "Recording t = "  << std::chrono::duration_cast<std::chrono::seconds>(t-t0).count() << "s" << std::flush;
            tk = t;
        }
    };

    std::vector<rs2::pipeline> pipelines;
    // We need to start T265 first!! For some reason I can't explain.
    // -- no longer needed but let's keep it anyway
    for (ssize_t i = cfgs.size() - 1; i >= 0 ; --i) {
        pipelines.emplace_back(ctx);
        int retry = 5;
        while (retry) try {
            // enable the callback if desire realtime output of recording length
            pipelines.back().start(cfgs[i]/*, callback*/);
            break;
        } catch (const rs2::error & e) {
            std::cerr << filenames[i] << ": Error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            if (--retry) std::cout << "Will retry for " << retry << " times..." << std::endl;
            continue;
        }
        if (retry) {
            std::cout << filenames[i] << ": start recording" << std::endl;
        } else {
            std::cout << filenames[i] << " failed to start" << std::endl;
            pipelines.pop_back();
        }
    }

    if (pipelines.empty()) return EXIT_FAILURE;
    auto t0 = std::chrono::system_clock::now();
    auto tk = t0;
    signal(SIGINT, intHandler);
    std::cout << "======================= Press Ctrl+C to stop =======================\n";
    while(running) {
        auto t = std::chrono::system_clock::now();
        if (t - tk >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(3) << std::fixed
                      << "Recording t = "  << std::chrono::duration_cast<std::chrono::seconds>(t-t0).count() << "s" << std::flush;
            tk = t;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "\nStopping..." << std::flush;

    for (auto &pipe : pipelines) {
        pipe.stop();
    }
    std::cout << "Stopped" << std::endl;

    return EXIT_SUCCESS;
}
