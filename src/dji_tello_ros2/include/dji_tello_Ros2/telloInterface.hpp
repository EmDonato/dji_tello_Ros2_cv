#pragma once

#include "telloComunication.hpp" 

#include <string>
#include <thread>
#include <queue>
#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>

#define VERBOSE 0
#define VERBOSE_ACK 1
namespace tello {

// Alias for steady clock timestamp
using steady_time = std::chrono::steady_clock::time_point;

/**
 * @brief IMU telemetry data
 */
struct IMU {
    float pitch{0.f}, roll{0.f}, yaw{0.f}; ///< attitude in degrees
    int   vgx{0}, vgy{0}, vgz{0};          ///< velocity along x/y/z in cm/s
    float agx{0.f}, agy{0.f}, agz{0.f};    ///< acceleration in m/s^2
};

/**
 * @brief Complete telemetry data from Tello
 */
struct Telemetry {
    IMU   imu;                 ///< attitude, velocity, acceleration
    int   templ{0}, temph{0};  ///< temperature min/max in °C
    int   tof{0};              ///< time of flight sensor (cm)
    int   h{0};                ///< height (cm)
    int   bat{0};              ///< battery level (%)
    float baro{0.f};           ///< barometer altitude (m)
    steady_time stamp{};       ///< timestamp when received
};



/**
 * @brief Main interface class to control DJI Tello and receive telemetry
 */
class interface {
    private:
        // Network constants
        static constexpr const char* TELLO_IP   = "192.168.10.1";
        static constexpr const char* LOCAL_IP   = "0.0.0.0";
        static constexpr int PORT_CMD = 8889;
        static constexpr int PORT_TLM = 8890;
        static constexpr int PORT_VID = 11111;

        // API command limits
        static constexpr float MIN_DIST_M   = 0.20f; // 20 cm
        static constexpr float MAX_DIST_M   = 5.00f; // 500 cm
        static constexpr float MIN_SPEED_MS = 0.10f; // 10 cm/s
        static constexpr float MAX_SPEED_MS = 1.00f; // 100 cm/s

        //Telemetry
        Telemetry info_;
    public:
        interface() = default;
        ~interface();

        // === Lifecycle ===
        int  begin(bool video_stream = false); ///< Open sockets (command & telemetry)
        int  connect();                        ///< Send "command" to enter SDK mode
        void close();                          ///< Stop threads and close sockets

        // === High-level API ===
        int  set_speed(float mps);             ///< Set movement speed (m/s)
        int  takeoff();                        ///< Take off
        int  land();                           ///< Land
        int  stop();                           ///< Stop motors
        int  forward(float m);                 ///< Move forward (meters)
        int  back(float m);                    ///< Move backward (meters)
        int  left(float m);                    ///< Move left (meters)
        int  right(float m);                   ///< Move right (meters)
        int  up(float m);                      ///< Move upward (meters)
        int  down(float m);                    ///< Move downward (meters)
        int  turn_cw(int deg);                 ///< Rotate clockwise (degrees)
        int  turn_ccw(int deg);                ///< Rotate counterclockwise (degrees)

        // === RC mode ===
        int  move(int a, int b, int h_vel, int c); ///< Direct RC control [-100..100]

        // === Raw SDK command ===
        int  cmd_raw(const std::string& sdk_cmd, int timeout_ms = 1500, int retries = 0);

        // === Video ===
        int video_on();      ///< Enable video streaming
        int video_off();     ///< Disable video streaming
        int switchCamera();  ///< Switch between forward and downward camera

        // === Logging ===
        void log(); ///< Append telemetry to Log/telemetry.txt

        // === Telemetry access ===
        /**
         * @brief Pop one telemetry packet from queue (non-blocking).
         * @return the structure of the telemetry
         */
        Telemetry popTelemetry();
        //ACK
        void waitAck();
        void setAck(bool logic_value);
        // Public queue (kept for ROS2 integration) 
        std::queue<Telemetry> queue_tlm_; ///< shared telemetry data
        std::mutex mtx_tlm_;              ///< mutex for telemetry queue
        /**
         * @brief Get a deep copy of the latest video frame.
         * @param[out] out Receives a deep copy of the last frame.
         * @return true if a valid frame was available, false otherwise.
         */
        bool get_last_frame(cv::Mat& out);
        //camera 
        bool show = false;

    private:
        // === Internal worker threads ===
        void ack_loop();       ///< Read ACKs from command socket
        void receiver_loop();  ///< Read and parse telemetry packets
        void video_loop();     ///< Video streaming loop

        // === State flags ===
        std::atomic_bool telemetry_run_flag{false};
        std::atomic_bool command_run_flag{false};
        std::atomic_bool video_on_{false};
        std::atomic_bool camera_flag{false};
        std::atomic_bool flagAck{false};

        // === Helpers ===
        static int meters_to_cm(float m)  { return (int)std::lround(m * 100.f); }
        static int mps_to_cms(float ms)   { return (int)std::lround(ms * 100.f); }
        static int clamp_rc(int v)        { return v < -100 ? -100 : (v > 100 ? 100 : v); }
        static float clamp(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        /**
         * @brief Parse raw telemetry string into Telemetry struct
         * @param str Raw telemetry line from UDP
         * @param out Output struct
         * @return true if parsing was successful
         */
        bool parseTelemetry(const std::string& str);


        // === Network sockets ===
        UDP_socket_tello sock_cmd_;
        UDP_socket_tello sock_tlm_;
        UDP_socket_tello sock_vid_;

        // === Threads ===
        std::thread th_sender_;
        std::thread th_receiver_;
        std::thread th_rc_;

        // === Video buffer ===
        cv::Mat last_frame_;
        std::mutex mtx_frame_;
    };

} // namespace tello
