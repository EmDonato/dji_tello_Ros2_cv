
#include "dji_tello_Ros2/telloComunication.hpp"
#include "dji_tello_Ros2/telloInterface.hpp"
#include <filesystem>
#include <fstream>

/// ============================================================================
/// @brief Parse raw telemetry string into a Telemetry struct.
/// 
/// @param str Input string received from Tello.
/// @return true if parsing succeeded (17 fields parsed), false otherwise.
/// ============================================================================
bool tello::interface::parseTelemetry(const std::string& str) {
    int time_dummy{0};
    int n = std::sscanf(str.c_str(),
        "mid:%*d;x:%*d;y:%*d;z:%*d;mpry:%*d,%*d,%*d;"
        "pitch:%f;roll:%f;yaw:%f;vgx:%d;vgy:%d;vgz:%d;"
        "templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%f;time:%d;"
        "agx:%f;agy:%f;agz:%f;",
        &info_.imu.pitch, &info_.imu.roll, &info_.imu.yaw,
        &info_.imu.vgx, &info_.imu.vgy, &info_.imu.vgz,
        &info_.templ, &info_.temph, &info_.tof, &info_.h, &info_.bat,
        &info_.baro, &time_dummy,
        &info_.imu.agx, &info_.imu.agy, &info_.imu.agz);

    info_.stamp = std::chrono::steady_clock::now();
    if (VERBOSE) std::cout << "[parse] fields=" << n << "\n";
    return (n > 1 );
}

/// @brief Destructor, ensures proper shutdown of sockets and threads.
tello::interface::~interface() { close(); }

// ============================================================================
// Lifecycle
// ============================================================================

/// @brief Initialize sockets for command and telemetry communication.
/// @param video_stream If true, enables video handling.
/// @return 0 on success, 1 on failure.
int tello::interface::begin(bool video_stream) {
    video_on_ = video_stream;

    if (!sock_cmd_.open() || !sock_cmd_.connect(TELLO_IP, PORT_CMD)) {
        std::cerr << "Error: open/connect cmd\n";
        return 1;
    }
    if (!sock_tlm_.open() || !sock_tlm_.bind(LOCAL_IP, PORT_TLM)) {
        std::cerr << "Error: open/bind telemetry\n";
        return 1;
    }
    return 0;
}

/// @brief Enter SDK "command" mode and start background threads.
/// @return 0 on success, 1 on failure.
int tello::interface::connect() {
    const char* c = "command";          
    if (sock_cmd_.send(c, std::strlen(c)) < 0) {
        std::cerr << "Error sending 'command'\n";
        return 1;
    }
    telemetry_run_flag = true;
    command_run_flag   = true;
    th_receiver_ = std::thread(&tello::interface::receiver_loop, this);
    th_sender_   = std::thread(&tello::interface::ack_loop, this);
    return 0;
}

/// @brief Stop threads and close all sockets.
void tello::interface::close() {
    video_on_ = false;  
    sock_vid_.close();
    if (th_rc_.joinable()) th_rc_.join();  


    telemetry_run_flag = false;
    command_run_flag   = false;

    sock_cmd_.close();
    sock_tlm_.close();

    if (th_receiver_.joinable()) th_receiver_.join();
    if (th_sender_.joinable())   th_sender_.join(); 
}

// ============================================================================
// Commands
// ============================================================================

/// @brief Take off the drone.
int tello::interface::takeoff() { const char* c = "takeoff"; return sock_cmd_.send(c, std::strlen(c)) < 0 ? 1 : 0; }

/// @brief Land the drone.
int tello::interface::land()    { const char* c = "land";    return sock_cmd_.send(c, std::strlen(c)) < 0 ? 1 : 0; }

/// @brief Emergency stop.
int tello::interface::stop()    { const char* c = "stop";    return sock_cmd_.send(c, std::strlen(c)) < 0 ? 1 : 0; }

/// @brief Move forward by distance in meters.
int tello::interface::forward(float m) { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="forward "+std::to_string(n);return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Move backward by distance in meters.
int tello::interface::back(float m)    { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="back "+std::to_string(n);   return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Move left by distance in meters.
int tello::interface::left(float m)    { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="left "+std::to_string(n);   return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Move right by distance in meters.
int tello::interface::right(float m)   { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="right "+std::to_string(n);  return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Move upward by distance in meters.
int tello::interface::up(float m)      { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="up "+std::to_string(n);     return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Move downward by distance in meters.
int tello::interface::down(float m)    { m=clamp(m,MIN_DIST_M,MAX_DIST_M);int n=meters_to_cm(m);std::string str="down "+std::to_string(n);   return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Rotate clockwise by degrees.
/// @param d Rotation angle [1, 360].
int tello::interface::turn_cw(int d)   { if(d<1)d=1;if(d>360)d=360;std::string str="cw "+std::to_string(d);  return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Rotate counterclockwise by degrees.
/// @param d Rotation angle [1, 360].
int tello::interface::turn_ccw(int d)  { if(d<1)d=1;if(d>360)d=360;std::string str="ccw "+std::to_string(d); return sock_cmd_.send(str.c_str(),str.size())<0?1:0; }

/// @brief Continuous joystick-style movement.
/// @param a Left-right [-100, 100].
/// @param b Forward-back [-100, 100].
/// @param h_vel Up-down [-100, 100].
/// @param c Yaw [-100, 100].
int tello::interface::move(int a, int b, int h_vel, int c) {
    a=clamp_rc(a);b=clamp_rc(b);h_vel=clamp_rc(h_vel);c=clamp_rc(c);
    std::string str="rc "+std::to_string(a)+" "+std::to_string(b)+" "+std::to_string(h_vel)+" "+std::to_string(c);
    return sock_cmd_.send(str.c_str(),str.size())<0?1:0;
}

// ============================================================================
// Raw command
// ============================================================================

/// @brief Send a raw SDK command string.
/// @param sdk_cmd String containing SDK command.
/// @return 0 on success, 1 on failure.
int tello::interface::cmd_raw(const std::string& sdk_cmd, int, int) {
    if (sdk_cmd.empty()) return 1;
    std::cout << "[SEND RAW] " << sdk_cmd << "\n";
    return sock_cmd_.send(sdk_cmd.c_str(), sdk_cmd.size()) < 0 ? 1 : 0;
}

// ============================================================================
// Logging
// ============================================================================

/// @brief Write telemetry data into a log file ("Log/telemetry.txt").
void tello::interface::log() {
    namespace fs = std::filesystem;
    // Se vuoi il log nella dir di lancio:
    fs::path logDir = fs::current_path() / "Log";
    if (!fs::exists(logDir)) {
        if (!fs::create_directory(logDir)) {
            std::cerr << "Failed to create Log directory at " << logDir << "\n";
            return;
        }
    }
    std::ofstream ofs(logDir / "telemetry.txt", std::ios::app);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open log file: " << (logDir / "telemetry.txt") << "\n";
        return;
    }
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ofs << std::ctime(&now)
        << " Pitch=" << info_.imu.pitch
        << " Roll=" << info_.imu.roll
        << " Yaw=" << info_.imu.yaw
        << " H=" << info_.h
        << " Bat=" << info_.bat
        << " ToF=" << info_.tof
        << " Baro=" << info_.baro
        << " Agx=" << info_.imu.agx
        << " Agy=" << info_.imu.agy
        << " Agz=" << info_.imu.agz
        << std::endl;
    if (VERBOSE) std::cout << "[log] wrote to " << (logDir / "telemetry.txt") << "\n";
}

// ============================================================================
// Video and Camera switch
// ============================================================================

/// @brief Toggle camera between forward and downward.
/// @return 0 on success, 1 on failure.
int tello::interface::switchCamera() {
    if (!video_on_) return 0;
    if (!camera_flag) {
        const char* c = "downvision 1";
        if (sock_cmd_.send(c, std::strlen(c)) < 0) return 1;
        camera_flag = true;
    } else {
        const char* c = "downvision 0";
        if (sock_cmd_.send(c, std::strlen(c)) < 0) return 1;
        camera_flag = false;
    }
    return 0;
}

/// @brief Enable video stream and start video thread.
int tello::interface::video_on() {
    const char* c = "streamon";
    if (sock_cmd_.send(c, std::strlen(c)) < 0) return 1;
    video_on_ = true;  
    if(video_on_) th_rc_ = std::thread(&tello::interface::video_loop, this);   
    return 0;   
}

/// @brief Disable video stream.
int tello::interface::video_off() {
    if (!video_on_) return 0;
    const char* c = "streamoff";
    sock_cmd_.send(c, std::strlen(c));  
    return 0; 
}

// ============================================================================
// Threads
// ============================================================================

/// @brief Thread loop: receive telemetry packets, parse, queue, and log.
void tello::interface::receiver_loop() {
    char buf[1024];
    while (telemetry_run_flag) {
        int n = sock_tlm_.recv(buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = '\0';
            if (VERBOSE) std::cout << buf << "\n";
            if (parseTelemetry( std::string(buf))){
                std::lock_guard<std::mutex> lk(mtx_tlm_);
                queue_tlm_.push(info_);
                if (VERBOSE) std::cout << "[TLM Parsed] h=" << info_.h << " bat=" << info_.bat << "\n";
                log();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

/// @brief Thread loop: listen for ACK messages on command socket.
void tello::interface::ack_loop() {
    char buf[1024];
    while (command_run_flag) {
        int n = sock_cmd_.recv(buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = '\0';
            if (VERBOSE_ACK) std::cout << "[ACK] " << buf << "\n";
            flagAck = true;
    }
}
}

/// @brief Wait until ACK flag is set true.
void tello::interface::waitAck() {
    while (!flagAck) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
void tello::interface::setAck(bool logic_value){
    flagAck = logic_value;
}

/// @brief Thread loop: handle video stream frames with OpenCV.
void tello::interface::video_loop() {
    cv::VideoCapture cap("udp://0.0.0.0:11111", cv::CAP_FFMPEG);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cv::Mat frame;
    while (video_on_) {
        if (!cap.read(frame)) continue;
        {
            std::lock_guard<std::mutex> lk(mtx_frame_);
            frame.copyTo(last_frame_);
        }

        if(show == true) cv::imshow("Tello Video", frame);
        if (cv::waitKey(1) == 27) break;
    }
}

// ============================================================================
// Telemetry queue
// ============================================================================

/// @brief Pop one telemetry packet from the queue (thread-safe).
/// @return the structure telemetry with the current info
tello::Telemetry tello::interface::popTelemetry() {
    std::lock_guard<std::mutex> lk(mtx_tlm_);
    if (queue_tlm_.empty()) {
        return Telemetry{}; // ritorna struct vuota se non c’è nulla
    }
    Telemetry out = queue_tlm_.front();
    queue_tlm_.pop();
    return out;
}
bool tello::interface::get_last_frame(cv::Mat& out) {
    std::lock_guard<std::mutex> lk(mtx_frame_);
    if (last_frame_.empty()) {
        return false;                 // nessun frame disponibile ancora
    }
    last_frame_.copyTo(out);          // deep copy: thread-safe per il chiamante
    return true;
}