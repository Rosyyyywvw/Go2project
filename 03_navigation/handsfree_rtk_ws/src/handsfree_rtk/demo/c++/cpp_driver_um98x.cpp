// cpp_driver_um98x.cpp
// NMEA RTK 串口读取（非ROS，跨平台：Ubuntu/Windows）
// - 功能：串口读取 → 解析 GGA / RMC  / THS→ 控制台打印（原始行 + 结构化信息）
// - 依赖：Boost.Asio（建议安装 libboost-all-dev 或通过 vcpkg 安装 Boost）
// - 用法示例：
// 编译：
//    Linux:    g++ -std=c++11 -O2 cpp_driver_um98x.cpp -o cpp_driver_um98x -lboost_system -lpthread
//    Windows:    g++ -std=gnu++11 -O2 cpp_driver_um98x.cpp -o cpp_driver_um98x.exe -lboost_system -lws2_32 -lmswsock
// 运行：
//   Linux:   ./cpp_driver_um98x --port /dev/ttyUSB0 --baud 115200 --no-raw
//   Windows: .\cpp_driver_um98x.exe --port COM6 --baud 115200

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <regex>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <mutex>
#include <limits>

#if !defined(_WIN32)
  #include <unistd.h>
  #include <fcntl.h>
#endif

#include <boost/version.hpp>
#include <boost/system/error_code.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
namespace asio = boost::asio;

// 兼容别名：新 Boost 用 io_context，老 Boost 用 io_service
#if BOOST_VERSION >= 106600
using io_t = boost::asio::io_context;
#else
using io_t = boost::asio::io_service;
#endif


#if defined(_WIN32)
  #include <windows.h>
#endif

using boost::asio::ip::tcp;
namespace asio = boost::asio;

// ---------------------------
// 简单日志（含UTC时间）
// ---------------------------
static std::mutex g_log_mu;

static std::string now_utc_string() {
    using namespace std::chrono;
    auto tp = system_clock::now();
    auto tt = system_clock::to_time_t(tp);
    auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;

    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &tt);
#else
    gmtime_r(&tt, &tm_utc);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%d %H:%M:%S") << "."
        << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

static void log_line(const char* level, const std::string& msg) {
    std::lock_guard<std::mutex> lk(g_log_mu);
    std::cout << "[" << now_utc_string() << "][" << level << "] " << msg << std::endl;
}

// ---------------------------
// 定位状态映射（UM982/通用风格）
// ---------------------------
struct FixInfo {
    const char* desc;
    const char* status_code;
    double cov_m; // 协方差的标准差（米）
};

static const FixInfo FIX_TYPE_MAPPING[9] = {
    {"Invalid Fix", "NO_FIX", 10000.0},
    {"GPS Fix (SPS)", "FIX", 1.0},
    {"DGPS Fix", "SBAS_FIX", 0.5},
    {"PPS Fix", "NO_FIX", 10000.0},
    {"RTK Fixed", "GBAS_FIX", 0.01},
    {"RTK Float", "GBAS_FIX", 0.1},
    {"Estimated", "FIX", 5.0},
    {"Manual", "GBAS_FIX", 0.01},
    {"Simulation", "FIX", 10000.0}
};

// ---------------------------
// 工具：DMS -> 十进制度
// dms: 如 "2231.86004675", direction: 'N'/'S'/'E'/'W'
// ---------------------------
static bool dms_to_decimal(const std::string& dms, const std::string& dir, double& out_deg) {
    try {
        if (dms.empty()) return false;
        std::regex re(R"(^(\d+)(\d\d\.\d+)$)");
        std::smatch m;
        if (!std::regex_match(dms, m, re)) return false;
        double degrees = std::stod(m[1].str());
        double minutes = std::stod(m[2].str());
        double decimal = degrees + minutes / 60.0;
        if (!dir.empty() && (dir[0] == 'S' || dir[0] == 'W')) decimal = -decimal;
        out_deg = decimal;
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------
// 解析 NMEA 时间 HHMMSS.SS → 把当天 UTC 时间补上（仅打印展示用）
// ---------------------------
static std::string nmea_time_to_utc_string(const std::string& hhmmss) {
    if (hhmmss.size() < 6) return now_utc_string();
    try {
        int hh = std::stoi(hhmmss.substr(0,2));
        int mm = std::stoi(hhmmss.substr(2,2));
        double ss_f = std::stod(hhmmss.substr(4));
        int ss = static_cast<int>(ss_f);
        int usec = static_cast<int>((ss_f - ss) * 1e6);

        using namespace std::chrono;
        auto tt = std::time(nullptr);
        std::tm tm_utc{};
#if defined(_WIN32)
        gmtime_s(&tm_utc, &tt);
#else
        gmtime_r(&tt, &tm_utc);
#endif
        tm_utc.tm_hour = hh;
        tm_utc.tm_min  = mm;
        tm_utc.tm_sec  = ss;
#if defined(_WIN32)
        // 没有 timegm，使用 _mkgmtime
        std::time_t t2 = _mkgmtime(&tm_utc);
#else
        std::time_t t2 = timegm(&tm_utc);
#endif
        auto today = std::chrono::system_clock::from_time_t(t2) + std::chrono::microseconds(usec);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(today.time_since_epoch()) % 1000;
        std::tm out_tm{};
        auto tt2 = std::chrono::system_clock::to_time_t(today);
#if defined(_WIN32)
        gmtime_s(&out_tm, &tt2);
#else
        gmtime_r(&tt2, &out_tm);
#endif
        std::ostringstream oss;
        oss << std::put_time(&out_tm, "%Y-%m-%d %H:%M:%S")
            << "." << std::setw(3) << std::setfill('0') << ms.count();
        return oss.str();
    } catch (...) {
        return now_utc_string();
    }
}

// ---------------------------
// 数据结构：解析结果
// ---------------------------
struct Parsed {
    std::string type; // "GNGGA" or "GNRMC"
    std::string utc_time_str; // 拼出带日期的UTC字符串（仅显示）
    // GGA
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int fix_quality = 0;
    int num_sats = 0;
    double hdop = 99.9;
    bool has_gga = false;
    // RMC
    bool status_valid = false;
    double speed_mps = 0.0;
    double raw_speed_knots = 0.0;
    double cog_deg = 0.0;
    bool has_rmc = false;
    // THS (True Heading)
    double heading_deg = 0.0;
    bool ths_valid = false;
    bool has_ths = false;
};

// ---------------------------
// 句子规范化（修剪不可见字符；缺$时补上）
// ---------------------------
static void normalize_sentence(std::string& s) {
    while (!s.empty() && (unsigned char)s.front() <= 0x20) s.erase(s.begin());
    while (!s.empty() && (unsigned char)s.back()  <= 0x20) s.pop_back();
    if (s.empty()) return;

    auto starts_with = [&](const char* p){ return s.rfind(p, 0) == 0; };
    if (s[0] != '$' && (starts_with("GNRMC") || starts_with("GPRMC") ||
                        starts_with("GNGGA") || starts_with("GPGGA") ||
                        starts_with("GNTHS") || starts_with("GPTHS") ||
                        starts_with("GNVTG") || starts_with("GNZDA") ||
                        starts_with("GNGSA") || starts_with("GNGST"))) {
        s.insert(s.begin(), '$');
    }
}

// ---------------------------
// 解析 NMEA 行（支持 GNGGA, GNRMC/GPRMC）
// ---------------------------
static bool parse_nmea_sentence(const std::string& line, Parsed& out) {
    try {
        if (line.rfind("$GNGGA", 0) == 0) {
            std::vector<std::string> p; p.reserve(20);
            for (size_t i=0, j=0; i<=line.size(); ++i) {
                if (i==line.size() || line[i]==',') { p.emplace_back(line.substr(j, i-j)); j=i+1; }
            }
            if (p.size() < 10) return false;

            out.type = "GNGGA";
            out.utc_time_str = nmea_time_to_utc_string(p[1]);

            double lat=0.0, lon=0.0;
            dms_to_decimal(p[2], p[3], lat);
            dms_to_decimal(p[4], p[5], lon);
            out.latitude = lat;
            out.longitude = lon;

            try { out.fix_quality = p[6].empty()?0:std::stoi(p[6]); } catch(...) { out.fix_quality=0; }
            try { out.num_sats = p[7].empty()?0:std::stoi(p[7]); }   catch(...) { out.num_sats=0; }
            try { out.hdop     = p[8].empty()?99.9:std::stod(p[8]); } catch(...) { out.hdop=99.9; }
            try { out.altitude = p[9].empty()?0.0:std::stod(p[9]); }  catch(...) { out.altitude=0.0; }

            out.has_gga = true;
            return true;
        }

        if (line.rfind("$GNRMC", 0) == 0 || line.rfind("$GPRMC", 0) == 0) {
            std::vector<std::string> p; p.reserve(20);
            for (size_t i=0, j=0; i<=line.size(); ++i) {
                if (i==line.size() || line[i]==',') { p.emplace_back(line.substr(j, i-j)); j=i+1; }
            }
            if (p.size() < 12) return false;

            out.type = "GNRMC";
            out.utc_time_str = nmea_time_to_utc_string(p[1]);

            char status = p[2].empty()? 'V':p[2][0];
            out.status_valid = (status=='A');

            double lat=0.0, lon=0.0;
            dms_to_decimal(p[3], p[4], lat);
            dms_to_decimal(p[5], p[6], lon);
            out.latitude = lat;
            out.longitude = lon;

            try { out.raw_speed_knots = p[7].empty()?0.0:std::stod(p[7]); } catch(...) { out.raw_speed_knots=0.0; }
            try { out.cog_deg         = p[8].empty()?0.0:std::stod(p[8]); } catch(...) { out.cog_deg=0.0; }
            out.speed_mps = out.raw_speed_knots * 0.514444;

            out.has_rmc = true;
            return true;
        }

        if (line.rfind("$GNTHS", 0) == 0 || line.rfind("$GPTHS", 0) == 0) {
            // 典型格式: $GNTHS,230.1081,A*10
            std::vector<std::string> p; p.reserve(8);
            for (size_t i=0, j=0; i<=line.size(); ++i) {
                if (i==line.size() || line[i]==',') { p.emplace_back(line.substr(j, i-j)); j=i+1; }
            }
            if (p.size() < 3) return false;

            out.type = "GNTHS";
            try { out.heading_deg = p[1].empty()? 0.0 : std::stod(p[1]); } catch(...) { out.heading_deg = 0.0; }

            // 第三个字段形如 "A*10" 或 "V*hh"；用 starts_with('A') 判有效
            out.ths_valid = !p[2].empty() && (p[2][0] == 'A');

            out.has_ths = true;
            return true;
        }
        return false;
    } catch (...) {
        return false;
    }
}

// ---------------------------
// Windows 串口名适配（COM10+）
// ---------------------------
static std::string normalize_port_name(const std::string& in) {
#if defined(_WIN32)
    // 对于 COM1..COM9 直接用即可；COM10及以上需要加前缀
    if (in.size() >= 4 && (in.rfind("COM", 0) == 0 || in.rfind("com", 0) == 0)) {
        try {
            int num = std::stoi(in.substr(3));
            if (num >= 10) return std::string("\\\\.\\") + in;
        } catch (...) {}
    }
#endif
    return in;
}

// ---------------------------
// 串口读取器
// ---------------------------
class SerialNMEAReader {
public:
    SerialNMEAReader(std::string port, int baud, double timeout_sec, bool print_raw)
        : port_(normalize_port_name(port)),
          baud_(baud),
          timeout_ms_(static_cast<int>(timeout_sec * 1000)),
          print_raw_(print_raw),
          io_(),
          serial_(io_),
          stop_(false),
          need_reopen_(false) {}

    // 运行主循环（打开串口→读取→异常后自动重连）
    void run() {
        // 首启阻塞式无限重试
        if (!open_port_blocking()) {
            log_line("ERROR", "Start aborted.");
            return;
        }
        log_line("INFO", "Serial opened: " + port_);

        while (!stop_.load()) {
            // 内层读取循环
            while (!stop_.load()) {
                std::string line;
                if (!readline(line)) {
                    // 超时或错误
                    if (stop_.load()) break;
                    if (need_reopen_.load()) {
                        log_line("WARN", "Serial error occurred, will reopen port...");
                        need_reopen_.store(false);
                        break; // 跳出内层循环，走 close→reopen
                    }
                    // 仅超时：轻微等待，避免忙等
                    if (wait_or_stopped(1)) break;
                    continue;
                }

                if (print_raw_) std::cout << line << std::endl;
                normalize_sentence(line);
                // 打印关键 NMEA 句子
                if (line.rfind("$GNGGA",0)==0 || line.rfind("$GPGGA",0)==0 || 
                    line.rfind("$GNRMC",0)==0 || line.rfind("$GPRMC",0)==0 || 
                    line.rfind("$GNTHS",0)==0 || line.rfind("$GPTHS",0)==0) {
                    log_line("INFO", line);
                }

                Parsed parsed;
                if (!parse_nmea_sentence(line, parsed)) continue;

                if (parsed.has_gga) {
                    int q = parsed.fix_quality;
                    const FixInfo* fi = (q>=0 && q<=8) ? &FIX_TYPE_MAPPING[q] : nullptr;
                    std::ostringstream oss;
                    oss << "GGA: fix=" << q << " (" << (fi?fi->desc:"Unknown") << ")"
                        << ", sats=" << parsed.num_sats
                        << ", hdop=" << std::fixed << std::setprecision(2) << parsed.hdop
                        << ", lat=" << std::setprecision(8) << parsed.latitude
                        << ", lon=" << std::setprecision(8) << parsed.longitude
                        << ", alt=" << std::setprecision(3) << parsed.altitude << " m";
                    log_line("INFO", oss.str());
                } else if (parsed.has_rmc) {
                    std::ostringstream oss;
                    oss << "RMC: status=" << (parsed.status_valid ? "effective" : "invalid")
                        << ", speed=" << std::fixed << std::setprecision(3) << parsed.speed_mps
                        << " m/s (" << parsed.raw_speed_knots << " kn)"
                        << ", COG=" << std::setprecision(2) << parsed.cog_deg << " deg";
                    log_line("INFO", oss.str());
                } else if (parsed.has_ths) {
                    std::ostringstream oss;
                    oss << "THS: heading=" << std::fixed << std::setprecision(3) << parsed.heading_deg
                        << " deg (" << (parsed.ths_valid ? "valid" : "invalid") << ")";
                    log_line("INFO", oss.str());
                }
            }

            close_port();
            if (stop_.load()) break;

            log_line("INFO", "Reconnecting in 1s...");
            if (wait_or_stopped(1000)) break;

            if (!open_port_blocking()) break;
            log_line("INFO", "Serial reopened: " + port_);
        }
        log_line("INFO", "Reader stopped.");
    }

    void stop() {
        stop_.store(true);
        close_port();
    }

private:
    // 首启/重连：阻塞式无限重试打开串口
    bool open_port_blocking() {
        double backoff = 0.5; // 0.5s 起，×1.5，封顶 5s
        while (!stop_.load()) {
            if (open_port()) return true;
            log_line("WARN", "Open port failed, will retry...");
            int ms = static_cast<int>(backoff * 1000);
            if (wait_or_stopped(ms)) return false;
            backoff = std::min(5.0, backoff * 1.5);
        }
        return false;
    }

    bool open_port() {
        try {
            serial_.open(port_);
            // 串口参数
            serial_.set_option(asio::serial_port_base::baud_rate(baud_));
            serial_.set_option(asio::serial_port_base::character_size(8));
            serial_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            serial_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            serial_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
            // 非阻塞/短超时
            tune_serial_nonblocking();
            return true;
        } catch (const std::exception& e) {
            log_line("ERROR", std::string("Failed to open port: " + port_ + ",") + e.what());
            return false;
        }
    }

    void tune_serial_nonblocking() {
#if defined(_WIN32)
        HANDLE h = reinterpret_cast<HANDLE>(serial_.native_handle());
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout         = 1;
        to.ReadTotalTimeoutMultiplier  = 0;
        to.ReadTotalTimeoutConstant    = 1;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant   = 0;
        SetCommTimeouts(h, &to);
#else
        int fd = serial_.native_handle();
        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
    }

    void close_port() {
        try {
            if (serial_.is_open()) {
                std::string p = port_;
                serial_.close();
                log_line("INFO", "Serial closed: " + p);
            }
        } catch (const std::exception& e) {
            log_line("WARN", std::string("Close warning: ") + e.what());
        }
    }

    // 同步读取一行（以 '\n' 结束），带超时；致命错误会置位 need_reopen_
    bool readline(std::string& out) {
        out.clear();
        if (!serial_.is_open()) { need_reopen_.store(true); return false; }

        char c = 0;
        boost::system::error_code ec;
        auto start = std::chrono::steady_clock::now();

        while (!stop_.load()) {
            std::size_t n = serial_.read_some(asio::buffer(&c, 1), ec);
            if (ec) {
                if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else { need_reopen_.store(true); return false; }
            } else if (n == 1) {
                if (c == '\r' || c == '\n') {
                    if (!out.empty()) return true;  // 拿到一整句
                    // 连续分隔符→继续
                } else {
                    out.push_back(c);
                }
            }
            if (timeout_ms_ > 0 &&
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count() >= timeout_ms_) return false;
        }
        return false;
    }

    bool wait_or_stopped(int ms) {
        for (int i=0;i<ms && !stop_.load();++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return stop_.load();
    }

private:
    std::string port_;
    int baud_;
    int timeout_ms_;
    bool print_raw_;

    io_t io_;
    asio::serial_port serial_;
    std::atomic<bool> stop_;
    std::atomic<bool> need_reopen_;
};

// ---------------------------
// 简易命令行参数
// ---------------------------
struct Options {
    std::string port =
#if defined(_WIN32)
        "COM6";
#else
        "/dev/ttyUSB0";
#endif
    int baud = 115200;
    double timeout = 0.1; // 秒
    bool no_raw = false;
};

static void print_usage(const char* exe) {
    std::cout << "Usage: " << exe << " --port <PORT> --baud <BAUD> [--timeout <sec>] [--no-raw]\n"
              << "  e.g. Linux:   " << exe << " --port /dev/ttyUSB0 --baud 115200\n"
              << "       Windows: " << exe << " --port COM6 --baud 115200\n";
}

static bool parse_args(int argc, char** argv, Options& opt) {
    for (int i=1;i<argc;++i) {
        std::string a = argv[i];
        if (a=="--port" && i+1<argc) { opt.port = argv[++i]; }
        else if (a=="--baud" && i+1<argc) { opt.baud = std::stoi(argv[++i]); }
        else if (a=="--timeout" && i+1<argc) { opt.timeout = std::stod(argv[++i]); }
        else if (a=="--no-raw") { opt.no_raw = true; }
        else if (a=="-h" || a=="--help") { return false; }
        else { std::cerr << "Unknown arg: " << a << "\n"; return false; }
    }
    return true;
}

// ---------------------------
// Ctrl+C 处理
// ---------------------------
static SerialNMEAReader* g_reader = nullptr;

#if defined(_WIN32)
#include <windows.h>
BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType) {
    if (dwCtrlType == CTRL_C_EVENT || dwCtrlType == CTRL_BREAK_EVENT || dwCtrlType == CTRL_CLOSE_EVENT) {
        if (g_reader) g_reader->stop();
        return TRUE;
    }
    return FALSE;
}
#else
void sigint_handler(int) {
    if (g_reader) g_reader->stop();
}
#endif

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) {
        print_usage(argv[0]);
        return 0;
    }

#if defined(_WIN32)
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#else
    std::signal(SIGINT,  sigint_handler);
    std::signal(SIGTERM, sigint_handler);
#endif

    SerialNMEAReader reader(opt.port, opt.baud, opt.timeout, !opt.no_raw);
    g_reader = &reader;

    log_line("INFO", "Starting NMEA reader...");
    reader.run();
    log_line("INFO", "Exit.");
    return 0;
}
