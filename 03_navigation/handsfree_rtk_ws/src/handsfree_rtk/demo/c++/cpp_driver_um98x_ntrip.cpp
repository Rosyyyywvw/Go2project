// cpp_driver_um98x_ntrip.cpp
// Cross-platform NTRIP + GNSS serial client (Ubuntu / Windows)
// Linux 编译:
//   g++ -std=c++11 -O2 -pthread cpp_driver_um98x_ntrip.cpp -o cpp_driver_um98x_ntrip -lboost_system
// Windows编译 (MSYS2/MinGW64):
//   g++ -std=gnu++11 -O2 cpp_driver_um98x_ntrip.cpp -o cpp_driver_um98x_ntrip.exe -lboost_system -lws2_32 -lmswsock
//  Linux 运行方法：
// ./cpp_driver_um98x_ntrip --port /dev/ttyUSB0 --baudrate 115200 \
//   --server 120.253.239.161 --server-port 8002 \
//   --username "$NTRIP_USERNAME" --password "$NTRIP_PASSWORD" \
//   --mount RTCM33_GRCE --print-raw
//  Windows 运行方法：
// ./cpp_driver_um98x_ntrip --port COM3 --baudrate 115200 \
//   --server 120.253.239.161 --server-port 8002 \
//   --username "$NTRIP_USERNAME" --password "$NTRIP_PASSWORD" \
//   --mount RTCM33_GRCE --print-raw

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstdlib>
#include <cctype>
#include <csignal>
#include <map>
#include <limits>
#include <cstring>
#include <cstdio>

#ifndef _WIN32
  #include <unistd.h>   // isatty, fileno
  #include <fcntl.h>    // fcntl, O_NONBLOCK
#endif

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#ifdef _WIN32
#  include <windows.h>
#endif

using boost::asio::ip::tcp;
namespace asio = boost::asio;

// ---------- ANSI (toggle by --no-color) ----------
static bool ANSI_ENABLE =
#if defined(_WIN32)
    false;
#else
    true;
#endif

static const char* BOLD = "\033[1m";
static const char* YEL  = "\033[33m";
static const char* RED  = "\033[31m";
static const char* GRN  = "\033[32m";
static const char* RST  = "\033[0m";

static inline void apply_no_color() {
    ANSI_ENABLE = false;
    BOLD = YEL = RED = GRN = RST = "";
}

static inline void log_info(const std::string& s){
    std::cout << (ANSI_ENABLE?GRN:"") << "[INFO]" << (ANSI_ENABLE?RST:"") << " " << s << std::endl;
}
static inline void log_warn(const std::string& s){
    std::cout << (ANSI_ENABLE?YEL:"") << "[WARN]" << (ANSI_ENABLE?RST:"") << " " << s << std::endl;
}
static inline void log_err(const std::string& s){
    std::cerr << (ANSI_ENABLE?RED:"") << "[ERR ]" << (ANSI_ENABLE?RST:"") << " " << s << std::endl;
}

// ---------- Args ----------
static std::string env_or(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

struct Args {
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    double serial_timeout = 1.0;
    std::string server = env_or("NTRIP_SERVER", "120.253.239.161");
    int server_port = 8002;
    std::string username = env_or("NTRIP_USERNAME", "");
    std::string password = env_or("NTRIP_PASSWORD", "");
    std::string mount = env_or("NTRIP_MOUNTPOINT", "RTCM33_GRCE");
    double gga_period = 3.0;
    bool print_raw = false;
    bool no_color = false;
};

static inline bool starts_with(const std::string& s, const char* pfx){
    return s.rfind(pfx, 0) == 0;
}
static inline bool ieq(const std::string& a, const char* b){
    if (a.size()!=std::strlen(b)) return false;
    for (size_t i=0;i<a.size();++i) if (std::tolower(a[i])!=std::tolower(b[i])) return false;
    return true;
}

static Args parse_args(int argc, char** argv){
    Args a;
    for (int i=1;i<argc;i++){
        std::string k = argv[i];
        auto need = [&](int n){ if (i+n>=argc){ log_err("Missing value for "+k); std::exit(2);} };
        if (k=="--port"){ need(1); a.port = argv[++i]; }
        else if (k=="--baudrate"){ need(1); a.baudrate = std::atoi(argv[++i]); }
        else if (k=="--serial-timeout"){ need(1); a.serial_timeout = std::atof(argv[++i]); }
        else if (k=="--server"){ need(1); a.server = argv[++i]; }
        else if (k=="--server-port"){ need(1); a.server_port = std::atoi(argv[++i]); }
        else if (k=="--username"){ need(1); a.username = argv[++i]; }
        else if (k=="--password"){ need(1); a.password = argv[++i]; }
        else if (k=="--mount"){ need(1); a.mount = argv[++i]; }
        else if (k=="--gga-period"){ need(1); a.gga_period = std::max(0.2, std::atof(argv[++i])); }
        else if (k=="--print-raw"){ a.print_raw = true; }
        else if (k=="--no-color"){ a.no_color = true; }
        else if (k=="-h" || k=="--help"){
            std::cout <<
R"(Simple NTRIP+GNSS client (print GGA/RMC)
Options:
  --port /dev/ttyUSB0 | COM3
  --baudrate 115200
  --serial-timeout 1.0
  --server <host>
  --server-port <port>
  --username <u>
  --password <p>
  --mount <mountpoint>
  --gga-period 3.0
  --print-raw
  --no-color
)" << std::endl;
            std::exit(0);
        } else {
            log_warn("Unknown arg: "+k);
        }
    }
    if (a.no_color) apply_no_color();
#ifdef _WIN32
    if (starts_with(a.port, "COM")) {
        a.port = "\\\\.\\" + a.port; // COM10+ 前缀修正
    }
#endif
    return a;
}

// ---------- Base64 ----------
static const char b64tbl[] =
"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string base64_encode(const std::string& in){
    std::string out;
    size_t i=0;
    while (i<in.size()){
        uint32_t v=0; int n=0;
        for (int k=0;k<3;k++){
            v <<= 8;
            if (i<in.size()){ v |= (unsigned char)in[i++]; n++; }
        }
        for (int k=0;k<4;k++){
            int idx = (v >> (18 - 6*k)) & 0x3F;
            if (k < n + 1) out.push_back(b64tbl[idx]); else out.push_back('=');
        }
        if (n==1) out[out.size()-2]='=', out[out.size()-1]='=';
        else if (n==2) out[out.size()-1]='=';
    }
    return out;
}

// ---------- NMEA helpers ----------
static bool verify_nmea_checksum(const std::string& s){
    auto pos = s.find('*');
    if (pos == std::string::npos) return true; // tolerate no checksum
    unsigned char x=0;
    for (size_t i=1; i<pos; ++i) x ^= (unsigned char)s[i];
    char buf[3]; std::snprintf(buf, sizeof(buf), "%02X", x);
    if (pos+1 >= s.size()) return false;
    std::string got = s.substr(pos+1, 2);
    for (auto& c: got) c = std::toupper(c);
    return got == buf;
}

static double to_float_def(const std::string& s, double dflt){
    try{ if (s.empty()) return dflt; return std::stod(s); }
    catch(...){return dflt;}
}

static double dms_to_decimal(const std::string& dms, char dir, bool is_lat){
    try{
        if (dms.find('.')==std::string::npos) return std::numeric_limits<double>::quiet_NaN();
        int degw = is_lat ? 2 : 3;
        double deg = std::stod(dms.substr(0, degw));
        double min = std::stod(dms.substr(degw));
        double dec = deg + min/60.0;
        if (dir=='S' || dir=='W') dec = -dec;
        return dec;
    }catch(...){
        return std::numeric_limits<double>::quiet_NaN();
    }
}

struct GGAInfo{ int fix_q=0; int num_sats=0; double hdop=99.9; double lat=0, lon=0, alt=0; bool valid=false; };
struct RMCInfo{ bool status_valid=false; double speed_kn=0, speed_mps=0, cog_deg=0; bool valid=false; };
struct THSInfo{ double heading_deg=0; bool valid=false; };

static std::string fix_desc(int f){
    static std::map<int,std::string> m = {
        {0,"Invalid Fix"},{1,"GPS Fix (SPS)"},{2,"DGPS Fix"},{3,"PPS Fix"},
        {4,"RTK Fixed"},{5,"RTK Float"},{6,"Estimated"},{7,"Manual"},{8,"Simulation"}
    };
    auto it=m.find(f); return it==m.end()?"Unknown":it->second;
}

static bool parse_gga(const std::string& line, GGAInfo& out){
    if (!(starts_with(line,"$GNGGA") || starts_with(line,"$GPGGA"))) return false;
    std::cout << line << std::endl;
    if (!verify_nmea_checksum(line)) return false;
    std::vector<std::string> p; p.reserve(20);
    size_t b=0; while (true){ auto e=line.find(',',b); if (e==std::string::npos){p.push_back(line.substr(b));break;} p.push_back(line.substr(b,e-b)); b=e+1; }
    if (p.size()<10) return false;
    std::string lat_s = p[2], lat_dir = p[3];
    std::string lon_s = p[4], lon_dir = p[5];
    out.fix_q = std::atoi(p[6].c_str());
    out.num_sats = std::atoi(p[7].c_str());
    out.hdop = to_float_def(p[8], 99.9);
    out.alt = to_float_def(p[9], 0.0);
    out.lat = dms_to_decimal(lat_s, lat_dir.empty()? 'N':lat_dir[0], true);
    out.lon = dms_to_decimal(lon_s, lon_dir.empty()? 'E':lon_dir[0], false);
    out.valid = true;
    return true;
}

static bool parse_rmc(const std::string& line, RMCInfo& out){
    if (!(starts_with(line,"$GNRMC") || starts_with(line,"$GPRMC"))) return false;
    std::cout << line << std::endl;
    if (!verify_nmea_checksum(line)) return false;
    std::vector<std::string> p; p.reserve(20);
    size_t b=0; while (true){ auto e=line.find(',',b); if (e==std::string::npos){p.push_back(line.substr(b));break;} p.push_back(line.substr(b,e-b)); b=e+1; }
    if (p.size()<12) return false;
    char status = p[2].empty()? 'V':p[2][0];
    out.status_valid = (status=='A');
    out.speed_kn = to_float_def(p[7], 0.0);
    out.cog_deg  = to_float_def(p[8], 0.0);
    out.speed_mps = out.speed_kn * 0.514444;
    out.valid = true;
    return true;
}

static bool parse_ths(const std::string& line, THSInfo& out){
    // 常见格式: $GNTHS,230.1081,A*10  或 $GPTHS,...
    if (!(starts_with(line,"$GNTHS") || starts_with(line,"$GPTHS"))) return false;
    std::cout << line << std::endl;
    if (!verify_nmea_checksum(line)) return false;
    std::vector<std::string> p; p.reserve(8);
    size_t b=0;
    while (true){
        auto e=line.find(',',b);
        if (e==std::string::npos){ p.push_back(line.substr(b)); break; }
        p.push_back(line.substr(b,e-b)); b=e+1;
    }
    if (p.size()<3) return false;
    try { out.heading_deg = p[1].empty()? 0.0 : std::stod(p[1]); } catch(...){ out.heading_deg = 0.0; }
    // 第三段形如 "A*hh" 或 "V*hh"
    out.valid = !p[2].empty() && (p[2][0] == 'A');
    return true;
}

// --------- 全局停止标志：先前置声明，再定义 ----------
extern std::atomic<bool> g_stop;  // 前置声明

// ---------- App ----------
class App {
public:
    explicit App(const Args& a) : args_(a), io_(), serial_(io_), sock_(io_) {}

    void run(){
        open_serial_blocking();
        wait_first_gga();
        connect_ntrip();

        th_gga_  = std::thread([this]{ loop_send_gga(); });
        th_rtcm_ = std::thread([this]{ loop_recv_rtcm(); });

        // main loop: read serial -> parse & print
        read_serial_loop();

        g_stop = true;
        close_all();
        if (th_gga_.joinable()) th_gga_.join();
        if (th_rtcm_.joinable()) th_rtcm_.join();
    }

private:
    // 把串口设成“非阻塞/短超时”
    void make_serial_nonblocking() {
#ifdef _WIN32
        HANDLE h = reinterpret_cast<HANDLE>(serial_.native_handle());
        COMMTIMEOUTS to{};
        // 让 ReadFile 快速返回：无字符时 1ms 返回
        to.ReadIntervalTimeout         = 1;
        to.ReadTotalTimeoutMultiplier  = 0;
        to.ReadTotalTimeoutConstant    = 1;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant   = 0;
        SetCommTimeouts(h, &to);
#else
        int fd = serial_.native_handle();
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
    }

    void open_serial(){
        boost::system::error_code ec;
        serial_.open(args_.port, ec);
        if (ec) throw std::runtime_error("open serial failed: " + ec.message());

        serial_.set_option(boost::asio::serial_port_base::baud_rate(args_.baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        make_serial_nonblocking();

        log_info("Opened serial " + args_.port + " @ " + std::to_string(args_.baudrate));
    }

    void open_serial_blocking(){
        double backoff = 0.5;                 // 退避：0.5s 起，×1.5，封顶 5s
        for (;;){
            if (g_stop) throw std::runtime_error("stopped before serial opened");

            boost::system::error_code ec;
            serial_.open(args_.port, ec);
            if (!ec) {
                try {
                    serial_.set_option(boost::asio::serial_port_base::baud_rate(args_.baudrate));
                    serial_.set_option(boost::asio::serial_port_base::character_size(8));
                    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                    make_serial_nonblocking();
                    log_info("Opened serial " + args_.port + " @ " + std::to_string(args_.baudrate) + " (initial blocking open)");
                    return;
                } catch (...) {
                    // 配置失败就关掉，走重试
                    boost::system::error_code ec2; serial_.close(ec2);
                }
            }

            // 打不开：打印并重试（次数不封顶）
            log_warn(std::string("Open serial " + args_.port + "@" + std::to_string(args_.baudrate) + " failed (will retry): ") + ec.message());
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(backoff * 1000)));
            backoff = std::min(5.0, backoff * 1.5);
        }
    }

    void reopen_serial_with_backoff(){
        double backoff = 0.5;
        boost::system::error_code ec;
        try{ if (serial_.is_open()) serial_.close(ec); }catch(...){}
        while (!g_stop){
            try{
                open_serial();
                return;
            }catch(const std::exception& e){
                log_warn(std::string("Reopen serial failed: ")+e.what());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(backoff*1000)));
            backoff = std::min(5.0, backoff*1.5);
        }
    }

    void wait_first_gga(){
        std::string line;
        for (;;){
            if (g_stop) return;
            if (!readline_serial(line)){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            handle_nmea_line(line, /*print*/false);
            if (have_gga_){
                log_info("First GGA received, will connect NTRIP.");
                return;
            }
        }
    }

    void connect_ntrip() {
        boost::system::error_code ec;
        tcp::resolver resolver(io_);
        tcp::resolver::query query(args_.server, std::to_string(args_.server_port));
        auto it = resolver.resolve(query, ec);
        if (ec) { log_warn("Resolve failed: " + ec.message()); return; }

        asio::connect(sock_, it, ec);
        if (ec) {
            log_warn("Connect NTRIP failed: "+ec.message());
            safe_close_sock();
            return;
        }
        std::string auth = base64_encode(args_.username + ":" + args_.password);
        std::string req =
            "GET /" + args_.mount + " HTTP/1.1\r\n"
            "Host: " + args_.server + "\r\n"
            "User-Agent: NTRIP GNSS/2.0\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "Accept: */*\r\n"
            "Connection: keep-alive\r\n"
            "Authorization: Basic " + auth + "\r\n"
            "\r\n";
        asio::write(sock_, boost::asio::buffer(req), ec);
        if (ec) {
            log_warn("NTRIP send request failed: "+ec.message());
            safe_close_sock();
            return;
        }
        log_info(std::string("NTRIP socket connected, waiting for data... ") +
                 (ANSI_ENABLE?std::string(YEL)+BOLD:"") +
                 "(If this sentence appears multiple times, it may indicate a CORS account issue)" + (ANSI_ENABLE?RST:""));
    }

    void try_reconnect_ntrip(double delay_sec = 1.0){
        auto t0 = std::chrono::steady_clock::now();
        while (!g_stop){
            connect_ntrip();
            if (sock_.is_open()) return;
            auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
            if (dt > delay_sec) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    void safe_close_sock(){
        boost::system::error_code ec;
        if (sock_.is_open()){
            sock_.shutdown(tcp::socket::shutdown_both, ec);
            sock_.close(ec);
        }
    }

    void loop_send_gga(){
        using clk = std::chrono::steady_clock;
        auto next_t = clk::now();
        while (!g_stop){
            auto now = clk::now();
            if (now < next_t){
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            next_t = now + std::chrono::milliseconds((int)(args_.gga_period*1000));

            std::string gga;
            {
                std::lock_guard<std::mutex> lk(mu_);
                gga = latest_gga_;
            }
            if (gga.empty() || !sock_.is_open()) continue;

            std::string payload = gga + "\r\n";
            boost::system::error_code ec;
            asio::write(sock_, boost::asio::buffer(payload), ec);
            if (ec){
                log_warn("NTRIP send failed: " + ec.message());
                safe_close_sock();
                try_reconnect_ntrip(1.0);
            }
        }
    }

    void loop_recv_rtcm(){
        std::vector<uint8_t> buf(4096);
        while (!g_stop){
            if (!sock_.is_open() || !serial_.is_open()){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            boost::system::error_code ec;
            size_t n = sock_.read_some(boost::asio::buffer(buf), ec);
            if (ec == boost::asio::error::eof){
                safe_close_sock();
                try_reconnect_ntrip(1.0);
                continue;
            } else if (ec){
                continue; // transient
            }
            if (n==0) continue;

            if (n>=10){
                std::string s((const char*)buf.data(), std::min<size_t>(n, 32));
                if (s.find("ICY 200 OK")!=std::string::npos){
                    log_info("NTRIP: ICY 200 OK");
                    continue;
                }
            }
            boost::system::error_code sec;
            asio::write(serial_, boost::asio::buffer(buf.data(), n), sec);
            if (sec){
                log_warn("Write RTCM to serial failed: "+sec.message());
            }
        }
    }

    bool readline_serial(std::string& out){
        char c;
        boost::system::error_code ec;
        for (;;){
            if (g_stop) return false;

            size_t n = serial_.read_some(boost::asio::buffer(&c,1), ec);

            if (ec) {
                if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }
                log_warn("Serial read error: " + ec.message() + ", try reopen...");
                reopen_serial_with_backoff();
                return false;
            }

            if (n == 1) {
                if (c == '\r' || c == '\n') {
                    if (!linebuf_.empty()) {
                        out.swap(linebuf_);
                        linebuf_.clear();
                        return true;
                    } else {
                        // 连续的换行符，继续读
                        continue;
                    }
                }
                linebuf_.push_back(c);
                if (linebuf_.size() > 4096) linebuf_.clear();
            }
        }
    }

    void read_serial_loop(){
        std::string line;
        while (!g_stop){
            if (!readline_serial(line)){
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            handle_nmea_line(line, /*print*/true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void handle_nmea_line(const std::string& line, bool print){
        if (line.empty() || line[0]!='$') return;

        if (starts_with(line,"$GPGGA") || starts_with(line,"$GNGGA")){
            if (verify_nmea_checksum(line)){
                std::lock_guard<std::mutex> lk(mu_);
                latest_gga_ = line;
                have_gga_ = true;
            }

            if (!print) return;
            // if (args_.print_raw) {
            //     std::cout << line << std::endl;
            // }
        }

        GGAInfo gga;
        if (parse_gga(line, gga) && gga.valid){
            std::cout << "[GGA] fix=" << gga.fix_q << " (" << fix_desc(gga.fix_q) << ")"
                      << ", lat=" << std::fixed << std::setprecision(8) << gga.lat
                      << ", lon=" << std::fixed << std::setprecision(8) << gga.lon
                      << ", alt=" << std::fixed << std::setprecision(3) << gga.alt << " m"
                      << ", hdop=" << std::fixed << std::setprecision(2) << gga.hdop
                      << std::endl;
            return;
        }
        RMCInfo rmc;
        if (parse_rmc(line, rmc) && rmc.valid){
            std::cout << "[RMC] valid=" << (rmc.status_valid? "effective":"invalid")
                      << ", speed=" << std::fixed << std::setprecision(3) << rmc.speed_mps << " m/s"
                      << " (" << rmc.speed_kn << " kn)"
                      << ", COG=" << std::fixed << std::setprecision(2) << rmc.cog_deg << " deg"
                      << std::endl;
            return;
        }

        THSInfo ths;
        if (parse_ths(line, ths)){
            std::cout << "[THS] heading=" << std::fixed << std::setprecision(3) << ths.heading_deg
                    << " deg (" << (ths.valid? "valid":"invalid") << ")"
                    << std::endl;
            return;
        }
    }

    void close_all(){
        safe_close_sock();
        boost::system::error_code ec;
        if (serial_.is_open()) serial_.close(ec);
    }

private:
    Args args_;
    asio::io_service io_;
    asio::serial_port serial_;
    tcp::socket sock_;

    std::thread th_gga_, th_rtcm_;
    std::mutex mu_;
    std::string latest_gga_;
    bool have_gga_ = false;
    std::string linebuf_;
};

// --------- 全局停止标志定义 ----------
std::atomic<bool> g_stop(false);
static void on_sigint(int){ g_stop=true; }

int main(int argc, char** argv){
    std::signal(SIGINT, on_sigint);
#ifndef _WIN32
    if (!isatty(fileno(stdout))) apply_no_color();
#endif
    Args a = parse_args(argc, argv);
    try{
        App app(a);
        app.run();
    }catch(const std::exception& e){
        log_err(std::string("Fatal: ")+e.what());
        return 1;
    }
    return 0;
}
