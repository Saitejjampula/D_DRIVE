/**
 * SKID STEERING — CAN MOTOR CONTROLLER
 * Vehicle: 220kg | 4 Motors | Track=875mm | Wheel=245mm | Gear=2.86 | MaxRPM=1000
 *
 * Build:
 *   Windows: g++ -std=c++17 -O2 skid_steer_final.cpp -o skid_steer.exe
 *   Linux  : g++ -std=c++17 -O2 -pthread skid_steer_final.cpp -o skid_steer
 *
 * Keyboard Controls (during run):
 *   W / S  = Forward / Reverse
 *   A / D  = Turn Left / Turn Right
 *   X      = Spin in place left
 *   C      = Spin in place right
 *   SPACE  = Emergency Stop
 *   Q      = Quit
 */

#ifdef _WIN32
  #define PLATFORM_WINDOWS
  #include <windows.h>
  #include <conio.h>
#else
  #define PLATFORM_LINUX
  #include <fcntl.h>
  #include <termios.h>
  #include <unistd.h>
  #include <signal.h>
#endif

#include <cstdint>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <set>
#include <algorithm>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────────────────────────────
//  CAN / SERIAL CONFIGURATION
// ─────────────────────────────────────────────────────────────
#ifdef PLATFORM_WINDOWS
  constexpr const char* TTY_DEVICE = "\\\\.\\COM3";
#else
  constexpr const char* TTY_DEVICE = "/dev/ttyUSB0";
#endif

constexpr int      USB_BAUD          = 2000000;
constexpr int      CAN_SPEED         = 250000;
constexpr uint32_t CMD_CAN_ID        = 0x18530905;
constexpr double   SEND_INTERVAL     = 0.1;       // seconds
constexpr double   INTER_FRAME_DELAY = 0.001;     // seconds

// Motor IDs
//  LEFT  side: D1 (front-left),  D2 (rear-left)
//  RIGHT side: D3 (front-right), D4 (rear-right)
const std::vector<uint8_t> LEFT_MOTORS  = { 0xD1, 0xD2 };
const std::vector<uint8_t> RIGHT_MOTORS = { 0xD3, 0xD4 };

// ─────────────────────────────────────────────────────────────
//  VEHICLE PARAMETERS  ← YOUR REAL VALUES
// ─────────────────────────────────────────────────────────────
constexpr double TRACK_WIDTH   = 0.875;    // m  (875 mm)
constexpr double WHEELBASE     = 1.050;    // m  (1050 mm)
constexpr double WHEEL_RADIUS  = 0.245;    // m  (245 mm)
constexpr double GEAR_RATIO    = 2.86;     // motor RPM / wheel RPM
constexpr double VEHICLE_MASS  = 220.0;    // kg
constexpr double MOTOR_PEAK_NM = 25.0;     // Nm per motor
constexpr int    MAX_RPM       = 1000;     // absolute motor max
constexpr int    SAFE_MAX_RPM  = 100;      // ~3 m/s → recommended limit
constexpr int    MIN_RPM       = 30;       // below this = stall

// Derived constants
constexpr double WHEEL_CIRC   = 2.0 * M_PI * WHEEL_RADIUS;  // 1.5394 m
constexpr double RPM_TO_MS    = WHEEL_CIRC / (60.0 * GEAR_RATIO); // 0.00897 m/s per RPM
constexpr double MS_TO_RPM    = 1.0 / RPM_TO_MS;                   // RPM per m/s

// Speed presets
constexpr double SPEED_CREEP   = 0.45;   // m/s  (50  RPM)
constexpr double SPEED_SLOW    = 0.90;   // m/s  (100 RPM)
constexpr double SPEED_NORMAL  = 1.79;   // m/s  (200 RPM)
constexpr double SPEED_FAST    = 2.69;   // m/s  (300 RPM)  ← recommended max

constexpr double TURN_GENTLE   = 0.3;    // rad/s
constexpr double TURN_NORMAL   = 0.6;    // rad/s
constexpr double TURN_SHARP    = 0.9;    // rad/s
constexpr double SPIN_SLOW     = 0.5;    // rad/s  (safe on concrete)

// ─────────────────────────────────────────────────────────────
//  SERIAL PORT
// ─────────────────────────────────────────────────────────────
class SerialPort {
public:
#ifdef PLATFORM_WINDOWS
    HANDLE hPort = INVALID_HANDLE_VALUE;

    bool open(const char* dev, int baud) {
        hPort = CreateFileA(dev, GENERIC_READ | GENERIC_WRITE, 0,
                            nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (hPort == INVALID_HANDLE_VALUE) return false;
        DCB dcb = {}; dcb.DCBlength = sizeof(dcb);
        GetCommState(hPort, &dcb);
        dcb.BaudRate = baud; dcb.ByteSize = 8;
        dcb.Parity = NOPARITY; dcb.StopBits = TWOSTOPBITS;
        SetCommState(hPort, &dcb);
        COMMTIMEOUTS ct = { 10, 0, 10, 0, 0 };
        SetCommTimeouts(hPort, &ct);
        return true;
    }
    void write_bytes(const uint8_t* data, size_t len) {
        DWORD w; WriteFile(hPort, data, (DWORD)len, &w, nullptr);
    }
    size_t read_bytes(uint8_t* buf, size_t max_len) {
        DWORD r = 0; ReadFile(hPort, buf, (DWORD)max_len, &r, nullptr); return r;
    }
    void close() { if (hPort != INVALID_HANDLE_VALUE) CloseHandle(hPort); }
#else
    int fd = -1;
    bool open(const char* dev, int baud) {
        fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;
        struct termios tty = {};
        tcgetattr(fd, &tty);
        cfsetispeed(&tty, B2000000); cfsetospeed(&tty, B2000000);
        tty.c_cflag &= ~PARENB; tty.c_cflag |= CSTOPB;
        tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8 | CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
        tty.c_iflag &= ~(IXON|IXOFF|IXANY|ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty); return true;
    }
    void write_bytes(const uint8_t* data, size_t len) { ::write(fd, data, len); }
    size_t read_bytes(uint8_t* buf, size_t max_len) {
        ssize_t n = ::read(fd, buf, max_len); return (n < 0) ? 0 : (size_t)n;
    }
    void close() { if (fd >= 0) ::close(fd); }
#endif
};

// ─────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline double elapsed(const TimePoint& start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
inline void sleep_sec(double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

// ─────────────────────────────────────────────────────────────
//  CSV LOGGER
// ─────────────────────────────────────────────────────────────
class CSVLogger {
    std::mutex   mtx;
    std::ofstream tx_f, rpm_f, ac_f, fault_f, drive_f;

    static std::string na(double v, bool valid) {
        if (!valid) return "N/A";
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << v;
        return ss.str();
    }

public:
    CSVLogger() {
        tx_f.open("can_tx.csv");
        rpm_f.open("can_rpm_batt.csv");
        ac_f.open("can_ac_temp.csv");
        fault_f.open("can_faults.csv");
        drive_f.open("drive_commands.csv");

        tx_f    << "Timestamp (s),Motor ID,CAN ID,Target RPM,Raw Frame (Hex)\n";
        rpm_f   << "Timestamp (s),Motor ID,RPM,Throttle (%),Batt V (V),Batt I (A)\n";
        ac_f    << "Timestamp (s),Motor ID,AC Volt (V),AC Curr (A),MCU Temp (C),Motor Temp (C),Ready\n";
        fault_f << "Timestamp (s),Motor ID,Drive Status,Faults,Raw Hex\n";
        drive_f << "Timestamp (s),Linear Vel (m/s),Angular Vel (rad/s),"
                   "Left RPM,Right RPM,Speed (km/h),Torque State\n";
    }

    void log_tx(double ts, uint8_t mid, int rpm, const std::string& raw) {
        std::lock_guard<std::mutex> lk(mtx);
        tx_f << std::fixed << std::setprecision(3) << ts
             << ",0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)mid
             << ",0x" << std::setw(8) << CMD_CAN_ID
             << std::dec << "," << rpm << "," << raw << "\n";
    }

    void log_rpm_batt(double ts, uint8_t mid, int rpm, int throttle,
                      double bv, bool bv_ok, double bi, bool bi_ok) {
        std::lock_guard<std::mutex> lk(mtx);
        rpm_f << std::fixed << std::setprecision(3) << ts
              << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
              << std::dec << "," << rpm << "," << throttle
              << "," << na(bv, bv_ok) << "," << na(bi, bi_ok) << "\n";
    }

    void log_ac_temp(double ts, uint8_t mid,
                     double av, bool av_ok, double ai, bool ai_ok,
                     int mcu, int mot, const std::string& ready) {
        std::lock_guard<std::mutex> lk(mtx);
        ac_f << std::fixed << std::setprecision(3) << ts
             << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
             << std::dec
             << "," << na(av, av_ok) << "," << na(ai, ai_ok)
             << "," << mcu << "," << mot << "," << ready << "\n";
    }

    void log_faults(double ts, uint8_t mid,
                    const std::string& status, const std::string& faults,
                    const std::string& raw) {
        std::lock_guard<std::mutex> lk(mtx);
        fault_f << std::fixed << std::setprecision(3) << ts
                << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
                << std::dec
                << ",\"" << status << "\",\"" << faults << "\"," << raw << "\n";
    }

    void log_drive(double ts, double lin, double ang,
                   int l_rpm, int r_rpm, double speed_kmh,
                   const std::string& state) {
        std::lock_guard<std::mutex> lk(mtx);
        drive_f << std::fixed << std::setprecision(3) << ts
                << "," << lin << "," << ang
                << "," << l_rpm << "," << r_rpm
                << "," << std::setprecision(1) << speed_kmh
                << "," << state << "\n";
    }

    void save() {
        std::lock_guard<std::mutex> lk(mtx);
        tx_f.flush(); rpm_f.flush(); ac_f.flush();
        fault_f.flush(); drive_f.flush();
        std::cout << "[SAVED] All CSV logs written.\n";
    }
};

// ─────────────────────────────────────────────────────────────
//  TX FRAME BUILDER
// ─────────────────────────────────────────────────────────────
std::vector<uint8_t> build_tx_frame(uint8_t motor_id, int rpm) {
    int16_t rpm16 = (int16_t)std::max(-32768, std::min(32767, rpm));
    std::vector<uint8_t> frame(15, 0);
    frame[0]  = 0xAA;
    frame[1]  = 0xE8;
    frame[2]  = CMD_CAN_ID & 0xFF;
    frame[3]  = (CMD_CAN_ID >> 8)  & 0xFF;
    frame[4]  = (CMD_CAN_ID >> 16) & 0xFF;
    frame[5]  = (CMD_CAN_ID >> 24) & 0xFF;
    frame[6]  = motor_id;
    frame[7]  = (uint8_t)(rpm16 & 0xFF);
    frame[8]  = (uint8_t)((rpm16 >> 8) & 0xFF);
    frame[14] = 0x55;
    return frame;
}

// ─────────────────────────────────────────────────────────────
//  RX FRAME PARSER
// ─────────────────────────────────────────────────────────────
constexpr size_t FRAME_LEN = 20;

struct ParsedFrame {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  payload[8];
    bool     valid = false;
};

ParsedFrame find_and_parse_frame(std::vector<uint8_t>& buf) {
    ParsedFrame pf;
    while (buf.size() >= FRAME_LEN) {
        size_t idx = SIZE_MAX;
        for (size_t i = 0; i + 3 < buf.size(); ++i) {
            if (buf[i]==0xAA && buf[i+1]==0x55 &&
                buf[i+2]==0x01 && buf[i+3]==0x02) { idx = i; break; }
        }
        if (idx == SIZE_MAX) {
            if (buf.size() > 3) buf.erase(buf.begin(), buf.end() - 3);
            return pf;
        }
        if (idx != 0) buf.erase(buf.begin(), buf.begin() + idx);
        if (buf.size() < FRAME_LEN) break;
        pf.can_id = buf[5] | (buf[6]<<8) | (buf[7]<<16) | (buf[8]<<24);
        pf.dlc    = std::min((int)buf[9], 8);
        memcpy(pf.payload, &buf[10], pf.dlc);
        pf.valid  = true;
        buf.erase(buf.begin(), buf.begin() + FRAME_LEN);
        return pf;
    }
    return pf;
}

// ─────────────────────────────────────────────────────────────
//  SIGNAL DECODERS
// ─────────────────────────────────────────────────────────────
struct RpmBattData {
    uint8_t motor_id; uint16_t rpm; uint8_t throttle;
    double batt_v; bool bv_ok;
    double batt_i; bool bi_ok;
};
RpmBattData decode_0x14520902(const uint8_t* d) {
    RpmBattData r = {};
    r.motor_id = d[0];
    r.rpm      = d[1] | (d[2] << 8);
    r.throttle = d[3];
    uint16_t bv = d[4] | (d[5] << 8);
    int16_t  bi = (int16_t)(d[6] | (d[7] << 8));
    r.batt_v = bv * 0.1; r.bv_ok = true;
    r.bi_ok  = (bi != -32768);
    if (r.bi_ok) r.batt_i = bi * 0.1;
    return r;
}

struct AcTempData {
    uint8_t motor_id;
    double ac_v; bool av_ok;
    double ac_i; bool ai_ok;
    int mcu_temp, mot_temp;
    std::string ready;
};
AcTempData decode_0x18530904(const uint8_t* d) {
    AcTempData r = {};
    r.motor_id = d[0];
    uint16_t av = d[1] | (d[2] << 8);
    int16_t  ai = (int16_t)(d[3] | (d[4] << 8));
    r.av_ok = (av != 0x8000); if (r.av_ok) r.ac_v = av * 0.1;
    r.ai_ok = (ai != -32768); if (r.ai_ok) r.ac_i = ai * 0.1;
    r.mcu_temp = d[5] - 40;
    r.mot_temp = d[6] - 40;
    r.ready    = (d[7] == 1) ? "Yes" : "No";
    return r;
}

const char* DRIVE_STATUS1[] = {"Low","Medium","High","Neutral","Forward","Reverse","Brake","Parking"};
const char* DRIVE_STATUS2[] = {"Regen Brake","Charging","Hill Assist","Limp Home","-","-","-","-"};
const char* FAULT1[] = {"Over Voltage","Under Voltage","Motor OverTemp","MCU OverTemp",
                        "Throttle Error","Motor Stalling","Motor Locked","Position Sensor Fault"};
const char* FAULT2[] = {"MCU Fault1","Encoder Fault","MCU Fault2","Brake Fault",
                        "Motor Start Fault","Motor Overload","-","-"};
const char* FAULT3[] = {"MCU Fault3","MCU Fault4","MCU Fault5","MCU Fault6",
                        "MCU Fault7","MCU Fault8","MCU Fault9","MCU Fault10"};

std::string active_bits(uint8_t val, const char** names, bool skip_dash = false) {
    std::string out;
    for (int i = 0; i < 8; ++i)
        if (val & (1 << i)) {
            if (skip_dash && std::string(names[i]) == "-") continue;
            if (!out.empty()) out += ", ";
            out += names[i];
        }
    return out;
}

struct FaultData { uint8_t motor_id; std::string status, faults; };
FaultData decode_0x1C530902(const uint8_t* d) {
    FaultData r = {};
    r.motor_id = d[0];
    std::string s1 = active_bits(d[1], DRIVE_STATUS1);
    std::string s2 = active_bits(d[2], DRIVE_STATUS2);
    r.status = (s1.empty() && s2.empty()) ? "None"
               : s1 + ((!s1.empty() && !s2.empty()) ? ", " : "") + s2;
    std::string f1 = active_bits(d[3], FAULT1, true);
    std::string f2 = active_bits(d[4], FAULT2, true);
    std::string f3 = active_bits(d[5], FAULT3, true);
    std::vector<std::string> fv;
    if (!f1.empty()) fv.push_back(f1);
    if (!f2.empty()) fv.push_back(f2);
    if (!f3.empty()) fv.push_back(f3);
    r.faults = fv.empty() ? "None" : "";
    for (size_t i = 0; i < fv.size(); ++i) { if (i) r.faults += ", "; r.faults += fv[i]; }
    return r;
}

// ─────────────────────────────────────────────────────────────
//  SKID STEERING CORE
// ─────────────────────────────────────────────────────────────
struct WheelRPMs { int left; int right; };

enum class TorqueState { SAFE, WARNING, OVERLOAD };

TorqueState check_torque(double lin, double ang) {
    double spin_torque = VEHICLE_MASS * 9.81 * 0.3 * (TRACK_WIDTH / 2.0)
                         * std::abs(ang) / 4.0 / GEAR_RATIO;
    double roll_torque = VEHICLE_MASS * 9.81 * 0.05 * WHEEL_RADIUS / 4.0 / GEAR_RATIO;
    double total = spin_torque + roll_torque;
    if (total > MOTOR_PEAK_NM)        return TorqueState::OVERLOAD;
    if (total > MOTOR_PEAK_NM * 0.80) return TorqueState::WARNING;
    return TorqueState::SAFE;
}

WheelRPMs skid_steer(double lin, double ang) {
    double factor    = 60.0 / (WHEEL_CIRC / GEAR_RATIO);
    double rpm_left  = (lin - ang * TRACK_WIDTH / 2.0) * factor;
    double rpm_right = (lin + ang * TRACK_WIDTH / 2.0) * factor;

    auto clamp = [](double r) -> int {
        double mag = std::abs(r);
        if (mag < MIN_RPM) return 0;
        mag = std::min(mag, (double)SAFE_MAX_RPM);
        return (int)(r >= 0 ? mag : -mag);
    };
    return { clamp(rpm_left), clamp(rpm_right) };
}

// ─────────────────────────────────────────────────────────────
//  SEND COMMAND TO ALL 4 MOTORS
// ─────────────────────────────────────────────────────────────
std::atomic<bool> stop_flag{false};

void send_drive(SerialPort& port, CSVLogger& logger,
                double lin, double ang, const TimePoint& start)
{
    TorqueState ts_state = check_torque(lin, ang);
    std::string state_str;

    if (ts_state == TorqueState::OVERLOAD) {
        std::cout << "[SAFETY] ❌ OVERLOAD — command blocked! Reduce turn rate.\n";
        // Send stop instead
        lin = 0.0; ang = 0.0;
        state_str = "OVERLOAD";
    } else if (ts_state == TorqueState::WARNING) {
        std::cout << "[SAFETY] ⚠️  Near torque limit!\n";
        state_str = "WARNING";
    } else {
        state_str = "SAFE";
    }

    WheelRPMs rpms = skid_steer(lin, ang);
    double speed_kmh = std::abs((rpms.left + rpms.right) / 2.0) * RPM_TO_MS * 3.6;
    double ts = elapsed(start);

    // Log drive command
    logger.log_drive(ts, lin, ang, rpms.left, rpms.right, speed_kmh, state_str);

    // Send to LEFT motors (D1, D2)
    for (uint8_t mid : LEFT_MOTORS) {
        auto frame = build_tx_frame(mid, rpms.left);
        port.write_bytes(frame.data(), frame.size());
        std::ostringstream hex_ss;
        for (size_t i = 0; i < frame.size(); ++i) {
            if (i) hex_ss << ' ';
            hex_ss << std::uppercase << std::hex
                   << std::setw(2) << std::setfill('0') << (int)frame[i];
        }
        logger.log_tx(ts, mid, rpms.left, hex_ss.str());
        sleep_sec(INTER_FRAME_DELAY);
    }

    // Send to RIGHT motors (D3, D4)
    // NOTE: If right-side motors are mirror-mounted, change rpms.right → -rpms.right
    for (uint8_t mid : RIGHT_MOTORS) {
        auto frame = build_tx_frame(mid, rpms.right);
        port.write_bytes(frame.data(), frame.size());
        std::ostringstream hex_ss;
        for (size_t i = 0; i < frame.size(); ++i) {
            if (i) hex_ss << ' ';
            hex_ss << std::uppercase << std::hex
                   << std::setw(2) << std::setfill('0') << (int)frame[i];
        }
        logger.log_tx(ts, mid, rpms.right, hex_ss.str());
        sleep_sec(INTER_FRAME_DELAY);
    }
}

// ─────────────────────────────────────────────────────────────
//  KEYBOARD INPUT THREAD
// ─────────────────────────────────────────────────────────────
struct DriveCommand {
    double linear  = 0.0;
    double angular = 0.0;
};

std::atomic<double> g_linear{0.0};
std::atomic<double> g_angular{0.0};

void print_controls() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║       SKID STEER KEYBOARD CONTROL    ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║   W = Forward      S = Reverse       ║\n";
    std::cout << "║   A = Turn Left    D = Turn Right    ║\n";
    std::cout << "║   X = Spin Left    C = Spin Right    ║\n";
    std::cout << "║  [SPACE] = STOP   Q = Quit & Save   ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║  Speed: " << std::setw(5) << SPEED_FAST*3.6 << " km/h max (safe mode) ║\n";
    std::cout << "╚══════════════════════════════════════╝\n\n";
}

void keyboard_thread_fn() {
#ifdef PLATFORM_WINDOWS
    while (!stop_flag.load()) {
        if (_kbhit()) {
            char c = (char)_getch();
            switch (tolower(c)) {
                case 'w': g_linear.store(SPEED_NORMAL);  g_angular.store(0.0);        break;
                case 's': g_linear.store(-SPEED_NORMAL); g_angular.store(0.0);        break;
                case 'a': g_linear.store(SPEED_SLOW);    g_angular.store(TURN_NORMAL);break;
                case 'd': g_linear.store(SPEED_SLOW);    g_angular.store(-TURN_NORMAL);break;
                case 'x': g_linear.store(0.0);           g_angular.store(SPIN_SLOW);  break;
                case 'c': g_linear.store(0.0);           g_angular.store(-SPIN_SLOW); break;
                case ' ': g_linear.store(0.0);           g_angular.store(0.0);
                          std::cout << "[STOP] Emergency stop!\n"; break;
                case 'q': stop_flag.store(true); break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
#else
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (!stop_flag.load()) {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch (tolower(c)) {
                case 'w': g_linear.store(SPEED_NORMAL);  g_angular.store(0.0);        break;
                case 's': g_linear.store(-SPEED_NORMAL); g_angular.store(0.0);        break;
                case 'a': g_linear.store(SPEED_SLOW);    g_angular.store(TURN_NORMAL);break;
                case 'd': g_linear.store(SPEED_SLOW);    g_angular.store(-TURN_NORMAL);break;
                case 'x': g_linear.store(0.0);           g_angular.store(SPIN_SLOW);  break;
                case 'c': g_linear.store(0.0);           g_angular.store(-SPIN_SLOW); break;
                case ' ': g_linear.store(0.0);           g_angular.store(0.0);
                          std::cout << "[STOP] Emergency stop!\n"; break;
                case 'q': stop_flag.store(true); break;
            }
        }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
#endif
}

// ─────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────
int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "  SKID STEER CAN CONTROLLER\n";
    std::cout << "  Track=" << TRACK_WIDTH*1000 << "mm | Wheel=" << WHEEL_RADIUS*1000
              << "mm | Gear=" << GEAR_RATIO << " | MaxRPM=" << MAX_RPM << "\n";
    std::cout << "  Max Speed=" << MAX_RPM * RPM_TO_MS * 3.6 << " km/h"
              << " | Safe Speed=" << SAFE_MAX_RPM * RPM_TO_MS * 3.6 << " km/h\n";
    std::cout << std::string(70, '=') << "\n";

    SerialPort port;
    if (!port.open(TTY_DEVICE, USB_BAUD)) {
        std::cerr << "[ERROR] Cannot open " << TTY_DEVICE << "\n";
        return 1;
    }

    // Set CAN 250k
    const uint8_t can_cmd[] = { 'S', '5', '\r' };
    port.write_bytes(can_cmd, 3);
    sleep_sec(0.1);
    std::cout << "[CONFIG] CAN 250kbps set\n";

    CSVLogger logger;
    auto start_time = Clock::now();

    print_controls();

    // Start keyboard input thread
    std::thread kb_thread(keyboard_thread_fn);

    // RX buffer
    std::vector<uint8_t> buf;
    buf.reserve(1024);
    uint8_t chunk[256];
    std::set<uint32_t> unknown_ids;
    int frame_count = 0;

    auto last_send = Clock::now();

    while (!stop_flag.load()) {
        // ── RX ──
        size_t n = port.read_bytes(chunk, sizeof(chunk));
        if (n > 0) buf.insert(buf.end(), chunk, chunk + n);

        while (true) {
            ParsedFrame pf = find_and_parse_frame(buf);
            if (!pf.valid) break;
            ++frame_count;
            double ts = elapsed(start_time);
            std::string raw_hex;
            {
                std::ostringstream ss;
                for (int i = 0; i < pf.dlc; ++i) {
                    if (i) ss << ' ';
                    ss << std::uppercase << std::hex
                       << std::setw(2) << std::setfill('0') << (int)pf.payload[i];
                }
                raw_hex = ss.str();
            }

            if (pf.can_id == 0x14520902) {
                auto d = decode_0x14520902(pf.payload);
                std::cout << std::fixed << std::setprecision(3)
                          << "[" << ts << "s] M0x" << std::hex << (int)d.motor_id
                          << std::dec << " RPM=" << d.rpm
                          << " BattV=" << (d.bv_ok ? std::to_string(d.batt_v) : "N/A") << "V\n";
                logger.log_rpm_batt(ts, d.motor_id, d.rpm, d.throttle,
                                    d.batt_v, d.bv_ok, d.batt_i, d.bi_ok);

            } else if (pf.can_id == 0x18530904) {
                auto d = decode_0x18530904(pf.payload);
                std::cout << std::fixed << std::setprecision(3)
                          << "[" << ts << "s] M0x" << std::hex << (int)d.motor_id
                          << std::dec << " MCU=" << d.mcu_temp
                          << "C Mot=" << d.mot_temp << "C Ready=" << d.ready << "\n";
                logger.log_ac_temp(ts, d.motor_id, d.ac_v, d.av_ok,
                                   d.ac_i, d.ai_ok, d.mcu_temp, d.mot_temp, d.ready);

            } else if (pf.can_id == 0x1C530902) {
                auto d = decode_0x1C530902(pf.payload);
                if (d.faults != "None")
                    std::cout << "[⚠️  FAULT] M0x" << std::hex << (int)d.motor_id
                              << std::dec << " " << d.faults << "\n";
                logger.log_faults(ts, d.motor_id, d.status, d.faults, raw_hex);

            } else if (pf.can_id != CMD_CAN_ID) {
                if (!unknown_ids.count(pf.can_id)) {
                    unknown_ids.insert(pf.can_id);
                    std::cout << "[UNKNOWN] 0x" << std::hex << pf.can_id << "\n";
                }
            }
        }

        // ── TX (every SEND_INTERVAL) ──
        double since_send = std::chrono::duration<double>(
            Clock::now() - last_send).count();
        if (since_send >= SEND_INTERVAL) {
            double lin = g_linear.load();
            double ang = g_angular.load();
            send_drive(port, logger, lin, ang, start_time);
            last_send = Clock::now();
        }

        if (n == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Stop all motors
    std::cout << "\n[STOP] Sending zero RPM to all motors...\n";
    for (int i = 0; i < 20; ++i) {
        for (uint8_t mid : { 0xD1, 0xD2, 0xD3, 0xD4 }) {
            auto frame = build_tx_frame(mid, 0);
            port.write_bytes(frame.data(), frame.size());
            sleep_sec(0.01);
        }
    }

    kb_thread.join();
    double total = elapsed(start_time);
    std::cout << "[DONE] " << frame_count << " RX frames in "
              << std::fixed << std::setprecision(1) << total << "s\n";
    logger.save();
    port.close();
    return 0;
}
