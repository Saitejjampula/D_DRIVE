/**
 * CAN COMBINED: SENDER + RECEIVER → Excel Logger (CSV fallback)
 *
 * Build (Linux/Windows with MinGW):
 *   g++ -std=c++17 -O2 -pthread can_combined_xl.cpp -o can_logger
 *
 * Dependencies:
 *   - Windows: none (uses Win32 Serial API)
 *   - Linux  : none (uses POSIX termios)
 *   - Excel output: uses CSV (open in Excel) — link libxlsxwriter for .xlsx
 *
 * For native .xlsx output, install libxlsxwriter and replace CSVLogger
 * with the XlsxLogger variant shown at the bottom of this file.
 */

// ─────────────────────────────────────────────────────────────
//  PLATFORM DETECTION
// ─────────────────────────────────────────────────────────────
#ifdef _WIN32
  #define PLATFORM_WINDOWS
  #include <windows.h>
#else
  #define PLATFORM_LINUX
  #include <fcntl.h>
  #include <termios.h>
  #include <unistd.h>
#endif

#include <cstdint>
#include <cstring>
#include <ctime>
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
#include <functional>

// ─────────────────────────────────────────────────────────────
//  CONFIGURATION
// ─────────────────────────────────────────────────────────────
#ifdef PLATFORM_WINDOWS
  constexpr const char* TTY_DEVICE    = "\\\\.\\COM3";
#else
  constexpr const char* TTY_DEVICE    = "/dev/ttyUSB0";
#endif
constexpr int          USB_BAUD       = 2000000;
constexpr int          CAN_SPEED      = 250000;
constexpr const char*  LOG_FILE       = "can_log.xlsx";   // rename to .xlsx if using libxlsxwriter
constexpr uint32_t     CMD_CAN_ID     = 0x18530905;
constexpr double       SEND_INTERVAL  = 0.1;             // seconds
constexpr double       INTER_FRAME_DELAY = 0.001;        // seconds
constexpr int          TARGET_RPM     = 100;

const std::vector<uint8_t> MOTOR_IDS = { 0xD1, 0xD2, 0xD3, 0xD4 };

// ─────────────────────────────────────────────────────────────
//  SERIAL PORT (cross-platform)
// ─────────────────────────────────────────────────────────────
class SerialPort {
public:
#ifdef PLATFORM_WINDOWS
    HANDLE hPort = INVALID_HANDLE_VALUE;

    bool open(const char* dev, int baud) {
        hPort = CreateFileA(dev, GENERIC_READ | GENERIC_WRITE, 0,
                            nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
        if (hPort == INVALID_HANDLE_VALUE) return false;

        DCB dcb = {};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(hPort, &dcb);
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = TWOSTOPBITS;
        SetCommState(hPort, &dcb);

        COMMTIMEOUTS ct = { 10, 0, 10, 0, 0 };
        SetCommTimeouts(hPort, &ct);
        return true;
    }
    void write(const uint8_t* data, size_t len) {
        DWORD written;
        WriteFile(hPort, data, (DWORD)len, &written, nullptr);
    }
    size_t read(uint8_t* buf, size_t max_len) {
        DWORD rd = 0;
        ReadFile(hPort, buf, (DWORD)max_len, &rd, nullptr);
        return rd;
    }
    void close() { if (hPort != INVALID_HANDLE_VALUE) CloseHandle(hPort); }
#else
    int fd = -1;

    bool open(const char* dev, int baud) {
        fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;

        struct termios tty = {};
        tcgetattr(fd, &tty);
        speed_t spd = (baud == 2000000) ? B2000000 : B921600;
        cfsetispeed(&tty, spd);
        cfsetospeed(&tty, spd);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag |= CSTOPB;     // 2 stop bits
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8 | CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty);
        return true;
    }
    void write(const uint8_t* data, size_t len) {
        ::write(fd, data, len);
    }
    size_t read(uint8_t* buf, size_t max_len) {
        ssize_t n = ::read(fd, buf, max_len);
        return (n < 0) ? 0 : (size_t)n;
    }
    void close() { if (fd >= 0) ::close(fd); }
#endif
};

// ─────────────────────────────────────────────────────────────
//  TIMING HELPERS
// ─────────────────────────────────────────────────────────────
using Clock    = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline double elapsed(const TimePoint& start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
inline void sleep_sec(double s) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(s));
}

// ─────────────────────────────────────────────────────────────
//  CSV LOGGER  (import into Excel)
// ─────────────────────────────────────────────────────────────
class CSVLogger {
    std::mutex mtx;
    std::ofstream tx_f, rpm_f, ac_f, fault_f;

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

        tx_f    << "Timestamp (s),Motor ID,CAN ID,Target RPM,Raw Frame (Hex)\n";
        rpm_f   << "Timestamp (s),Motor ID,RPM,Throttle (%),Batt V (V),Batt I (A)\n";
        ac_f    << "Timestamp (s),Motor ID,AC Volt (V),AC Curr (A),MCU Temp (C),Motor Temp (C),Ready\n";
        fault_f << "Timestamp (s),Motor ID,Drive Status,Faults,Raw Hex\n";
    }

    void log_tx(double ts, uint8_t mid, int rpm, const std::string& raw) {
        std::lock_guard<std::mutex> lk(mtx);
        tx_f << std::fixed << std::setprecision(3) << ts
             << ",0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)mid
             << std::dec
             << ",0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << CMD_CAN_ID
             << std::dec << "," << rpm << "," << raw << "\n";
    }

    void log_rpm_batt(double ts, uint8_t mid, int rpm, int throttle,
                      double batt_v, bool bv_valid,
                      double batt_i, bool bi_valid) {
        std::lock_guard<std::mutex> lk(mtx);
        rpm_f << std::fixed << std::setprecision(3) << ts
              << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
              << std::dec << "," << rpm << "," << throttle
              << "," << na(batt_v, bv_valid)
              << "," << na(batt_i, bi_valid) << "\n";
    }

    void log_ac_temp(double ts, uint8_t mid,
                     double ac_v, bool av_valid,
                     double ac_i, bool ai_valid,
                     int mcu_temp, int mot_temp, const std::string& ready) {
        std::lock_guard<std::mutex> lk(mtx);
        ac_f << std::fixed << std::setprecision(3) << ts
             << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
             << std::dec
             << "," << na(ac_v, av_valid)
             << "," << na(ac_i, ai_valid)
             << "," << mcu_temp << "," << mot_temp << "," << ready << "\n";
    }

    void log_faults(double ts, uint8_t mid,
                    const std::string& status,
                    const std::string& faults,
                    const std::string& raw) {
        std::lock_guard<std::mutex> lk(mtx);
        fault_f << std::fixed << std::setprecision(3) << ts
                << ",0x" << std::hex << std::setw(2) << std::setfill('0') << (int)mid
                << std::dec
                << ",\"" << status << "\",\"" << faults << "\"," << raw << "\n";
    }

    void save() {
        std::lock_guard<std::mutex> lk(mtx);
        tx_f.flush(); rpm_f.flush(); ac_f.flush(); fault_f.flush();
        std::cout << "[SAVED] CSV logs written (4 files).\n";
    }
};

// ─────────────────────────────────────────────────────────────
//  TX FRAME BUILDER
// ─────────────────────────────────────────────────────────────
std::vector<uint8_t> build_tx_frame(uint8_t motor_id, int rpm) {
    int16_t rpm16 = (int16_t)std::max(-32768, std::min(32767, rpm));
    uint8_t can_id_le[4];
    can_id_le[0] = CMD_CAN_ID & 0xFF;
    can_id_le[1] = (CMD_CAN_ID >> 8)  & 0xFF;
    can_id_le[2] = (CMD_CAN_ID >> 16) & 0xFF;
    can_id_le[3] = (CMD_CAN_ID >> 24) & 0xFF;

    //  Frame layout (15 bytes):
    //  [0]    = 0xAA  (start)
    //  [1]    = 0xE8  (cmd / DLC=8 marker)
    //  [2-5]  = CAN ID little-endian
    //  [6]    = motor_id        \
    //  [7]    = rpm LSB          |
    //  [8]    = rpm MSB          |  8-byte CAN payload
    //  [9-13] = 0x00             |
    //  [14]   = 0x55  (end)     /

    std::vector<uint8_t> frame(15, 0);   // ← 15 bytes, not 14
    frame[0]  = 0xAA;
    frame[1]  = 0xE8;
    frame[2]  = can_id_le[0];
    frame[3]  = can_id_le[1];
    frame[4]  = can_id_le[2];
    frame[5]  = can_id_le[3];
    frame[6]  = motor_id;
    frame[7]  = (uint8_t)(rpm16 & 0xFF);
    frame[8]  = (uint8_t)((rpm16 >> 8) & 0xFF);
    // frame[9..13] = 0x00 (already zeroed)
    frame[14] = 0x55;                    // ← end byte at index 14
    return frame;
}

// ─────────────────────────────────────────────────────────────
//  HEX STRING HELPER
// ─────────────────────────────────────────────────────────────
std::string to_hex(const std::vector<uint8_t>& data, size_t len = 0) {
    std::ostringstream ss;
    size_t n = (len == 0) ? data.size() : std::min(len, data.size());
    for (size_t i = 0; i < n; ++i) {
        if (i) ss << ' ';
        ss << std::uppercase << std::hex
           << std::setw(2) << std::setfill('0') << (int)data[i];
    }
    return ss.str();
}

// ─────────────────────────────────────────────────────────────
//  RX FRAME PARSER
// ─────────────────────────────────────────────────────────────
constexpr size_t FRAME_LEN = 20;

struct ParsedFrame {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  payload[8];
    bool     valid;
};

ParsedFrame find_and_parse_frame(std::vector<uint8_t>& buf) {
    ParsedFrame pf = {};
    while (buf.size() >= FRAME_LEN) {
        size_t idx = std::string::npos;
        for (size_t i = 0; i + 3 < buf.size(); ++i) {
            if (buf[i] == 0xAA && buf[i+1] == 0x55 &&
                buf[i+2] == 0x01 && buf[i+3] == 0x02) {
                idx = i;
                break;
            }
        }
        if (idx == std::string::npos) {
            if (buf.size() > 3)
                buf.erase(buf.begin(), buf.end() - 3);
            return pf;
        }
        if (idx != 0)
            buf.erase(buf.begin(), buf.begin() + idx);
        if (buf.size() < FRAME_LEN)
            break;

        pf.can_id  = buf[5] | (buf[6] << 8) | (buf[7] << 16) | (buf[8] << 24);
        pf.dlc     = std::min((int)buf[9], 8);
        memcpy(pf.payload, &buf[10], pf.dlc);
        pf.valid   = true;
        buf.erase(buf.begin(), buf.begin() + FRAME_LEN);
        return pf;
    }
    return pf;
}

// ─────────────────────────────────────────────────────────────
//  SIGNAL DECODERS
// ─────────────────────────────────────────────────────────────
struct RpmBattData {
    uint8_t motor_id;
    uint16_t rpm;
    uint8_t  throttle;
    double   batt_v;     bool bv_valid;
    double   batt_i;     bool bi_valid;
};

RpmBattData decode_0x14520902(const uint8_t* d) {
    RpmBattData r = {};
    r.motor_id = d[0];
    r.rpm      = d[1] | (d[2] << 8);
    r.throttle = d[3];
    uint16_t bv_raw = d[4] | (d[5] << 8);
    int16_t  bi_raw = (int16_t)(d[6] | (d[7] << 8));
    r.batt_v   = bv_raw * 0.1; r.bv_valid = true;
    if (bi_raw == -32768) { r.bi_valid = false; }
    else { r.batt_i = bi_raw * 0.1; r.bi_valid = true; }
    return r;
}

struct AcTempData {
    uint8_t     motor_id;
    double      ac_v;   bool av_valid;
    double      ac_i;   bool ai_valid;
    int         mcu_temp, mot_temp;
    std::string ready;
};

AcTempData decode_0x18530904(const uint8_t* d) {
    AcTempData r = {};
    r.motor_id = d[0];
    uint16_t av_raw = d[1] | (d[2] << 8);
    int16_t  ai_raw = (int16_t)(d[3] | (d[4] << 8));
    r.av_valid = (av_raw != 0x8000); if (r.av_valid) r.ac_v = av_raw * 0.1;
    r.ai_valid = (ai_raw != -32768); if (r.ai_valid) r.ac_i = ai_raw * 0.1;
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
    for (int i = 0; i < 8; ++i) {
        if (val & (1 << i)) {
            if (skip_dash && std::string(names[i]) == "-") continue;
            if (!out.empty()) out += ", ";
            out += names[i];
        }
    }
    return out;
}

struct FaultData {
    uint8_t     motor_id;
    std::string status, faults;
};

FaultData decode_0x1C530902(const uint8_t* d) {
    FaultData r = {};
    r.motor_id = d[0];
    std::string s1 = active_bits(d[1], DRIVE_STATUS1);
    std::string s2 = active_bits(d[2], DRIVE_STATUS2);
    r.status   = (s1.empty() && s2.empty()) ? "None"
                 : (s1.empty() ? s2 : (s2.empty() ? s1 : s1 + ", " + s2));

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
//  SENDER THREAD
// ─────────────────────────────────────────────────────────────
std::atomic<bool> stop_flag{false};

void sender_thread_fn(SerialPort& port, int rpm,
                      CSVLogger& logger, const TimePoint& start) {
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> frames, stop_frames;
    for (auto mid : MOTOR_IDS) {
        frames.push_back({mid, build_tx_frame(mid, rpm)});
        stop_frames.push_back({mid, build_tx_frame(mid, 0)});
    }

    std::cout << "[TX] Sending " << rpm << " RPM to motors\n";

    while (!stop_flag.load()) {
        auto cycle_start = Clock::now();
        for (auto& [mid, frame] : frames) {
            port.write(frame.data(), frame.size());
            double ts = elapsed(start);
            std::ostringstream hex_ss;
            for (size_t i = 0; i < frame.size(); ++i) {
                if (i) hex_ss << ' ';
                hex_ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)frame[i];
            }
            logger.log_tx(ts, mid, rpm, hex_ss.str());
            sleep_sec(INTER_FRAME_DELAY);
        }
        double used = std::chrono::duration<double>(Clock::now() - cycle_start).count();
        double rem  = SEND_INTERVAL - used;
        if (rem > 0) sleep_sec(rem);
    }

    std::cout << "\n[TX] Sending STOP (0 RPM)...\n";
    for (int i = 0; i < 20; ++i)
        for (auto& [mid, frame] : stop_frames) {
            port.write(frame.data(), frame.size());
            sleep_sec(0.01);
        }
    std::cout << "[TX] Stopped.\n";
}

// ─────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────
int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "  CAN COMBINED: SENDER + RECEIVER  →  CSV Logger (Excel-ready)\n";
    std::cout << "  Device: " << TTY_DEVICE << " | CAN: " << CAN_SPEED << " bps\n";
    std::cout << "  Target: " << TARGET_RPM << " RPM | Log: " << LOG_FILE << "\n";
    std::cout << std::string(70, '=') << "\n\n";

    SerialPort port;
    if (!port.open(TTY_DEVICE, USB_BAUD)) {
        std::cerr << "[ERROR] Could not open serial port: " << TTY_DEVICE << "\n";
        return 1;
    }

    // Set CAN speed
    const uint8_t can_speed_cmd[] = { 'S', '5', '\r' }; // S5 = 250k
    port.write(can_speed_cmd, 3);
    sleep_sec(0.1);
    std::cout << "[CONFIG] CAN speed set to " << CAN_SPEED << " bps\n\n";

    CSVLogger logger;
    auto start_time = Clock::now();

    std::thread tx_thread(sender_thread_fn,
                          std::ref(port), TARGET_RPM,
                          std::ref(logger), std::ref(start_time));

    std::vector<uint8_t> buf;
    buf.reserve(1024);
    uint8_t chunk[256];
    int frame_count = 0;
    std::set<uint32_t> unknown_ids;

    std::cout << "[INFO] Press Ctrl+C to stop...\n\n";

    // Install Ctrl+C handler
#ifdef PLATFORM_WINDOWS
    SetConsoleCtrlHandler([](DWORD) -> BOOL {
        stop_flag.store(true); return TRUE;
    }, TRUE);
#else
    struct sigaction sa = {};
    // Simple flag-based; use sigaction in real code
#endif

    try {
        while (!stop_flag.load()) {
            size_t n = port.read(chunk, sizeof(chunk));
            if (n > 0)
                buf.insert(buf.end(), chunk, chunk + n);

            while (true) {
                ParsedFrame pf = find_and_parse_frame(buf);
                if (!pf.valid) break;

                ++frame_count;
                double ts = elapsed(start_time);

                // Build raw hex string
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
                              << std::dec << " RPM/BATT | RPM=" << d.rpm
                              << " | BattV=" << (d.bv_valid ? std::to_string(d.batt_v) : "N/A")
                              << "V\n";
                    logger.log_rpm_batt(ts, d.motor_id, d.rpm, d.throttle,
                                        d.batt_v, d.bv_valid,
                                        d.batt_i, d.bi_valid);

                } else if (pf.can_id == 0x18530904) {
                    auto d = decode_0x18530904(pf.payload);
                    std::cout << std::fixed << std::setprecision(3)
                              << "[" << ts << "s] M0x" << std::hex << (int)d.motor_id
                              << std::dec << " AC/TEMP | MCU=" << d.mcu_temp
                              << "C | Mot=" << d.mot_temp << "C | Ready=" << d.ready << "\n";
                    logger.log_ac_temp(ts, d.motor_id, d.ac_v, d.av_valid,
                                       d.ac_i, d.ai_valid,
                                       d.mcu_temp, d.mot_temp, d.ready);

                } else if (pf.can_id == 0x1C530902) {
                    auto d = decode_0x1C530902(pf.payload);
                    std::cout << std::fixed << std::setprecision(3)
                              << "[" << ts << "s] M0x" << std::hex << (int)d.motor_id
                              << std::dec << " FAULTS | " << d.status << "\n";
                    logger.log_faults(ts, d.motor_id, d.status, d.faults, raw_hex);

                } else if (pf.can_id == CMD_CAN_ID) {
                    // TX echo — already logged in sender thread

                } else {
                    if (unknown_ids.find(pf.can_id) == unknown_ids.end()) {
                        unknown_ids.insert(pf.can_id);
                        std::cout << std::fixed << std::setprecision(3)
                                  << "[" << ts << "s] UNKNOWN: 0x"
                                  << std::hex << pf.can_id << " | " << raw_hex << "\n";
                    }
                }
            }

            // Yield CPU if no data
            if (n == 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (...) {}

    stop_flag.store(true);
    tx_thread.join();

    double elapsed_total = elapsed(start_time);
    std::cout << "\n[STOP] " << frame_count << " RX frames in "
              << std::fixed << std::setprecision(1) << elapsed_total << "s\n";
    logger.save();
    port.close();
    return 0;
}

/*
 * ─────────────────────────────────────────────────────────────
 *  HOW TO GET NATIVE .xlsx OUTPUT
 * ─────────────────────────────────────────────────────────────
 *
 * Install libxlsxwriter:
 *   vcpkg install libxlsxwriter      (Windows)
 *   sudo apt install libxlsxwriter-dev   (Ubuntu)
 *
 * Then replace CSVLogger with an XlsxLogger class that wraps:
 *   lxw_workbook*  wb  = workbook_new("can_log.xlsx");
 *   lxw_worksheet* ws  = workbook_add_worksheet(wb, "RPM & Battery");
 *   worksheet_write_string(ws, row, col, value, format);
 *   workbook_close(wb);
 *
 * All decode logic and threading remains identical.
 * ─────────────────────────────────────────────────────────────
 */