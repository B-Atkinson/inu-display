#include "demo.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

extern "C" {
#include "sh2_SensorValue.h"
}

// ---------------------------------------------------------------------------
// CSV output files
// ---------------------------------------------------------------------------

static constexpr const char* kImuCsvPath  = "imu_measurements.csv";
static constexpr const char* kNmeaCsvPath = "nmea_sentences.csv";

// Change this if your GPS is on a different Pi serial port.
static constexpr const char* kNmeaPort = "/dev/ttyACM0";
static constexpr int         kNmeaBaud = 9600;

static std::atomic<bool> g_running{true};

// Protects the IMU CSV stream.
static std::mutex g_imu_csv_mtx;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

static bool csv_needs_header(const char* path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    return !in.good() || in.tellg() == std::streampos(0);
}

static double epoch_seconds_ms() {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto ms_since_epoch =
        duration_cast<milliseconds>(now.time_since_epoch());

    return static_cast<double>(ms_since_epoch.count()) / 1000.0;
}

static std::string csv_quote(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);

    out.push_back('"');

    for (char c : value) {
        if (c == '"') {
            out += "\"\"";
        } else if (c != '\r' && c != '\n') {
            out.push_back(c);
        }
    }

    out.push_back('"');
    return out;
}

// ---------------------------------------------------------------------------
// IMU per-measurement logger
// ---------------------------------------------------------------------------

static void write_imu_csv_header(std::ostream& csv) {
    csv << "epoch_s,"
        << "sensor_timestamp_us,"
        << "sensor_id,"
        << "sensor_name,"
        << "status,"
        << "accuracy,"
        << "x,"
        << "y,"
        << "z,"
        << "i,"
        << "j,"
        << "k,"
        << "real,"
        << "rotation_accuracy"
        << '\n';
}

static void write_imu_prefix(
    std::ostream& csv,
    const sh2_SensorValue_t& val,
    const char* sensor_name
) {
    const double epoch_s = epoch_seconds_ms();

    const int sensor_id = static_cast<int>(val.sensorId);
    const int status    = static_cast<int>(val.status);
    const int accuracy  = static_cast<int>(val.status & 0x03);

    csv << std::fixed << std::setprecision(3)
        << epoch_s << ',';

    csv << std::setprecision(9)
        << val.timestamp << ','
        << sensor_id << ','
        << csv_quote(sensor_name) << ','
        << status << ','
        << accuracy << ',';
}

static void log_imu_measurement_csv(const sh2_SensorValue_t& val) {
    std::lock_guard<std::mutex> lock(g_imu_csv_mtx);

    static std::ofstream csv;
    static bool initialized = false;

    if (!initialized) {
        const bool need_header = csv_needs_header(kImuCsvPath);

        csv.open(kImuCsvPath, std::ios::out | std::ios::app);
        if (!csv) {
            std::cerr << "[ERROR] Failed to open IMU CSV file: "
                      << kImuCsvPath << "\n";
            return;
        }

        if (need_header) {
            write_imu_csv_header(csv);
            csv.flush();
        }

        initialized = true;
    }

    switch (val.sensorId) {
        case SH2_ACCELEROMETER:
            write_imu_prefix(csv, val, "ACCELEROMETER");

            csv << val.un.accelerometer.x << ','
                << val.un.accelerometer.y << ','
                << val.un.accelerometer.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_LINEAR_ACCELERATION:
            write_imu_prefix(csv, val, "LINEAR_ACCELERATION");

            csv << val.un.linearAcceleration.x << ','
                << val.un.linearAcceleration.y << ','
                << val.un.linearAcceleration.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            write_imu_prefix(csv, val, "GYROSCOPE_CALIBRATED");

            csv << val.un.gyroscope.x << ','
                << val.un.gyroscope.y << ','
                << val.un.gyroscope.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            write_imu_prefix(csv, val, "MAGNETIC_FIELD_CALIBRATED");

            csv << val.un.magneticField.x << ','
                << val.un.magneticField.y << ','
                << val.un.magneticField.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_ROTATION_VECTOR:
            write_imu_prefix(csv, val, "ROTATION_VECTOR");

            csv << ",,,"
                << val.un.rotationVector.i << ','
                << val.un.rotationVector.j << ','
                << val.un.rotationVector.k << ','
                << val.un.rotationVector.real << ','
                << val.un.rotationVector.accuracy
                << '\n';
            break;

        case SH2_GAME_ROTATION_VECTOR:
            write_imu_prefix(csv, val, "GAME_ROTATION_VECTOR");

            csv << ",,,"
                << val.un.gameRotationVector.i << ','
                << val.un.gameRotationVector.j << ','
                << val.un.gameRotationVector.k << ','
                << val.un.gameRotationVector.real << ','
                << '\n';
            break;

        default:
            return;
    }

    // Immediate per-measurement write.
    csv.flush();
}

static void sensor_callback(void* /*cookie*/, sh2_SensorEvent_t* event) {
    sh2_SensorValue_t val{};

    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
        return;
    }

    // This is now the only IMU logging path.
    // Every decoded BNO085 report gets one CSV row immediately.
    log_imu_measurement_csv(val);
}

// ---------------------------------------------------------------------------
// NMEA / GPS per-sentence logger
// ---------------------------------------------------------------------------

static std::string trim_line_endings(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
        s.pop_back();
    }
    return s;
}

static std::string nmea_sentence_id(const std::string& sentence) {
    if (sentence.empty()) return "";

    size_t start = 0;
    if (sentence[0] == '$' || sentence[0] == '!') {
        start = 1;
    }

    size_t end = sentence.find(',', start);
    size_t star = sentence.find('*', start);

    if (end == std::string::npos || (star != std::string::npos && star < end)) {
        end = star;
    }

    if (end == std::string::npos) {
        end = sentence.size();
    }

    if (end <= start) return "";

    return sentence.substr(start, end - start);
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    return -1;
}

// Returns:
//   1  = checksum valid
//   0  = checksum invalid
//  -1  = no usable checksum found
static int nmea_checksum_ok(const std::string& sentence) {
    if (sentence.empty()) return -1;
    if (sentence[0] != '$' && sentence[0] != '!') return -1;

    const size_t star = sentence.find('*');
    if (star == std::string::npos || star + 2 >= sentence.size()) {
        return -1;
    }

    const int hi = hex_value(sentence[star + 1]);
    const int lo = hex_value(sentence[star + 2]);

    if (hi < 0 || lo < 0) {
        return -1;
    }

    const unsigned char expected =
        static_cast<unsigned char>((hi << 4) | lo);

    unsigned char actual = 0;

    for (size_t i = 1; i < star; ++i) {
        actual ^= static_cast<unsigned char>(sentence[i]);
    }

    return actual == expected ? 1 : 0;
}

static bool baud_to_speed(int baud, speed_t& speed) {
    switch (baud) {
        case 4800:   speed = B4800;   return true;
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;

#ifdef B230400
        case 230400: speed = B230400; return true;
#endif

#ifdef B460800
        case 460800: speed = B460800; return true;
#endif

#ifdef B921600
        case 921600: speed = B921600; return true;
#endif

        default:
            return false;
    }
}

static bool configure_serial_port(int fd, int baud) {
    speed_t speed{};

    if (!baud_to_speed(baud, speed)) {
        std::cerr << "[ERROR] Unsupported NMEA baud rate: " << baud << "\n";
        return false;
    }

    struct termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[ERROR] tcgetattr failed for NMEA serial port: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
        std::cerr << "[ERROR] Failed to set NMEA baud rate: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    // 8N1, raw serial.
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CREAD;

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed for NMEA serial port: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

static void write_nmea_csv_header(std::ostream& csv) {
    csv << "epoch_s,"
        << "port,"
        << "baud,"
        << "sentence_id,"
        << "checksum_ok,"
        << "nmea_sentence"
        << '\n';
}

static void log_nmea_sentence_csv(
    std::ostream& csv,
    const std::string& port,
    int baud,
    const std::string& sentence
) {
    const double epoch_s = epoch_seconds_ms();
    const std::string sentence_id = nmea_sentence_id(sentence);
    const int checksum_state = nmea_checksum_ok(sentence);

    csv << std::fixed << std::setprecision(3)
        << epoch_s << ','
        << csv_quote(port) << ','
        << baud << ','
        << csv_quote(sentence_id) << ','
        << checksum_state << ','
        << csv_quote(sentence)
        << '\n';

    // Immediate per-sentence write.
    csv.flush();
}

static void nmea_recorder_thread(
    std::string port,
    int baud,
    std::string csv_path
) {
    const int fd = ::open(
        port.c_str(),
        O_RDONLY | O_NOCTTY | O_NONBLOCK
    );

    if (fd < 0) {
        std::cerr << "[ERROR] Failed to open NMEA serial port "
                  << port << ": " << std::strerror(errno) << "\n";
        return;
    }

    if (!configure_serial_port(fd, baud)) {
        ::close(fd);
        return;
    }

    const bool need_header = csv_needs_header(csv_path.c_str());

    std::ofstream csv(csv_path, std::ios::out | std::ios::app);
    if (!csv) {
        std::cerr << "[ERROR] Failed to open NMEA CSV file: "
                  << csv_path << "\n";
        ::close(fd);
        return;
    }

    if (need_header) {
        write_nmea_csv_header(csv);
        csv.flush();
    }

    std::cerr << "[INFO] NMEA recorder started on "
              << port << " @ " << baud
              << " baud -> " << csv_path << "\n";

    std::string line;
    char buffer[256];

    while (g_running.load(std::memory_order_relaxed)) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        const int poll_result = ::poll(&pfd, 1, 100);

        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "[ERROR] poll failed on NMEA serial port: "
                      << std::strerror(errno) << "\n";
            break;
        }

        if (poll_result == 0) {
            continue;
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::cerr << "[ERROR] NMEA serial port disconnected or invalid\n";
            break;
        }

        const ssize_t n = ::read(fd, buffer, sizeof(buffer));

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }

            std::cerr << "[ERROR] read failed on NMEA serial port: "
                      << std::strerror(errno) << "\n";
            break;
        }

        if (n == 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const char ch = buffer[i];

            if (ch == '\n') {
                std::string sentence = trim_line_endings(line);
                line.clear();

                if (sentence.empty()) {
                    continue;
                }

                const size_t start = sentence.find_first_of("$!");
                if (start == std::string::npos) {
                    continue;
                }

                sentence.erase(0, start);

                if (sentence.empty()) {
                    continue;
                }

                // Every complete NMEA line gets one CSV row immediately.
                log_nmea_sentence_csv(csv, port, baud, sentence);
            } else if (ch != '\0') {
                if (line.size() < 1024) {
                    line.push_back(ch);
                } else {
                    line.clear();
                }
            }
        }
    }

    csv.flush();
    ::close(fd);

    std::cerr << "[INFO] NMEA recorder stopped\n";
}

// ---------------------------------------------------------------------------
// BNO085 helper
// ---------------------------------------------------------------------------

static bool enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;

    int last_rc = SH2_OK;

    for (int attempt = 1; attempt <= 20; ++attempt) {
        last_rc = sh2_setSensorConfig(sensor_id, &cfg);

        if (last_rc == SH2_OK) {
            std::cerr << "[INFO] Enabled sensor id="
                      << static_cast<int>(sensor_id)
                      << " interval_us=" << interval_us
                      << "\n";
            return true;
        }

        for (int i = 0; i < 5; ++i) {
            sh2_service();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::cerr << "[WARN] Failed to enable sensor id="
              << static_cast<int>(sensor_id)
              << " interval_us=" << interval_us
              << " rc=" << last_rc
              << "\n";

    return false;
}

// RAII wrapper: puts stdin into raw non-blocking mode, restores on destruction.
struct RawTerminal {
    struct termios saved{};

    RawTerminal() {
        tcgetattr(STDIN_FILENO, &saved);

        struct termios t = saved;
        t.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        t.c_cc[VMIN]  = 0;
        t.c_cc[VTIME] = 0;

        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    ~RawTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    }
};

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

void run_demo() {
    g_running.store(true, std::memory_order_relaxed);

    RawTerminal term;

    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        return;
    }

    sh2_setSensorCallback(sensor_callback, nullptr);

    // Let the BNO085/SH2 stack process startup traffic before enabling reports.
    for (int i = 0; i < 20; ++i) {
        sh2_service();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // These intervals are in microseconds.
    // 2,500 us  = 400 Hz request
    // 10,000 us = 100 Hz request
    enable_sensor(SH2_ACCELEROMETER,             2'500);
    enable_sensor(SH2_LINEAR_ACCELERATION,       2'500);
    enable_sensor(SH2_GYROSCOPE_CALIBRATED,      2'500);
    enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10'000);
    enable_sensor(SH2_ROTATION_VECTOR,           2'500);
    enable_sensor(SH2_GAME_ROTATION_VECTOR,      2'500);

    std::thread service_thread([]() {
        while (g_running.load(std::memory_order_relaxed)) {
            // This triggers sensor_callback(), which writes IMU CSV rows.
            sh2_service();
        }
    });

    std::thread nmea_thread(
        nmea_recorder_thread,
        std::string(kNmeaPort),
        kNmeaBaud,
        std::string(kNmeaCsvPath)
    );

    std::cout << "BNO085 per-measurement logging + NMEA recording — press Esc to stop\n";
    std::cout << "IMU CSV:  " << kImuCsvPath << "\n";
    std::cout << "NMEA CSV: " << kNmeaCsvPath << "\n\n";

    while (g_running.load(std::memory_order_relaxed)) {
        char ch = 0;

        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1b') {
            g_running.store(false, std::memory_order_relaxed);
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (service_thread.joinable()) {
        service_thread.join();
    }

    if (nmea_thread.joinable()) {
        nmea_thread.join();
    }

    sh2_close();

    std::cout << "\nShutdown complete.\n";
}