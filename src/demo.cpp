#include "demo.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>

#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

static constexpr const char* kNmeaPort    = "/dev/serial0";          // Pi UART alias
static constexpr int         kNmeaBaud    = 9600;                    // Common GPS NMEA default
static constexpr const char* kNmeaCsvPath = "nmea_sentences.csv";

extern "C" {
#include "sh2_SensorValue.h"
}

#include "imu_data.hpp"

// ---------------------------------------------------------------------------
// Shared IMU snapshot — updated by sensor_callback, read by the print loop
// ---------------------------------------------------------------------------

struct Snapshot {
    std::mutex             mtx;
    AccelerometerData      accel{};
    LinearAccelData        linear_accel{};
    GyroscopeData          gyro{};
    MagnetometerData       mag{};
    RotationVectorData     rot{};
    GameRotationVectorData game_rot{};
};

static Snapshot        g_snapshot;
static std::atomic<bool> g_running{true};

// ---------------------------------------------------------------------------
// Sensor callback — fires on the service thread for every decoded report
// ---------------------------------------------------------------------------

static bool csv_needs_header(const char* path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    return !in.good() || in.tellg() == std::streampos(0);
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

    // Non-blocking-ish read behavior.
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

static void enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;
    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        std::cerr << "[WARN] Failed to enable sensor id=" << sensor_id << "\n";
    }
}

static constexpr const char* kSnapshotCsvPath = "imu_snapshot.csv";

static double epoch_seconds_ms() {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto ms_since_epoch =
        duration_cast<milliseconds>(now.time_since_epoch());

    // Example output when printed with precision 3:
    // 1716400000.123
    return static_cast<double>(ms_since_epoch.count()) / 1000.0;
}

static void write_snapshot_csv_header(std::ostream& csv) {
    csv
        << "epoch_s,"

        << "accel_timestamp_us,"
        << "accel_x,"
        << "accel_y,"
        << "accel_z,"
        << "accel_accuracy,"

        << "linear_accel_timestamp_us,"
        << "linear_accel_x,"
        << "linear_accel_y,"
        << "linear_accel_z,"
        << "linear_accel_accuracy,"

        << "gyro_timestamp_us,"
        << "gyro_x,"
        << "gyro_y,"
        << "gyro_z,"

        << "mag_timestamp_us,"
        << "mag_x,"
        << "mag_y,"
        << "mag_z,"
        << "mag_accuracy,"

        << "rot_timestamp_us,"
        << "rot_i,"
        << "rot_j,"
        << "rot_k,"
        << "rot_real,"
        << "rot_accuracy,"

        << "game_rot_timestamp_us,"
        << "game_rot_i,"
        << "game_rot_j,"
        << "game_rot_k,"
        << "game_rot_real"
        << '\n';
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

                // If there is noise before the NMEA start char, discard it.
                const size_t start = sentence.find_first_of("$!");
                if (start == std::string::npos) {
                    continue;
                }

                sentence.erase(0, start);

                if (sentence.empty()) {
                    continue;
                }

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

                csv.flush();
            } else if (ch != '\0') {
                // Prevent runaway growth if a cable/device sends junk without newlines.
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

static void sensor_callback(void* /*cookie*/, sh2_SensorEvent_t* event) {
    sh2_SensorValue_t val{};
    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) return;

    std::lock_guard<std::mutex> lock(g_snapshot.mtx);

    switch (val.sensorId) {
        case SH2_ACCELEROMETER:
            g_snapshot.accel.timestamp_us = val.timestamp;
            g_snapshot.accel.x            = val.un.accelerometer.x;
            g_snapshot.accel.y            = val.un.accelerometer.y;
            g_snapshot.accel.z            = val.un.accelerometer.z;
            g_snapshot.accel.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_LINEAR_ACCELERATION:
            g_snapshot.linear_accel.timestamp_us = val.timestamp;
            g_snapshot.linear_accel.x            = val.un.linearAcceleration.x;
            g_snapshot.linear_accel.y            = val.un.linearAcceleration.y;
            g_snapshot.linear_accel.z            = val.un.linearAcceleration.z;
            g_snapshot.linear_accel.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            g_snapshot.gyro.timestamp_us = val.timestamp;
            g_snapshot.gyro.x            = val.un.gyroscope.x;
            g_snapshot.gyro.y            = val.un.gyroscope.y;
            g_snapshot.gyro.z            = val.un.gyroscope.z;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            g_snapshot.mag.timestamp_us = val.timestamp;
            g_snapshot.mag.x            = val.un.magneticField.x;
            g_snapshot.mag.y            = val.un.magneticField.y;
            g_snapshot.mag.z            = val.un.magneticField.z;
            g_snapshot.mag.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_ROTATION_VECTOR:
            g_snapshot.rot.timestamp_us = val.timestamp;
            g_snapshot.rot.i            = val.un.rotationVector.i;
            g_snapshot.rot.j            = val.un.rotationVector.j;
            g_snapshot.rot.k            = val.un.rotationVector.k;
            g_snapshot.rot.real         = val.un.rotationVector.real;
            g_snapshot.rot.accuracy     = val.un.rotationVector.accuracy;
            break;

        case SH2_GAME_ROTATION_VECTOR:
            g_snapshot.game_rot.timestamp_us = val.timestamp;
            g_snapshot.game_rot.i            = val.un.gameRotationVector.i;
            g_snapshot.game_rot.j            = val.un.gameRotationVector.j;
            g_snapshot.game_rot.k            = val.un.gameRotationVector.k;
            g_snapshot.game_rot.real         = val.un.gameRotationVector.real;
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void print_snapshot() {
    Snapshot snap_copy;

    {
        std::lock_guard<std::mutex> lock(g_snapshot.mtx);
        snap_copy.accel       = g_snapshot.accel;
        snap_copy.linear_accel = g_snapshot.linear_accel;
        snap_copy.gyro        = g_snapshot.gyro;
        snap_copy.mag         = g_snapshot.mag;
        snap_copy.rot         = g_snapshot.rot;
        snap_copy.game_rot    = g_snapshot.game_rot;
    }

    const double epoch_s = epoch_seconds_ms();

    static std::ofstream csv;

    if (!csv.is_open()) {
        const bool need_header = csv_needs_header(kSnapshotCsvPath);

        csv.open(kSnapshotCsvPath, std::ios::out | std::ios::app);
        if (!csv) {
            std::cerr << "[ERROR] Failed to open CSV file: "
                      << kSnapshotCsvPath << "\n";
            return;
        }

        if (need_header) {
            write_snapshot_csv_header(csv);
        }
    }

    csv << std::fixed << std::setprecision(3)
        << epoch_s << ',';

    csv << std::setprecision(9)

        << snap_copy.accel.timestamp_us << ','
        << snap_copy.accel.x << ','
        << snap_copy.accel.y << ','
        << snap_copy.accel.z << ','
        << static_cast<int>(snap_copy.accel.accuracy) << ','

        << snap_copy.linear_accel.timestamp_us << ','
        << snap_copy.linear_accel.x << ','
        << snap_copy.linear_accel.y << ','
        << snap_copy.linear_accel.z << ','
        << static_cast<int>(snap_copy.linear_accel.accuracy) << ','

        << snap_copy.gyro.timestamp_us << ','
        << snap_copy.gyro.x << ','
        << snap_copy.gyro.y << ','
        << snap_copy.gyro.z << ','

        << snap_copy.mag.timestamp_us << ','
        << snap_copy.mag.x << ','
        << snap_copy.mag.y << ','
        << snap_copy.mag.z << ','
        << static_cast<int>(snap_copy.mag.accuracy) << ','

        << snap_copy.rot.timestamp_us << ','
        << snap_copy.rot.i << ','
        << snap_copy.rot.j << ','
        << snap_copy.rot.k << ','
        << snap_copy.rot.real << ','
        << snap_copy.rot.accuracy << ','

        << snap_copy.game_rot.timestamp_us << ','
        << snap_copy.game_rot.i << ','
        << snap_copy.game_rot.j << ','
        << snap_copy.game_rot.k << ','
        << snap_copy.game_rot.real
        << '\n';

    csv.flush();
}

// RAII wrapper: puts stdin into raw non-blocking mode, restores on destruction
struct RawTerminal {
    struct termios saved{};

    RawTerminal() {
        tcgetattr(STDIN_FILENO, &saved);
        struct termios t = saved;
        t.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        t.c_cc[VMIN]  = 0;  // non-blocking: return immediately
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
    g_running = true;

    RawTerminal term;

    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        return;
    }

    sh2_setSensorCallback(sensor_callback, nullptr);

    enable_sensor(SH2_ACCELEROMETER,             2'500);
    enable_sensor(SH2_LINEAR_ACCELERATION,       2'500);
    enable_sensor(SH2_GYROSCOPE_CALIBRATED,      2'500);
    enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10'000);
    enable_sensor(SH2_ROTATION_VECTOR,           2'500);
    enable_sensor(SH2_GAME_ROTATION_VECTOR,      2'500);

    std::thread service_thread([]() {
        while (g_running.load(std::memory_order_relaxed)) {
            sh2_service();
        }
    });

    std::thread nmea_thread(
        nmea_recorder_thread,
        std::string(kNmeaPort),
        kNmeaBaud,
        std::string(kNmeaCsvPath)
    );

    int PRINT_INTERVAL = 1000;

    std::cout << "BNO085 streaming + NMEA recording — press Esc to stop\n\n";

    using clock = std::chrono::steady_clock;
    auto next_print = clock::now() + std::chrono::milliseconds(PRINT_INTERVAL);

    while (g_running.load(std::memory_order_relaxed)) {
        char ch = 0;
        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1b') {
            g_running = false;
            break;
        }

        auto now = clock::now();
        if (now >= next_print) {
            print_snapshot();
            next_print = now + std::chrono::milliseconds(PRINT_INTERVAL);
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