#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include <boost/asio.hpp>

#include "utils.hpp"
#include "SerialComService.hpp"
#include "IMUSerialPortReader.hpp"
#include "DatabaseManager.hpp"
#include "gps/GpsManager.hpp"
#include "RadarPositionNavigationController.hpp"

int main(int argc, char** argv) {
    try {
        GpsManager gps = GpsManager();

        auto database = std::make_shared<DatabaseManager>("./imu.db");
        RadarPositionNavigationController r = RadarPositionNavigationController(database);

        gps.InstallCallback(r.GetGPSCallback());

        r.StartAndConfigureRadarPNT(30.27433302862909, -97.73442629917186);

        std::signal(SIGINT, [](int) {
            std::exit(EXIT_SUCCESS);
        });

        while (true) {
            std::cout << r.GetLastX()(0, 0) << " " << r.GetLastX()(1, 0) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

    } catch (const std::exception& e) {
        std::cout << "[ERROR] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
