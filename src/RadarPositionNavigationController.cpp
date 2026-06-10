#include "RadarPositionNavigationController.hpp"
#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

extern "C" {
#include "sh2_SensorValue.h"
}

static void enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;
    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        std::cerr << "[WARN] Failed to enable sensor id=" << sensor_id << "\n";
    }
}

RadarPositionNavigationController::RadarPositionNavigationController() {
    Vector6d x0;
    x0 << lon0, lat0, 1e-15, 1e-15, 1e-16, 1e-16;

    Matrix6d P0 = Matrix6d::Zero();
    P0(0, 0) = 1e-10;
    P0(1, 1) = 1e-10;
    P0(2, 2) = 1e-8;
    P0(3, 3) = 1e-8;
    P0(4, 4) = 1e-10;
    P0(5, 5) = 1e-10;

    Matrix6d R0_GPS = Matrix6d::Zero();
    R0_GPS(0, 0) = 1e-10;
    R0_GPS(1, 1) = 1e-10;

    Matrix6d R0_IMU = Matrix6d::Zero();
    R0_IMU(2, 2) = 1e-8;
    R0_IMU(3, 3) = 1e-8;
    R0_IMU(4, 4) = 1e-10;
    R0_IMU(5, 5) = 1e-10;

    Matrix6d Q0 = Matrix6d::Zero();
    Q0(0, 0) = 1e-14;
    Q0(1, 1) = 1e-14;
    Q0(2, 2) = 1e-12;
    Q0(3, 3) = 1e-12;
    Q0(4, 4) = 1e-14;
    Q0(5, 5) = 1e-14;

    double chiSquaredBetaLowerBound_GPS = 0.05; // Chi SQ 5% for df=2
    double chiSquaredBetaUpperBound_GPS = 5.99; // Chi SQ 95% for df=2

    double chiSquaredBetaLowerBound_IMU = 0.71; // Chi SQ 5% for df=2
    double chiSquaredBetaUpperBound_IMU = 9.49; // Chi SQ 95% for df=4

    unsigned N_GPS = 10;
    unsigned L_GPS = 2;

    unsigned N_IMU = 100;
    unsigned L_IMU = 10;

    unsigned N_Q = 100;
    unsigned L_Q = 10;

    this->m_kf = IMUGPSFusionKF_2D_ConstantAcceleration(
        x0,
        P0,
        R0_GPS,
        R0_IMU,
        Q0,
        chiSquaredBetaLowerBound_GPS,
        chiSquaredBetaLowerBound_IMU,
        chiSquaredBetaUpperBound_GPS,
        chiSquaredBetaUpperBound_IMU,
        N_GPS,
        L_GPS,
        N_IMU,
        L_IMU,
        N_Q,
        L_Q
    );



    StartIMUReader();
}

RadarPositionNavigationController::~RadarPositionNavigationController() {

}

void RadarPositionNavigationController::StartIMUReader() {
    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        return;
    }

    sh2_setSensorCallback(IMUManager::SensorCallback, nullptr);

    enable_sensor(SH2_LINEAR_ACCELERATION, 2'500);
    enable_sensor(SH2_ROTATION_VECTOR, 2'500);

    std::thread service_thread([]() {
        while (g_running) {
            sh2_service();
        }
    });
}


