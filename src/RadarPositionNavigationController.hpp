#ifndef RADAR_POSITION_NAVIGATION_CONTROLLER_HPP
#define RADAR_POSITION_NAVIGATION_CONTROLLER_HPP

#include "IMUManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUGPSFusionKF.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class RadarPositionNavigationController {
public:
    /**
     * @brief Constructor for radar PNT self navigation. We assume all order position derivatives are 0.
     */
    RadarPositionNavigationController();

    ~RadarPositionNavigationController();

    /**
     * @brief Provides a callback for a GPS service to receive asyncronous data.
     * 
     * @return
     * 
     * @exception
     */
    std::function<void(const GpsUpdate&)> GetGPSCallback();

    void StartRadar

private:

    void StartIMUReader();
    

private:

    std::function<void(const GpsUpdate&)> m_gpsCallback;

    IMUManager m_imuManager;

    IMUGPSFusionKF_2D_ConstantAcceleration m_kf;
    

};

#endif // RADAR_POSITION_NAVIGATION_CONTROLLER_HPP