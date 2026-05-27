#ifndef CALIBRATION_H
#define CALIBRATION_H
#pragma once

#include <stdio.h>
#include <string.h>

extern "C" {
#include "sh2_SensorValue.h"
#include "sh2.h"
#include "sh2_err.h"
}

#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

typedef enum CalState_t
{
    CAL_WAIT_START,
    CAL_WAIT_FINISH,
    CAL_DONE,
} CalState_t;

class Calibration
{
public:
    Calibration();

    static void Service();

private:

    inline static const char * CAL_STATUS_MSG[] = {
        "Success",
        "No ZRO",
        "No stationary detection",
        "Rotation outside of specification",
        "ZRO outside of specification",
        "ZGO outside of specifidation",
        "Gyro gain outside of specification",
        "Gyro period outside of specification",
        "Gyro sample drops outside of specification",
    };

    static void Init();
    static void PrepareStep();
    static bool PressedEnter();
    static void ReportProdIds();
    static void ServiceCalibration();
    static void EventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);

    static bool m_sResetOccurred;
    static sh2_Hal_t m_sPSh2Hal;
    static CalState_t m_sCalState;
    static sh2_CalStatus_t m_sCalStatus;
};

#endif