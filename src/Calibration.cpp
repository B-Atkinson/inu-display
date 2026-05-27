#include "Calibration.hpp"

sh2_Hal_t Calibration::m_sPSh2Hal;
sh2_CalStatus_t Calibration::m_sCalStatus;
bool Calibration::m_sResetOccurred = false;
CalState_t Calibration::m_sCalState = CAL_WAIT_START;

Calibration::Calibration()
{
    Init();
}

void Calibration::Init()
{
    int status;
    
    printf("\n\n");
    printf("CEVA FSP200 Calibration Demo.\n");
    
    // Create HAL instance
    m_sPSh2Hal = bno085_hal_create();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(&m_sPSh2Hal, Calibration::EventHandler, NULL);
    if (status != SH2_OK)
    {
        printf("Error, %d, from sh2_open.\n", status);
    }

    // resetOccurred would have been set earlier.
    // We can reset it since we are starting the sensor reports now.
    m_sResetOccurred = false;

    // Read and display sensor hub product ids
    ReportProdIds();

    // Init calibration process
    PrepareStep();
}

void Calibration::Service()
{
    if (m_sResetOccurred) {
        // reinit calibration process
        PrepareStep();
    }

    // Service the sensor hub.
    sh2_service();

    // Service calibration state machine
    ServiceCalibration();

    if (m_sCalState == CAL_DONE)
    {
        // Restart calibration each time it finishes.
        PrepareStep();
    }
}

void Calibration::EventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET) {
        // Set flag indicating we saw a reset.
        m_sResetOccurred = true;
    }
}

void Calibration::ReportProdIds()
{
    sh2_ProductIds_t prodIds;
    int status;
     
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    
    if (status < 0) {
        printf("Error from sh2_getProdIds.\n");
        return;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }
}

void Calibration::PrepareStep()
{
    printf("Put module in start orientation, press ENTER.\n");
    printf("> ");

    // Set state
    m_sCalStatus = SH2_CAL_SUCCESS;
    m_sCalState = CAL_WAIT_START;
}

bool Calibration::PressedEnter()
{
    char c = getchar();
    if (c == '\n')
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Calibration::ServiceCalibration()
{
    switch (m_sCalState)
    {
        case CAL_WAIT_START:
            // Waiting for user to press ENTER
            if (PressedEnter())
            {
                uint32_t interval_us = 10000; // calibrate for 100Hz operation
                int status = sh2_startCal(interval_us);
                if (status != SH2_OK)
                {
                    // End calibration process with error
                    printf("Error from sh2_startCal: %d\n", status);
                    m_sCalState = CAL_DONE;
                }
                else
                {
                    // Proceed to next step: final orientation
                    printf("Rotate module to final orientation, press ENTER.\n");
                    printf("> ");
                    m_sCalState = CAL_WAIT_FINISH;
                }
            }
            break;
        case CAL_WAIT_FINISH:
            // Waiting for user to press ENTER at final orientation
            if (PressedEnter())
            {
                int status = sh2_finishCal(&m_sCalStatus);
                if (status != SH2_OK)
                {
                    // End calibration process with error
                    printf("Error from sh2_finishCal: %d\n", status);
                }
                else if (m_sCalStatus != 0)
                {
                    // Calibration process ended
                    printf("Calibration completed with status: %d\n", m_sCalStatus);
                    printf("%s\n", CAL_STATUS_MSG[m_sCalStatus]);
                }
                else
                {
                    printf("Calibration completed successfully.\n");
                }
                m_sCalState = CAL_DONE;
            }
            break;
        case CAL_DONE:
            break;
    }
}