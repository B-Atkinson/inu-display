#include "MagneticDeclination.hpp"
#include "Calibration.hpp"
#include "demo.hpp"
#include <iostream>

#include <chrono>

int main() {
    Calibration calibration;

    while (true)
    {
        calibration.Service();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // MagneticDeclination ob = MagneticDeclination();

    // ob.LoadCOF("/home/idler/WMM.COF");
    // double declination1 = ob.CalculateDeclination(240, -80, 100000.0, 2026.0);

    // std::cout << "Expected: 1: 88.47 " << ", got: " << declination1 << std::endl; 

    // run_demo();
    return 0;
}

