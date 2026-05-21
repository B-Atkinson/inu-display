#include "MagneticDeclination.hpp"
#include <iostream>

#include <chrono>

// int main() {
//     double* arr = MagneticDeclination::CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 0.2);

//     for (int i = 0; i < 12; i++) {
//         for (int j = 0; j < 13; j++) {
//             std::cout << arr[i * 13 + j] << " " ;
//         }
//         std::cout << std::endl;
//     }

//     // auto start = std::chrono::high_resolution_clock::now();

//     // for (int i = 0; i < 1000; i++) {
//     //     double* arr = MagneticDeclination::CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 0.4);
//     // }

//     // auto end = std::chrono::high_resolution_clock::now();
//     // double avg = std::chrono::duration<double, std::micro>(end - start).count() / 1000.0;
//     // std::cout << "Average time: " << avg << " us\n";

//     return 0;
// }

#include <iostream>
#include "MagneticDeclination.hpp"

int main() {
    MagneticDeclination::LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");
    double declination1 = MagneticDeclination::declination(-121.0, -99.77, 89.0, 2026.0);

    // std::cout << "Expected: 1: 88.47 " << ", got: " << declination1 << std::endl; 

    // double declination2 = MagneticDeclination::declination(-12.0, 39.77, 19.0, 2026.0);

    for (int i = 0; i < 50; i++) {
        std::cout << MagneticDeclination::declination(-12.0 + i, 39.77 - i, 19.0 + i, 2026.0) << std::endl;
    }

    return 0;
}

