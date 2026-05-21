#include "MagneticDeclination.hpp"
#include <iostream>

#include <chrono>

// int main() {
//     MagneticDeclination ob = MagneticDeclination();
//     ob.CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 0.2);

//     // for (int i = 0; i < 13; i++) {
//     //     for (int j = 0; j < 13; j++) {
//     //         std::cout << arr[i * 13 + j] << " " ;
//     //     }
//     //     std::cout << std::endl;
//     // }

//     // auto start = std::chrono::high_resolution_clock::now();

//     // for (int i = 0; i < 1000; i++) {
//     //     double* arr = MagneticDeclination::CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 0.4);
//     // }

//     // auto end = std::chrono::high_resolution_clock::now();
//     // double avg = std::chrono::duration<double, std::micro>(end - start).count() / 1000.0;
//     // std::cout << "Average time: " << avg << " us\n";

//     return 0;
// }

int main() {
    MagneticDeclination ob = MagneticDeclination();

    ob.LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");
    double declination1 = ob.declination(-121.0, -99.77, 89.0, 2026.0);

    std::cout << "Expected: 1: 88.47 " << ", got: " << declination1 << std::endl; 


    return 0;
}

