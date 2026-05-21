#include "MagneticDeclination.hpp"
#include <iostream>

#include <chrono>

int main() {
    MagneticDeclination ob = MagneticDeclination();

    ob.LoadCOF("/home/idler/WMM.COF");
    double declination1 = ob.CalculateDeclination(240, -80, 100000.0, 2026.0);

    std::cout << "Expected: 1: 88.47 " << ", got: " << declination1 << std::endl; 


    return 0;
}

