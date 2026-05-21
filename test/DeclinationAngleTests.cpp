#include <gtest/gtest.h>
#include <cmath>

#include <cmath>
#include <iostream>

#include "MagneticDeclination.hpp"

class AssociatedLegendreDerivativesTest : public ::testing::Test {
protected:
    static constexpr int L = 12;
    static constexpr int M = 12;
    static constexpr double X = 0.4;
    static constexpr double REL_TOL = 1e-4;

    double* result;

    void SetUp() override {
        result = MagneticDeclination::CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(L, M, X);
    }

    void TearDown() override {
        delete[] result;
    }

    void ExpectNearRelative(double actual, double expected, int row, int col) {
        if (std::abs(expected) < 1e-10) {
            EXPECT_NEAR(actual, expected, 1e-10);
        } else {
            EXPECT_NEAR(actual / expected, 1.0, REL_TOL);
        }
    }
};

// TEST_F(AssociatedLegendreDerivativesTest, PDerivativeValuesMatchExpected) {
//     const double expected[12][13] = {
//         { 0.4,1,0,0,0,0,0,0,0,0,0,0,0 },
//         { -0.26,1.2,3,0,0,0,0,0,0,0,0,0,0},
//         { -0.44,2.7,6,15,0,0,0,0,0,0,0,0,0},
//         { -0.113,-1.88, 15.9, 42, 105, 0,0,0,0,0,0,0,0},
//         {0.27064, 7.083,31.08,128.1,378, 945, 0,0,0,0,0,0,0},
//         { 0.292636, 11.177, 62.013, 299.88, 1304.1, 4158, 10395,0,0,0,0,0,0},
//         { -0.0145904, 17.9503,114.222,678.069,3520.44,16008.3,54054,135135, 0,0,0,0,0},
//         { -0.2669993, 28.1597,207.241,1413.44,8866.94,48648.6,229730, 810810, 2.02702e6,0,0,0,0},
//         { -0.18876356, 44.1298,364.493,2845.03,20508.1,134730, 772972, 3.77027e6,1.37838e7,3.44594e7,0,0,0},
//         { 0.0968390644, 68.5537,631.225,5511.93,45188.6,341005, 2.33013e6,1.38757e7,6.9608e7, 2.61892e8,6.54729e8,0,0},
//         { 0.24555306736, 106.128,1075.13,10410.7,95242.5,814231, 6.38813e6,4.51625e7,2.77605e8,1.42731e9,5.49972e9,1.37493e10,0   },
//         { 0.99488209276, 163.547,1809.72,19216.2,194258, 1.84957e6,1.63972e7,1.33051e8,9.6913e8, 6.12303e9,3.21734e10,1.26494e11,3.16234e11 }
//     };

//     for (int l = 0; l < L; ++l) {
//         for (int m = 0; m <= M; ++m) {
//             std::cout << result[l * (M + 1) + m] << " " << expected[l][m] << std::endl;
//             ExpectNearRelative(result[l * (M + 1) + m], expected[l][m], l, m);
//         }
//     }
// }

TEST_F(AssociatedLegendreDerivativesTest, LegrandrePolynomialValuesMatchExpected) {
    double x = 0.2;
    int n = 6;
    int m = 5;

    MagneticDeclination::LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");

    MagneticDeclination::SetAssociatedPolynomialMatrix(x);

    double real = MagneticDeclination::associatedLegendrePolynomial(x, n, m);

    EXPECT_NEAR(-3.0036723353, real, 0.0001);
}
